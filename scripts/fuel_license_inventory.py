"""Resolve stable license metadata for Gazebo Fuel world dependencies."""

from __future__ import annotations

import json
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable
from urllib.error import HTTPError, URLError
from urllib.parse import quote, unquote, urlencode, urlparse
from urllib.request import Request, urlopen


class LicenseInventoryError(RuntimeError):
    """Raised when Fuel licensing cannot be pinned without guessing."""


_LICENSES = {
    "Creative Commons Attribution 4.0 International": (
        "CC-BY-4.0",
        "https://creativecommons.org/licenses/by/4.0/",
    ),
    "Creative Commons Zero v1.0 Universal": (
        "CC0-1.0",
        "https://creativecommons.org/publicdomain/zero/1.0/",
    ),
    "Apache License 2.0": (
        "Apache-2.0",
        "https://www.apache.org/licenses/LICENSE-2.0",
    ),
}


@dataclass(frozen=True, order=True)
class FuelResourceRequest:
    kind: str
    owner: str
    name: str
    version: int

    @property
    def url(self) -> str:
        resource_type = "worlds" if self.kind == "world" else "models"
        return (
            "https://fuel.gazebosim.org/1.0/"
            f"{quote(self.owner, safe='')}/{resource_type}/{quote(self.name, safe='')}"
        )


def requests_from_materialization_report(
    source_url: str,
    source_version: int,
    materialization_report: dict[str, Any],
) -> list[FuelResourceRequest]:
    world = _request_from_url("world", source_url, source_version)
    models = {
        _request_from_cached_model_path(path)
        for path in materialization_report.get("model_files", [])
    }
    return [world, *sorted(models)]


def resolve_inventory(
    environment_id: str,
    requests: Iterable[FuelResourceRequest],
    workers: int = 8,
) -> dict[str, Any]:
    ordered = sorted(set(requests))
    if not ordered:
        raise LicenseInventoryError("Fuel license inventory has no resources")
    with ThreadPoolExecutor(max_workers=max(1, workers)) as executor:
        resources = list(executor.map(_resolve_resource, ordered))
    resources.sort(key=lambda resource: (resource["kind"], resource["url"]))
    return {
        "schema": "drone_city_nav_fuel_license_inventory_v1",
        "environment_id": environment_id,
        "resources": resources,
    }


def write_inventory(path: Path, inventory: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    resources = inventory["resources"]
    lines = [
        "{",
        f'  "environment_id": {json.dumps(inventory["environment_id"])},',
        '  "resources": [',
    ]
    for index, resource in enumerate(resources):
        suffix = "," if index + 1 < len(resources) else ""
        lines.append(f"    {json.dumps(resource, sort_keys=True)}{suffix}")
    lines.extend(
        [
            "  ],",
            f'  "schema": {json.dumps(inventory["schema"])}',
            "}",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def summarize_licenses(inventory: dict[str, Any]) -> dict[str, int]:
    summary: dict[str, int] = {}
    for resource in inventory.get("resources", []):
        spdx = resource["license"]["spdx"]
        summary[spdx] = summary.get(spdx, 0) + 1
    return dict(sorted(summary.items()))


def _resolve_resource(request: FuelResourceRequest) -> dict[str, Any]:
    endpoint = f"{request.url}?{urlencode({'version': request.version})}"
    payload: dict[str, Any] | None = None
    last_error: Exception | None = None
    for attempt in range(3):
        try:
            http_request = Request(
                endpoint,
                headers={
                    "Accept": "application/json",
                    "User-Agent": "drone-city-nav-environment-tooling/1",
                },
            )
            with urlopen(http_request, timeout=30) as response:
                payload = json.load(response)
            break
        except (HTTPError, URLError, TimeoutError, json.JSONDecodeError) as exc:
            last_error = exc
            if attempt < 2:
                time.sleep(0.5 * (attempt + 1))
    if payload is None:
        raise LicenseInventoryError(
            f"cannot resolve Fuel metadata for {request.url} version "
            f"{request.version}: {last_error}"
        )
    returned_version = payload.get("version")
    if returned_version != request.version:
        raise LicenseInventoryError(
            f"Fuel returned version {returned_version} for {request.url}, "
            f"expected {request.version}"
        )
    license_name = payload.get("license_name")
    license_contract = _LICENSES.get(license_name)
    if license_contract is None:
        raise LicenseInventoryError(
            f"unsupported Fuel license for {request.url}: {license_name!r}"
        )
    spdx, canonical_url = license_contract
    return {
        "kind": request.kind,
        "name": payload.get("name", request.name),
        "owner": payload.get("owner", request.owner),
        "url": request.url,
        "version": request.version,
        "license": {
            "spdx": spdx,
            "name": license_name,
            "url": canonical_url,
        },
    }


def _request_from_url(
    kind: str, source_url: str, version: int
) -> FuelResourceRequest:
    parsed = urlparse(source_url)
    parts = [unquote(part) for part in parsed.path.split("/") if part]
    resource_word = "worlds" if kind == "world" else "models"
    try:
        resource_index = [part.lower() for part in parts].index(resource_word)
    except ValueError as exc:
        raise LicenseInventoryError(f"invalid Fuel resource URL: {source_url}") from exc
    if resource_index == 0 or resource_index + 1 >= len(parts):
        raise LicenseInventoryError(f"invalid Fuel resource URL: {source_url}")
    return FuelResourceRequest(
        kind=kind,
        owner=parts[resource_index - 1],
        name=parts[resource_index + 1],
        version=version,
    )


def _request_from_cached_model_path(recorded_path: str) -> FuelResourceRequest:
    path = Path(recorded_path)
    parts = list(path.parts)
    try:
        model_index = [part.lower() for part in parts].index("models")
    except ValueError as exc:
        raise LicenseInventoryError(
            f"model path is not a Fuel cache path: {recorded_path}"
        ) from exc
    if model_index == 0 or model_index + 2 >= len(parts):
        raise LicenseInventoryError(f"invalid Fuel model path: {recorded_path}")
    version_text = parts[model_index + 2]
    if not version_text.isdigit():
        raise LicenseInventoryError(f"invalid Fuel model version: {recorded_path}")
    return FuelResourceRequest(
        kind="model",
        owner=unquote(parts[model_index - 1]),
        name=unquote(parts[model_index + 1]),
        version=int(version_text),
    )
