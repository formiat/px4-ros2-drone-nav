#!/usr/bin/env python3
"""Prepare one distributed environment for the shared Gazebo simulation runtime."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import shlex
import subprocess
import sys
from pathlib import Path
from urllib.parse import quote
from urllib.request import Request, urlopen
from xml.etree import ElementTree as ET

from compile_environment_topology import (
    default_output_path,
    resolve_static_map_inputs,
    select_static_map,
)
from environment_manifest import (
    ManifestError,
    find_environment,
    load_manifest,
    repository_root,
)
from sdf_collision_materializer import (
    CollisionWorldMaterializer,
    MaterializationError,
    ResourceResolver,
    validate_visual_resource_uris,
    write_materialized_world,
    write_report,
)


DEFAULT_MANIFEST = Path("environments/environment_manifest.yaml")
DEFAULT_INSTALL_ROOT = Path("external/environment-artifacts/installed")


class EnvironmentPreparationError(RuntimeError):
    """Raised when an environment cannot satisfy the simulation contract."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--environment", required=True)
    parser.add_argument("--static-map")
    parser.add_argument(
        "--scenario",
        type=Path,
        help="Optional scenario whose physical launch platforms are materialized.",
    )
    parser.add_argument("--asset-install-root", type=Path, default=DEFAULT_INSTALL_ROOT)
    parser.add_argument("--rebuild-topology", action="store_true")
    return parser.parse_args()


def ensure_release_artifact(
    repository: Path,
    manifest_path: Path,
    environment: dict,
    artifact_id: str,
    install_root: Path,
) -> Path:
    destination = install_root / environment["id"] / artifact_id
    if destination.is_dir():
        return destination
    completed = subprocess.run(
        [
            sys.executable,
            str(repository / "scripts/manage_environment_assets.py"),
            "--manifest",
            str(manifest_path),
            "fetch",
            "--environment",
            environment["id"],
            "--artifact",
            artifact_id,
            "--install-dir",
            str(install_root),
        ],
        cwd=repository,
        check=False,
    )
    if completed.returncode != 0 or not destination.is_dir():
        raise EnvironmentPreparationError(
            f"failed to install {environment['id']} artifact {artifact_id}"
        )
    return destination


def source_model_paths(source_root: Path) -> list[Path]:
    paths = sorted(source_root.glob("fuel/*/*/models"))
    return [path for path in paths if path.is_dir()]


def repository_path(repository: Path, path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(repository).as_posix()
    except ValueError:
        return str(resolved)


def write_runtime_environment(
    path: Path,
    repository: Path,
    world_name: str,
    collision_world_sdf: Path,
    gui_world_sdf: Path,
    source_root: Path,
    occupancy: Path,
    esdf: Path,
    topology: Path,
) -> None:
    values = {
        "SIM_WORLD_NAME": world_name,
        "SIM_WORLD_SDF_PATH": repository_path(repository, collision_world_sdf),
        "SIM_COLLISION_WORLD_SDF_PATH": repository_path(
            repository, collision_world_sdf
        ),
        "SIM_GUI_WORLD_SDF_PATH": repository_path(repository, gui_world_sdf),
        "SIM_WORLD_RESOURCE_PATH": repository_path(repository, source_root / "fuel"),
        "STATIC_OCCUPANCY_3D_PATH": repository_path(repository, occupancy),
        "STATIC_ESDF_3D_CACHE_PATH": repository_path(repository, esdf),
        "STATIC_FREE_SPACE_TOPOLOGY_3D_PATH": repository_path(repository, topology),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(
            f"export {name}={shlex.quote(value)}\n" for name, value in values.items()
        ),
        encoding="utf-8",
    )


def _finite_vector(value: object, length: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise EnvironmentPreparationError(
            f"{label} must contain {length} numeric values"
        )
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise EnvironmentPreparationError(f"{label} must contain finite values")
    return result


def _map_to_sdf(
    point: tuple[float, float, float], transform: dict
) -> tuple[float, float, float]:
    source_axes = (transform.get("sdf_x_from"), transform.get("sdf_y_from"))
    if set(source_axes) != {"map_x", "map_y"}:
        raise EnvironmentPreparationError("invalid canonical map_to_sdf axes")
    map_axes = {"map_x": point[0], "map_y": point[1]}
    return (
        float(transform.get("sdf_x_scale", 1.0)) * map_axes[source_axes[0]]
        + float(transform["sdf_x_offset_m"]),
        float(transform.get("sdf_y_scale", 1.0)) * map_axes[source_axes[1]]
        + float(transform["sdf_y_offset_m"]),
        float(transform.get("sdf_z_scale", 1.0)) * point[2]
        + float(transform.get("sdf_z_offset_m", 0.0)),
    )


def load_launch_platforms(scenario_path: Path) -> list[dict]:
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    platforms = scenario.get("launch_platforms", [])
    if not isinstance(platforms, list):
        raise EnvironmentPreparationError("launch_platforms must be an array")
    vehicles = scenario.get("vehicles")
    if isinstance(vehicles, list):
        starts = {
            vehicle["id"]: _finite_vector(
                vehicle.get("map_start_m"),
                3,
                f"vehicle {vehicle.get('id')} map_start_m",
            )
            for vehicle in vehicles
        }
    else:
        vehicle = scenario.get("vehicle")
        if not isinstance(vehicle, dict):
            raise EnvironmentPreparationError(
                "scenario must define vehicles or one point-to-point vehicle"
            )
        starts = {
            "point_to_point_vehicle": _finite_vector(
                vehicle.get("map_start_m"), 3, "vehicle map_start_m"
            )
        }
    canonical_path = Path(scenario["canonical_world"])
    if not canonical_path.is_absolute():
        canonical_path = scenario_path.parent / canonical_path
    canonical = json.loads(canonical_path.resolve().read_text(encoding="utf-8"))
    transform = canonical.get("map_to_sdf")
    if not isinstance(transform, dict):
        raise EnvironmentPreparationError("canonical world is missing map_to_sdf")

    resolved: list[dict] = []
    seen_ids: set[str] = set()
    assigned: set[str] = set()
    for index, platform in enumerate(platforms):
        if not isinstance(platform, dict):
            raise EnvironmentPreparationError(
                f"launch_platforms[{index}] must be an object"
            )
        platform_id = platform.get("id")
        if (
            not isinstance(platform_id, str)
            or not platform_id
            or not platform_id.replace("_", "a").isalnum()
            or platform_id in seen_ids
        ):
            raise EnvironmentPreparationError(
                f"launch_platforms[{index}] has an invalid or duplicate id"
            )
        seen_ids.add(platform_id)
        vehicle_ids = platform.get("vehicle_ids")
        if not isinstance(vehicle_ids, list) or not vehicle_ids:
            raise EnvironmentPreparationError(
                f"launch platform {platform_id} must list vehicle_ids"
            )
        if any(vehicle_id not in starts for vehicle_id in vehicle_ids):
            raise EnvironmentPreparationError(
                f"launch platform {platform_id} references an unknown vehicle"
            )
        if assigned.intersection(vehicle_ids):
            raise EnvironmentPreparationError(
                f"a vehicle belongs to multiple launch platforms"
            )
        assigned.update(vehicle_ids)
        size = _finite_vector(
            platform.get("size_m"), 3, f"launch platform {platform_id} size_m"
        )
        if any(component <= 0.0 for component in size):
            raise EnvironmentPreparationError(
                f"launch platform {platform_id} size must be positive"
            )
        top_z = float(platform.get("top_z_m"))
        if not math.isfinite(top_z):
            raise EnvironmentPreparationError(
                f"launch platform {platform_id} top_z_m must be finite"
            )
        center_map = (
            sum(starts[vehicle_id][0] for vehicle_id in vehicle_ids)
            / len(vehicle_ids),
            sum(starts[vehicle_id][1] for vehicle_id in vehicle_ids)
            / len(vehicle_ids),
            top_z - size[2] / 2.0,
        )
        center_sdf = _map_to_sdf(center_map, transform)
        sdf_size = (
            size[0] if transform["sdf_x_from"] == "map_x" else size[1],
            size[1] if transform["sdf_y_from"] == "map_y" else size[0],
            size[2],
        )
        resolved.append(
            {
                "id": platform_id,
                "vehicle_ids": tuple(vehicle_ids),
                "center_sdf_m": center_sdf,
                "size_sdf_m": sdf_size,
            }
        )
    if platforms and assigned != set(starts):
        missing = sorted(set(starts) - assigned)
        raise EnvironmentPreparationError(
            f"vehicles missing launch platforms: {', '.join(missing)}"
        )
    return resolved


def add_launch_platforms(
    tree: ET.ElementTree, platforms: list[dict], preserve_visuals: bool
) -> None:
    world = tree.getroot().find("world")
    if world is None:
        raise EnvironmentPreparationError("materialized SDF has no world")
    for platform in platforms:
        model = ET.SubElement(
            world, "model", {"name": f"scenario_launch_platform_{platform['id']}"}
        )
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = " ".join(
            f"{value:.9g}" for value in (*platform["center_sdf_m"], 0.0, 0.0, 0.0)
        )
        link = ET.SubElement(model, "link", {"name": "platform"})
        collision = ET.SubElement(link, "collision", {"name": "collision"})
        geometry = ET.SubElement(collision, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = " ".join(
            f"{value:.9g}" for value in platform["size_sdf_m"]
        )
        if preserve_visuals:
            visual = ET.SubElement(link, "visual", {"name": "visual"})
            visual_geometry = ET.SubElement(visual, "geometry")
            visual_box = ET.SubElement(visual_geometry, "box")
            ET.SubElement(visual_box, "size").text = ET.tostring(
                box.find("size"), encoding="unicode", method="text"
            ).strip()
            material = ET.SubElement(visual, "material")
            ET.SubElement(material, "ambient").text = "0.18 0.22 0.24 1"
            ET.SubElement(material, "diffuse").text = "0.32 0.38 0.42 1"


def configure_gui_lighting(tree: ET.ElementTree) -> int:
    """Make presentation worlds readable without changing their physics."""
    world = tree.getroot().find("world")
    if world is None:
        raise EnvironmentPreparationError("materialized SDF has no world")
    scene = world.find("scene")
    if scene is None:
        scene = ET.Element("scene")
        world.insert(0, scene)
    ambient = scene.find("ambient")
    if ambient is None:
        ambient = ET.SubElement(scene, "ambient")
    ambient.text = "0.35 0.35 0.35 1"

    for light in world.findall("light"):
        world.remove(light)
    light_name = "drone_city_nav_gui_fill"
    light = ET.SubElement(
        world, "light", {"name": light_name, "type": "directional"}
    )
    ET.SubElement(light, "cast_shadows").text = "false"
    ET.SubElement(light, "pose").text = "0 0 100 0 0 0"
    ET.SubElement(light, "diffuse").text = "0.8 0.8 0.8 1"
    ET.SubElement(light, "specular").text = "0.2 0.2 0.2 1"
    ET.SubElement(light, "direction").text = "-0.35 0.25 -0.9"
    return 1


def remote_visual_resource_uris(source_root: Path) -> set[str]:
    result: set[str] = set()
    resource_tags = {
        "uri",
        "albedo_map",
        "normal_map",
        "roughness_map",
        "metalness_map",
        "emissive_map",
        "environment_map",
        "light_map",
        "init_from",
    }

    def collect(element: ET.Element, inside_script: bool = False) -> None:
        tag = element.tag.rsplit("}", 1)[-1]
        value = (element.text or "").strip()
        fuel = ResourceResolver.fuel_parts(value)
        if tag in resource_tags and not inside_script and fuel is not None and fuel[3]:
            result.add(value)
        for child in element:
            collect(child, inside_script or tag == "script")

    for path in sorted(source_root.rglob("*")):
        if not path.is_file() or path.suffix.casefold() not in {".dae", ".sdf"}:
            continue
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as exc:
            raise EnvironmentPreparationError(
                f"cannot inspect visual resources in {path}: {exc}"
            ) from exc
        collect(root)
    return result


def ensure_versioned_visual_resources(
    source_root: Path, cache_root: Path, dependencies: list[dict]
) -> tuple[Path, int, list[dict]]:
    pins = {
        (dependency["owner"].casefold(), dependency["model"].casefold()): dependency
        for dependency in dependencies
    }
    downloaded = 0
    resources: list[dict] = []
    for uri in sorted(remote_visual_resource_uris(source_root)):
        fuel = ResourceResolver.fuel_parts(uri)
        assert fuel is not None
        owner, model, unused_version, remainder = fuel
        del unused_version
        dependency = pins.get((owner.casefold(), model.casefold()))
        if dependency is None:
            # The source bundle owns this versioned Fuel asset. Only resources
            # external to the bundle need an explicit download pin.
            continue
        version = int(dependency["version"])
        destination = (
            cache_root
            / "fuel.gazebosim.org"
            / owner.casefold()
            / "models"
            / model.casefold()
            / str(version)
        ).joinpath(*remainder)
        encoded_path = "/".join(quote(part, safe="") for part in remainder)
        download_url = (
            "https://fuel.gazebosim.org/1.0/"
            f"{quote(dependency['owner'], safe='')}/models/"
            f"{quote(dependency['model'], safe='')}/{version}/files/{encoded_path}"
        )
        if not destination.is_file() or destination.stat().st_size == 0:
            destination.parent.mkdir(parents=True, exist_ok=True)
            temporary = destination.with_suffix(destination.suffix + ".part")
            request = Request(download_url, headers={"User-Agent": "drone-city-nav/1"})
            try:
                with urlopen(request, timeout=120) as response, temporary.open(
                    "wb"
                ) as output:
                    shutil.copyfileobj(response, output)
            except OSError as exc:
                temporary.unlink(missing_ok=True)
                raise EnvironmentPreparationError(
                    f"failed to fetch pinned visual resource {download_url}: {exc}"
                ) from exc
            if temporary.stat().st_size == 0:
                temporary.unlink(missing_ok=True)
                raise EnvironmentPreparationError(
                    f"downloaded visual resource is empty: {download_url}"
                )
            temporary.replace(destination)
            downloaded += 1
        resources.append(
            {
                "source_uri": uri,
                "versioned_uri": download_url,
                "local_path": str(destination.resolve()),
                "size_bytes": destination.stat().st_size,
                "sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
            }
        )
    return cache_root, downloaded, resources


def main() -> None:
    args = parse_args()
    manifest_path = args.manifest.resolve()
    manifest = load_manifest(manifest_path)
    repository = repository_root(manifest_path)
    launch_platforms: list[dict] = []
    if args.scenario is not None:
        scenario_path = args.scenario
        if not scenario_path.is_absolute():
            scenario_path = repository / scenario_path
        launch_platforms = load_launch_platforms(scenario_path.resolve())
    environment = find_environment(manifest, args.environment)
    if environment["distribution"] != "release":
        raise EnvironmentPreparationError(
            "simulation preparation currently expects a release environment"
        )
    install_root = args.asset_install_root
    if not install_root.is_absolute():
        install_root = repository / install_root
    install_root = install_root.resolve()

    source_artifact_id = environment["source"]["bundle_artifact_id"]
    source_root = ensure_release_artifact(
        repository, manifest_path, environment, source_artifact_id, install_root
    )
    static_map = select_static_map(environment, args.static_map)
    ensure_release_artifact(
        repository,
        manifest_path,
        environment,
        static_map["artifact_id"],
        install_root,
    )
    inputs = resolve_static_map_inputs(
        repository, environment, static_map, install_root
    )

    topology = default_output_path(repository, environment, static_map)
    if args.rebuild_topology or not topology.is_file():
        completed = subprocess.run(
            [
                sys.executable,
                str(repository / "scripts/compile_environment_topology.py"),
                "--manifest",
                str(manifest_path),
                "--environment",
                environment["id"],
                "--static-map",
                static_map["id"],
                "--asset-install-root",
                str(install_root),
                "--output",
                str(topology),
            ],
            cwd=repository,
            check=False,
        )
        if completed.returncode != 0:
            raise EnvironmentPreparationError("topology compilation failed")
    if topology.stat().st_size == 0:
        raise EnvironmentPreparationError("compiled topology is empty")

    source_world = source_root / environment["source"]["entrypoint"]
    if not source_world.is_file():
        raise EnvironmentPreparationError(f"source world is missing: {source_world}")
    runtime_root = (
        repository
        / "external/environment-artifacts/derived"
        / environment["id"]
        / "runtime"
    )
    visual_cache, downloaded_visuals, visual_resources = (
        ensure_versioned_visual_resources(
            source_root,
            install_root / environment["id"] / "visual_resources",
            environment["source"].get("visual_dependencies", []),
        )
    )
    resolver = ResourceResolver(
        [source_root / "fuel", visual_cache], source_model_paths(source_root)
    )
    for legacy_output in (runtime_root / "world.sdf", runtime_root / "materialization.json"):
        legacy_output.unlink(missing_ok=True)
    shutil.rmtree(runtime_root / "assets" / "meshes", ignore_errors=True)
    runtime_root.mkdir(parents=True, exist_ok=True)
    (runtime_root / "visual_resources.json").write_text(
        json.dumps(
            {
                "schema": "drone_city_nav_visual_resources_v1",
                "resources": visual_resources,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    collision_world_sdf = runtime_root / "world_collision.sdf"
    collision_report_path = runtime_root / "materialization_collision.json"
    collision_tree, collision_report = CollisionWorldMaterializer(resolver).materialize(
        source_world
    )
    add_launch_platforms(collision_tree, launch_platforms, preserve_visuals=False)
    collision_report.collision_instances += len(launch_platforms)
    collision_report.geometry_types["box"] = (
        collision_report.geometry_types.get("box", 0) + len(launch_platforms)
    )
    collision_fingerprint = write_materialized_world(
        collision_tree, collision_world_sdf
    )
    write_report(
        collision_report,
        collision_world_sdf,
        collision_fingerprint,
        collision_report_path,
    )

    gui_world_sdf = runtime_root / "world_gui.sdf"
    gui_report_path = runtime_root / "materialization_gui.json"
    gui_tree, gui_report = CollisionWorldMaterializer(
        resolver,
        preserve_visuals=True,
        localized_mesh_root=runtime_root / "assets" / "meshes",
    ).materialize(source_world)
    add_launch_platforms(gui_tree, launch_platforms, preserve_visuals=True)
    gui_report.light_instances = configure_gui_lighting(gui_tree)
    gui_report.collision_instances += len(launch_platforms)
    gui_report.visual_instances += len(launch_platforms)
    gui_report.geometry_types["box"] = (
        gui_report.geometry_types.get("box", 0) + 2 * len(launch_platforms)
    )
    gui_fingerprint = write_materialized_world(gui_tree, gui_world_sdf)
    visual_uri_count = validate_visual_resource_uris(gui_world_sdf)
    write_report(gui_report, gui_world_sdf, gui_fingerprint, gui_report_path)

    world = ET.parse(collision_world_sdf).getroot().find("world")
    if world is None or not world.attrib.get("name"):
        raise EnvironmentPreparationError("materialized SDF has no named world")
    world_name = world.attrib["name"]
    environment_file = runtime_root / "environment.env"
    write_runtime_environment(
        environment_file,
        repository,
        world_name,
        collision_world_sdf,
        gui_world_sdf,
        source_root,
        inputs.occupancy,
        inputs.esdf,
        topology,
    )
    print(
        "ENVIRONMENT_SIMULATION_READY"
        f" environment={environment['id']} static_map={static_map['id']}"
        f" world={world_name} collisions={collision_report.collision_instances}"
        f" visuals={gui_report.visual_instances} lights={gui_report.light_instances}"
        f" visual_uris={visual_uri_count}"
        f" visual_resources_verified={len(visual_resources)}"
        f" visual_resources_downloaded={downloaded_visuals}"
        f" launch_platforms={len(launch_platforms)}"
        f" occupancy={inputs.occupancy} esdf={inputs.esdf} topology={topology}"
        f" env_file={environment_file}"
    )


if __name__ == "__main__":
    try:
        main()
    except (
        EnvironmentPreparationError,
        ManifestError,
        MaterializationError,
        OSError,
    ) as error:
        raise SystemExit(f"environment preparation error: {error}") from error
