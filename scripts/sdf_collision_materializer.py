"""Resolve SDF includes into a deterministic collision-only world."""

from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable
from urllib.parse import unquote, urlparse
from xml.etree import ElementTree as ET


class MaterializationError(RuntimeError):
    """Raised when physical geometry cannot be resolved without guessing."""


@dataclass(frozen=True)
class Transform:
    values: tuple[tuple[float, float, float, float], ...]

    @classmethod
    def identity(cls) -> "Transform":
        return cls(
            (
                (1.0, 0.0, 0.0, 0.0),
                (0.0, 1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
            )
        )

    @classmethod
    def from_pose(cls, pose: ET.Element | None) -> "Transform":
        if pose is None:
            return cls.identity()
        unsupported_attributes = set(pose.attrib) - {"frame", "relative_to"}
        if unsupported_attributes:
            raise MaterializationError(
                f"unsupported pose attributes: {sorted(unsupported_attributes)}"
            )
        relative_to = pose.attrib.get("relative_to", "")
        legacy_frame = pose.attrib.get("frame", "")
        if legacy_frame or relative_to not in {"", "__model__"}:
            raise MaterializationError(
                "pose frame relationship requires frame-graph resolution: "
                f"frame={legacy_frame!r} relative_to={relative_to!r}"
            )
        values = tuple(float(value) for value in (pose.text or "").split())
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise MaterializationError("pose must contain six finite values")
        x, y, z, roll, pitch, yaw = values
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return cls(
            (
                (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x),
                (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y),
                (-sp, cp * sr, cp * cr, z),
                (0.0, 0.0, 0.0, 1.0),
            )
        )

    def compose(self, child: "Transform") -> "Transform":
        return Transform(
            tuple(
                tuple(
                    sum(self.values[row][inner] * child.values[inner][column]
                        for inner in range(4))
                    for column in range(4)
                )
                for row in range(4)
            )
        )

    def as_pose_text(self) -> str:
        rotation = self.values
        pitch = math.asin(max(-1.0, min(1.0, -rotation[2][0])))
        if abs(math.cos(pitch)) > 1.0e-9:
            roll = math.atan2(rotation[2][1], rotation[2][2])
            yaw = math.atan2(rotation[1][0], rotation[0][0])
        else:
            roll = math.atan2(-rotation[1][2], rotation[1][1])
            yaw = 0.0
        values = (
            rotation[0][3], rotation[1][3], rotation[2][3], roll, pitch, yaw
        )
        return " ".join(f"{value:.12g}" for value in values)


@dataclass
class MaterializationReport:
    source_world: str
    mode: str = "collision"
    collision_instances: int = 0
    visual_instances: int = 0
    light_instances: int = 0
    dynamic_models_skipped: int = 0
    non_collision_resources_skipped: int = 0
    geometry_types: dict[str, int] = field(default_factory=dict)
    model_files: set[str] = field(default_factory=set)
    mesh_files: set[str] = field(default_factory=set)
    visual_resource_files: set[str] = field(default_factory=set)
    issues: list[str] = field(default_factory=list)

    def as_dict(self, output_sdf: Path, fingerprint: str) -> dict:
        return {
            "schema": "drone_city_nav_sdf_materialization_v2",
            "source_world": self.source_world,
            "mode": self.mode,
            "output_sdf": str(output_sdf.resolve()),
            "output_sha256": fingerprint,
            "collision_instances": self.collision_instances,
            "visual_instances": self.visual_instances,
            "light_instances": self.light_instances,
            "dynamic_models_skipped": self.dynamic_models_skipped,
            "non_collision_resources_skipped": self.non_collision_resources_skipped,
            "geometry_types": dict(sorted(self.geometry_types.items())),
            "model_files": sorted(self.model_files),
            "mesh_files": sorted(self.mesh_files),
            "visual_resource_files": sorted(self.visual_resource_files),
            "issues": self.issues,
        }


class ResourceResolver:
    _FUEL_HOSTS = ("fuel.gazebosim.org", "fuel.ignitionrobotics.org")

    def __init__(
        self,
        fuel_caches: Iterable[Path],
        model_paths: Iterable[Path],
        resource_paths: Iterable[Path] = (),
    ):
        self._fuel_caches = tuple(path.resolve() for path in fuel_caches)
        self._model_paths = tuple(path.resolve() for path in model_paths)
        self._resource_paths = tuple(path.resolve() for path in resource_paths)

    @staticmethod
    def fuel_parts(uri: str) -> tuple[str, str, int | None, tuple[str, ...]] | None:
        parsed = urlparse(uri.strip())
        if parsed.scheme not in {"http", "https"} or parsed.netloc not in {
            "fuel.gazebosim.org",
            "fuel.ignitionrobotics.org",
        }:
            return None
        parts = tuple(unquote(part) for part in parsed.path.split("/") if part)
        try:
            model_index = tuple(part.lower() for part in parts).index("models")
        except ValueError:
            return None
        if model_index == 0 or model_index + 1 >= len(parts):
            return None
        owner = parts[model_index - 1].lower()
        name = parts[model_index + 1].lower()
        version = None
        remainder = parts[model_index + 2 :]
        if remainder and remainder[0].isdigit():
            version = int(remainder[0])
            remainder = remainder[1:]
        elif remainder and remainder[0].casefold() == "tip":
            remainder = remainder[1:]
        if remainder and remainder[0].lower() == "files":
            remainder = remainder[1:]
        return owner, name, version, remainder

    def _fuel_version_directory(
        self, owner: str, name: str, requested_version: int | None
    ) -> Path:
        matches: list[Path] = []
        for cache in self._fuel_caches:
            for host in self._FUEL_HOSTS:
                models_root = cache / host / owner / "models"
                if not models_root.is_dir():
                    continue
                model_roots = sorted(
                    child
                    for child in models_root.iterdir()
                    if child.is_dir() and unquote(child.name).casefold() == name.casefold()
                )
                for model_root in model_roots:
                    if requested_version is not None:
                        candidate = model_root / str(requested_version)
                        if candidate.is_dir():
                            return candidate
                        continue
                    matches.extend(
                        child
                        for child in model_root.iterdir()
                        if child.is_dir() and child.name.isdigit()
                    )
        if not matches:
            version_text = "latest" if requested_version is None else str(requested_version)
            raise MaterializationError(
                f"Fuel model is not cached: {owner}/{name} version {version_text}"
            )
        return max(matches, key=lambda path: int(path.name))

    def resolve_model(self, uri: str, referring_file: Path) -> Path:
        fuel = self.fuel_parts(uri)
        if fuel is not None:
            owner, name, version, remainder = fuel
            if remainder:
                raise MaterializationError(f"model URI points to a file: {uri}")
            model_file = self._fuel_version_directory(owner, name, version) / "model.sdf"
            if model_file.is_file():
                return model_file.resolve()
            raise MaterializationError(f"Fuel model has no model.sdf: {uri}")

        stripped = uri.strip()
        if stripped.startswith("model://"):
            relative = Path(stripped.removeprefix("model://"))
            for root in self._model_paths:
                model_file = root / relative / "model.sdf"
                if model_file.is_file():
                    return model_file.resolve()
            raise MaterializationError(f"model URI is not present in model paths: {uri}")
        for candidate in self._local_candidates(stripped, referring_file):
            model_file = candidate / "model.sdf" if candidate.is_dir() else candidate
            if model_file.is_file():
                return model_file.resolve()
        raise MaterializationError(f"cannot resolve model URI: {uri}")

    def resolve_mesh(self, uri: str, referring_file: Path) -> Path:
        fuel = self.fuel_parts(uri)
        if fuel is not None:
            owner, name, version, remainder = fuel
            if not remainder:
                raise MaterializationError(f"mesh URI points to a model: {uri}")
            candidate = self._fuel_version_directory(owner, name, version).joinpath(
                *remainder
            )
            if candidate.is_file():
                return candidate.resolve()
            raise MaterializationError(f"Fuel mesh is not cached: {uri}")

        stripped = uri.strip()
        if stripped.startswith("model://"):
            relative = Path(stripped.removeprefix("model://"))
            for root in self._model_paths:
                candidate = root / relative
                if candidate.is_file():
                    return candidate.resolve()
            raise MaterializationError(f"mesh URI is not present in model paths: {uri}")
        for candidate in self._local_candidates(stripped, referring_file):
            if candidate.is_file():
                return candidate.resolve()
        raise MaterializationError(f"cannot resolve mesh URI: {uri}")

    def resolve_file(self, uri: str, referring_file: Path) -> Path:
        return self.resolve_mesh(uri, referring_file)

    def _local_candidates(self, uri: str, referring_file: Path) -> list[Path]:
        relative = Path(uri.removeprefix("file://"))
        if relative.is_absolute():
            return [relative]
        candidates = [(referring_file.parent / relative).resolve()]
        for root in self._model_paths:
            candidates.append((root / relative).resolve())
            if relative.parts and relative.parts[0] == root.name:
                candidates.append((root.parent / relative).resolve())
        for root in self._resource_paths:
            candidates.append((root / relative).resolve())
            candidates.append((root / relative.name).resolve())
        return candidates


class CollisionWorldMaterializer:
    def __init__(
        self,
        resolver: ResourceResolver,
        preview_visuals: bool = False,
        preserve_visuals: bool = False,
        localized_mesh_root: Path | None = None,
    ):
        self._resolver = resolver
        self._preview_visuals = preview_visuals
        self._preserve_visuals = preserve_visuals
        self._localized_mesh_root = (
            None if localized_mesh_root is None else localized_mesh_root.resolve()
        )
        self._output_world: ET.Element | None = None
        self._report: MaterializationReport | None = None
        self._active_model_files: set[Path] = set()
        self._localized_meshes: dict[Path, Path] = {}
        self._instance_number = 0

    def materialize(self, source_world: Path) -> tuple[ET.ElementTree, MaterializationReport]:
        source_world = source_world.resolve()
        source_root = ET.parse(source_world).getroot()
        source = source_root.find("world")
        if source is None:
            raise MaterializationError(f"SDF contains no world: {source_world}")

        output_root = ET.Element("sdf", {"version": "1.10"})
        self._output_world = ET.SubElement(
            output_root, "world", {"name": f"{source.attrib.get('name', 'world')}_collisions"}
        )
        self._report = MaterializationReport(
            source_world=str(source_world),
            mode="gui" if self._preserve_visuals else "collision",
        )
        self._copy_world_environment(source)
        for model in source.findall("model"):
            self._visit_model(
                model,
                Transform.identity(),
                source_world,
                None,
                model.attrib.get("name", "model"),
                None,
            )
        for include in source.findall("include"):
            self._visit_include(include, Transform.identity(), source_world, "include")
        return ET.ElementTree(output_root), self._report

    def _copy_world_environment(self, source: ET.Element) -> None:
        assert self._output_world is not None
        copied_tags = {
            "physics",
            "gravity",
            "magnetic_field",
            "atmosphere",
            "spherical_coordinates",
            "scene",
            "light",
        }
        present_tags: set[str] = set()
        for child in source:
            if child.tag not in copied_tags:
                continue
            self._output_world.append(copy.deepcopy(child))
            present_tags.add(child.tag)

        defaults = {
            "gravity": "0 0 -9.8",
            "magnetic_field": "6e-06 2.3e-05 -4.2e-05",
        }
        for tag, value in defaults.items():
            if tag not in present_tags:
                ET.SubElement(self._output_world, tag).text = value
        if "atmosphere" not in present_tags:
            ET.SubElement(self._output_world, "atmosphere", {"type": "adiabatic"})
        if "spherical_coordinates" not in present_tags:
            coordinates = ET.SubElement(self._output_world, "spherical_coordinates")
            ET.SubElement(coordinates, "surface_model").text = "EARTH_WGS84"
            ET.SubElement(coordinates, "world_frame_orientation").text = "ENU"
            ET.SubElement(coordinates, "latitude_deg").text = "47.397971057728974"
            ET.SubElement(coordinates, "longitude_deg").text = "8.546163739800146"
            ET.SubElement(coordinates, "elevation").text = "0"

    def _visit_include(
        self, include: ET.Element, parent: Transform, source_file: Path, prefix: str
    ) -> None:
        uri = (include.findtext("uri") or "").strip()
        if not uri:
            raise MaterializationError(f"include without URI in {source_file}")
        if include.find("placement_frame") is not None or include.find("merge") is not None:
            raise MaterializationError(f"unsupported include semantics for {uri}")
        model_file = self._resolver.resolve_model(uri, source_file)
        pose_element = include.find("pose")
        pose_override = (
            None if pose_element is None else Transform.from_pose(pose_element)
        )
        override_text = include.findtext("static")
        static_override = None if override_text is None else _parse_bool(override_text)
        include_name = include.findtext("name") or Path(uri.rstrip("/")).name or prefix
        self._visit_model_file(
            model_file, parent, static_override, f"{prefix}_{include_name}",
            pose_override,
        )

    def _visit_model_file(
        self, model_file: Path, parent: Transform, static_override: bool | None,
        prefix: str, pose_override: Transform | None,
    ) -> None:
        if model_file in self._active_model_files:
            raise MaterializationError(f"cyclic model include: {model_file}")
        self._active_model_files.add(model_file)
        try:
            root = ET.parse(model_file).getroot()
            model = root.find("model")
            if model is None:
                light = root.find("light")
                if light is not None:
                    if self._preserve_visuals:
                        self._emit_light(
                            light, parent, prefix, pose_override=pose_override
                        )
                    else:
                        assert self._report is not None
                        self._report.non_collision_resources_skipped += 1
                    return
                raise MaterializationError(f"SDF contains no model: {model_file}")
            assert self._report is not None
            self._report.model_files.add(str(model_file))
            self._visit_model(
                model, parent, model_file, static_override, prefix, pose_override
            )
        finally:
            self._active_model_files.remove(model_file)

    def _visit_model(
        self, model: ET.Element, parent: Transform, source_file: Path,
        static_override: bool | None, prefix: str,
        pose_override: Transform | None,
    ) -> None:
        model_pose = (
            Transform.from_pose(model.find("pose"))
            if pose_override is None
            else pose_override
        )
        model_transform = parent.compose(model_pose)
        model_static = _parse_bool(model.findtext("static") or "false")
        is_static = model_static if static_override is None else static_override
        model_name = model.attrib.get("name", "model")
        current_prefix = f"{prefix}_{model_name}"
        direct_collisions = model.findall("./link/collision")
        if is_static:
            if self._preserve_visuals:
                for light in model.findall("light"):
                    self._emit_light(light, model_transform, current_prefix)
            for link in model.findall("link"):
                link_transform = model_transform.compose(
                    Transform.from_pose(link.find("pose"))
                )
                if self._preserve_visuals:
                    for light in link.findall("light"):
                        self._emit_light(light, link_transform, current_prefix)
                for collision in link.findall("collision"):
                    collision_transform = link_transform.compose(
                        Transform.from_pose(collision.find("pose"))
                    )
                    self._emit_collision(
                        collision, collision_transform, source_file,
                        f"{current_prefix}_{link.attrib.get('name', 'link')}"
                    )
                if self._preserve_visuals:
                    for visual in link.findall("visual"):
                        visual_transform = link_transform.compose(
                            Transform.from_pose(visual.find("pose"))
                        )
                        self._emit_visual(
                            visual,
                            visual_transform,
                            source_file,
                            f"{current_prefix}_{link.attrib.get('name', 'link')}",
                        )
        elif direct_collisions:
            assert self._report is not None
            self._report.dynamic_models_skipped += 1

        for nested in model.findall("model"):
            self._visit_model(
                nested,
                model_transform,
                source_file,
                None,
                current_prefix,
                None,
            )
        for include in model.findall("include"):
            self._visit_include(include, model_transform, source_file, current_prefix)

    def _emit_collision(
        self, collision: ET.Element, transform: Transform, source_file: Path,
        prefix: str
    ) -> None:
        geometry = collision.find("geometry")
        if geometry is None or len(geometry) != 1:
            raise MaterializationError(f"collision must contain one geometry: {source_file}")
        geometry_copy = copy.deepcopy(geometry)
        geometry_type = geometry_copy[0].tag
        mesh_uri = geometry_copy.find("./mesh/uri")
        if mesh_uri is not None:
            mesh_path = self._resolver.resolve_mesh(mesh_uri.text or "", source_file)
            mesh_uri.text = str(
                self._localize_visual_mesh(mesh_path)
                if self._preserve_visuals
                else mesh_path
            )
            assert self._report is not None
            self._report.mesh_files.add(str(mesh_path))

        assert self._output_world is not None and self._report is not None
        self._instance_number += 1
        instance_name = _safe_name(
            f"{prefix}_{collision.attrib.get('name', 'collision')}_{self._instance_number}"
        )
        output_model = ET.SubElement(self._output_world, "model", {"name": instance_name})
        ET.SubElement(output_model, "static").text = "true"
        ET.SubElement(output_model, "pose").text = transform.as_pose_text()
        output_link = ET.SubElement(output_model, "link", {"name": "link"})
        output_collision = ET.SubElement(
            output_link, "collision", {"name": "collision"}
        )
        output_collision.append(geometry_copy)
        if self._preview_visuals:
            visual = ET.SubElement(output_link, "visual", {"name": "collision_visual"})
            visual.append(copy.deepcopy(geometry_copy))
            material = ET.SubElement(visual, "material")
            ET.SubElement(material, "diffuse").text = "0.55 0.58 0.62 1"
        self._report.collision_instances += 1
        self._report.geometry_types[geometry_type] = (
            self._report.geometry_types.get(geometry_type, 0) + 1
        )

    def _emit_visual(
        self,
        visual: ET.Element,
        transform: Transform,
        source_file: Path,
        prefix: str,
    ) -> None:
        geometry = visual.find("geometry")
        if geometry is None or len(geometry) != 1:
            raise MaterializationError(f"visual must contain one geometry: {source_file}")
        visual_copy = copy.deepcopy(visual)
        pose = visual_copy.find("pose")
        if pose is not None:
            visual_copy.remove(pose)
        for plugin in visual_copy.findall("plugin"):
            visual_copy.remove(plugin)
            assert self._report is not None
            self._report.non_collision_resources_skipped += 1
        mesh_uri = visual_copy.find("./geometry/mesh/uri")
        if mesh_uri is not None:
            mesh_path = self._resolver.resolve_mesh(mesh_uri.text or "", source_file)
            mesh_uri.text = str(self._localize_visual_mesh(mesh_path))
            assert self._report is not None
            self._report.mesh_files.add(str(mesh_path))
        self._localize_visual_material(visual_copy, source_file)

        assert self._output_world is not None and self._report is not None
        self._instance_number += 1
        instance_name = _safe_name(
            f"{prefix}_{visual.attrib.get('name', 'visual')}_{self._instance_number}"
        )
        output_model = ET.SubElement(self._output_world, "model", {"name": instance_name})
        ET.SubElement(output_model, "static").text = "true"
        ET.SubElement(output_model, "pose").text = transform.as_pose_text()
        output_link = ET.SubElement(output_model, "link", {"name": "link"})
        output_link.append(visual_copy)
        self._report.visual_instances += 1

    def _emit_light(
        self,
        light: ET.Element,
        parent: Transform,
        prefix: str,
        pose_override: Transform | None = None,
    ) -> None:
        light_copy = copy.deepcopy(light)
        local_pose = (
            Transform.from_pose(light_copy.find("pose"))
            if pose_override is None
            else pose_override
        )
        world_pose = parent.compose(local_pose)
        pose = light_copy.find("pose")
        if pose is None:
            pose = ET.Element("pose")
            light_copy.insert(0, pose)
        pose.attrib.clear()
        pose.text = world_pose.as_pose_text()

        assert self._output_world is not None and self._report is not None
        self._instance_number += 1
        light_copy.attrib["name"] = _safe_name(
            f"{prefix}_{light.attrib.get('name', 'light')}_{self._instance_number}"
        )
        self._output_world.append(light_copy)
        self._report.light_instances += 1

    def _localize_visual_material(
        self, visual: ET.Element, source_file: Path
    ) -> None:
        material = visual.find("material")
        if material is None:
            return
        script = material.find("script")
        if script is not None:
            material.remove(script)
        for element in material.iter():
            if _local_tag(element.tag) not in _MATERIAL_RESOURCE_TAGS:
                continue
            value = (element.text or "").strip()
            if not value:
                continue
            resource = self._resolver.resolve_file(value, source_file)
            element.text = str(resource)
            assert self._report is not None
            self._report.visual_resource_files.add(str(resource))

    def _localize_visual_mesh(self, source: Path) -> Path:
        if source.suffix.casefold() != ".dae":
            return source
        if source in self._localized_meshes:
            return self._localized_meshes[source]
        if self._localized_mesh_root is None:
            raise MaterializationError(
                "preserved DAE visuals require a localized mesh directory"
            )
        digest = hashlib.sha256(source.read_bytes()).hexdigest()[:16]
        destination = self._localized_mesh_root / f"{digest}_{source.name}"
        tree = ET.parse(source)
        root_namespace = _namespace_uri(tree.getroot().tag)
        if root_namespace:
            ET.register_namespace("", root_namespace)
        for element in _collada_image_init_from_elements(tree.getroot()):
            value = (element.text or "").strip()
            if not value or value.startswith("#"):
                continue
            resource = self._resolver.resolve_file(value, source)
            element.text = Path(
                os.path.relpath(resource, start=destination.parent)
            ).as_posix()
            assert self._report is not None
            self._report.visual_resource_files.add(str(resource))
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_suffix(destination.suffix + ".part")
        tree.write(temporary, encoding="utf-8", xml_declaration=True)
        temporary.replace(destination)
        self._localized_meshes[source] = destination
        return destination


def write_materialized_world(tree: ET.ElementTree, output: Path) -> str:
    output = output.resolve()
    for element in tree.getroot().iter():
        if _local_tag(element.tag) not in _SDF_RESOURCE_TAGS:
            continue
        resource_path = Path(element.text or "")
        if resource_path.is_absolute():
            element.text = Path(
                os.path.relpath(resource_path, start=output.parent)
            ).as_posix()
    ET.indent(tree.getroot(), space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return hashlib.sha256(output.read_bytes()).hexdigest()


def validate_visual_resource_uris(output_sdf: Path) -> int:
    output_sdf = output_sdf.resolve()
    root = ET.parse(output_sdf).getroot()
    resolved_count = 0
    dae_files: set[Path] = set()
    for element in root.iter():
        if _local_tag(element.tag) not in _SDF_RESOURCE_TAGS:
            continue
        value = (element.text or "").strip()
        if not value:
            raise MaterializationError("empty visual resource URI")
        resource = _require_local_resource(value, output_sdf.parent)
        resolved_count += 1
        if resource.suffix.casefold() == ".dae":
            dae_files.add(resource)
    for dae_file in dae_files:
        for element in _collada_image_init_from_elements(ET.parse(dae_file).getroot()):
            value = (element.text or "").strip()
            if value and not value.startswith("#"):
                _require_local_resource(value, dae_file.parent)
                resolved_count += 1
    return resolved_count


def write_report(report: MaterializationReport, output_sdf: Path, fingerprint: str,
                 output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report.as_dict(output_sdf, fingerprint), indent=2) + "\n",
        encoding="utf-8",
    )


def _parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true"}:
        return True
    if normalized in {"0", "false"}:
        return False
    raise MaterializationError(f"invalid SDF boolean: {value!r}")


def _safe_name(value: str) -> str:
    sanitized = re.sub(r"[^A-Za-z0-9_]+", "_", value).strip("_")
    return sanitized[:240] or "collision"


_MATERIAL_RESOURCE_TAGS = {
    "albedo_map",
    "normal_map",
    "roughness_map",
    "metalness_map",
    "emissive_map",
    "environment_map",
    "light_map",
}
_SDF_RESOURCE_TAGS = _MATERIAL_RESOURCE_TAGS | {"uri"}


def _local_tag(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _namespace_uri(tag: str) -> str:
    if not tag.startswith("{") or "}" not in tag:
        return ""
    return tag[1 : tag.index("}")]


def _collada_image_init_from_elements(root: ET.Element) -> Iterable[ET.Element]:
    """Yield image resource URIs, excluding symbolic COLLADA sampler references."""
    for image in root.iter():
        if _local_tag(image.tag) != "image":
            continue
        for child in image:
            if _local_tag(child.tag) == "init_from":
                yield child


def _require_local_resource(value: str, base: Path) -> Path:
    parsed = urlparse(value)
    if parsed.scheme or value.startswith("model://"):
        raise MaterializationError(f"non-local visual resource URI: {value}")
    resource = (base / value).resolve()
    if not resource.exists():
        raise MaterializationError(f"unresolved visual resource URI: {value}")
    return resource
