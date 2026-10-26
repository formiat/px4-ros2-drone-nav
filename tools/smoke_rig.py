#!/usr/bin/env python3
"""smoke_rig.py OUT_DIR LEVEL... (inside the container, after `make build`): what the vehicle's sensors see
through smoke (roadmap item 18 stage 1). A carrier floating 1.5 m over a textured floor carries the camera set of
models/stereo_tof_v1 (the pair, its carried light, the time-of-flight ring, the evaluation depth), the lidar of
models/lidar_3d_v1 and, for this measurement only, a thermal camera at the pair; a textured wall stands 8 m ahead
(310 K). Between them a box of smoke, read by every sensor through simulation_smoke_sensor_node as in a flight,
or Gazebo's own particles for the comparison. For each level the world runs headless, the carried light is set
as a flight sets it, and after a warm-up each sensor's next frames are saved decoded to OUT_DIR/NAME.npz
(tools/smoke_rig_report.py reads them on the host).

A LEVEL is NAME[,key=value...]:
  sigma=S      the smoke model's extinction coefficient, 1/m (0: no smoke)
  x0=X, len=L  the smoke's near face X m ahead of the carrier (1.0) and its length L m (3.0); 6 m wide, 3 m tall
  wall=W       the wall's face W m ahead of the carrier (8.0)
  inside=1     the smoke round the carrier as well (x from -3 m)
  light=pair|offset|off   the carried light where the model has it, a light of the same power 0.45 m aside and
               0.55 m below the pair (the remedy item 18 compares), or none
  noise=N      the pair's Gaussian noise setting instead of the model's 0.01 (0.045 gives about 2.5 levels)
  particles=R  Gazebo's ParticleEmitter in the same box at R particles a second (tools/media/fog), no model;
               ratio=P its particle_scatter_ratio (0.65)
  fog=D        Gazebo's scene fog of density D (exponential), no model
  panel=D,albedo=A  calibration: a matte panel of albedo A, D m in front of the pair, no wall, no smoke
  heat=T       a heat source (roadmap item 32): a sphere of radius heatr=R m (0.3) at the surface temperature T K, textured as
               the wall is, created at run time through the world's create service as a flight creates it; heatx=D
               its centre D m ahead of the carrier (4.0), heatz=Z its height (the carrier's); read by the thermal
               core of the vehicle's class (160 x 120, 57 degrees, 9 Hz, up to 650 K) the carrier adds for it
  corridor=W   two textured side walls W m apart along the whole floor, 6 m tall (the location's north corridor is
               4.4 m wide): the smoke box, 6 m wide, then fills the corridor wall to wall, and the walls give the
               pair's silence a rim at the smoke's face as the location does (the smoke face detector's measurement)
  ceiling=H    a textured ceiling H m over the floor (with corridor)
  height=Z     the carrier Z m over the floor instead of 1.5 (from 8 m up the floor is out of the lamp's reach, as a
               vehicle 9 m over a hall's floor has it); smokez=Z the smoke box's centre height (the carrier's by
               default), for a plume floating above the horizon; the mover's flight keeps 1.5
  record=S     also S seconds of the pair, as the camera driver passes them on (its automatic gain, grey), for the
               visual-inertial replay (tools/vio/replay_vio.cpp) in OUT_DIR/NAME_vio/: the carrier stands still, so
               the IMU reads gravity alone and whatever the estimate moves is the frames' doing
  move=1       with record: the carrier flies instead, from 6 m behind its place to 5 m ahead of it at up to
               1 m/s (2 s of rest, 2 s to speed, cruise, 2 s to rest; through the smoke when it stands ahead), set
               a hundred times a second; the IMU reads that motion and gravity
"""
from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.image_pb2 import Image
from gz.msgs10.light_pb2 import Light
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.transport13 import Node

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / "scripts"))
from carried_light import light_message  # noqa: E402

FOG = REPOSITORY / "tools/media/fog"
NODE = REPOSITORY / "install/drone_city_nav/lib/drone_city_nav/simulation_smoke_sensor_node"
WARM_UP_S = 10.0
FRAMES = 8
SENSORS = {  # key: (topic tail, message)
    "left": ("stereo_tof_link/sensor/stereo_left/image", Image),
    "right": ("stereo_tof_link/sensor/stereo_right/image", Image),
    "depth": ("stereo_tof_link/sensor/evaluation_depth_left/depth_image", Image),
    "thermal": ("thermal_link/sensor/thermal/image", Image),
    "thermal_right": ("thermal_right_link/sensor/thermal_right/image", Image),
    "thermal_core": ("thermal_core_link/sensor/thermal_core/image", Image),
    "tof_front": ("stereo_tof_link/sensor/tof_front/scan/points", PointCloudPacked),
    "lidar": ("link/sensor/lidar_3d_v1/scan/points", PointCloudPacked),
}
RELAYED = {"left", "right", "tof_front", "lidar"}


def carrier(models: Path, light: str, noise: str | None) -> None:
    """The carrier model and, for the offset light or another camera noise, its own copy of the camera set."""
    camera_set = "stereo_tof_v1"
    if light == "offset" or noise is not None:
        camera_set = "stereo_tof_rig"
        shutil.rmtree(models / camera_set, ignore_errors=True)
        shutil.copytree(REPOSITORY / "drone_city_nav/models/stereo_tof_v1", models / camera_set)
        sdf = models / camera_set / "model.sdf"
        text = sdf.read_text()
        if light == "offset":
            text, count = re.subn(r'(<light name="carried_light" type="spot">\s*<pose>)[^<]*(</pose>)',
                                  r"\g<1>0.02 0.45 -0.55 0 0 0\g<2>", text)
            assert count == 1, "the carried light was not found"
        if noise is not None:
            text, count = re.subn(r"(<sensor name=\"stereo_(?:left|right)\".*?<stddev>)[^<]*(</stddev>)",
                                  rf"\g<1>{float(noise)}\g<2>", text, flags=re.S)
            assert count == 2, "the pair's noise was not found"
        sdf.write_text(text.replace("<name>stereo_tof_v1</name>", f"<name>{camera_set}</name>"))
    directory = models / "carrier"
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "model.config").write_text(
        "<?xml version='1.0'?><model><name>carrier</name><sdf version='1.10'>model.sdf</sdf></model>\n")
    (directory / "model.sdf").write_text(f"""<?xml version="1.0"?>
<sdf version="1.10"><model name="carrier"><static>false</static>
  <include merge="true"><uri>model://{camera_set}</uri><pose>0 0 0 0 0 0</pose></include>
  <include merge="true"><uri>model://lidar_3d_v1</uri><pose>0 0 0.25 0 0 0</pose></include>
  <link name="thermal_link"><pose>0.32 0 0 0 0 0</pose>
    <sensor name="thermal" type="thermal_camera"><always_on>1</always_on><update_rate>5</update_rate>
      <camera><horizontal_fov>1.658</horizontal_fov><image><width>640</width><height>512</height><format>L16</format>
        </image><clip><near>0.1</near><far>100</far></clip></camera>
      <plugin filename="gz-sim-thermal-sensor-system" name="gz::sim::systems::ThermalSensor">
        <min_temp>250</min_temp><max_temp>350</max_temp><resolution>0.01</resolution></plugin>
    </sensor></link>
  <link name="thermal_right_link"><pose>0.32 -0.20 0 0 0 0</pose>
    <sensor name="thermal_right" type="thermal_camera"><always_on>1</always_on><update_rate>5</update_rate>
      <camera><horizontal_fov>1.658</horizontal_fov><image><width>640</width><height>512</height><format>L16</format>
        </image><clip><near>0.1</near><far>100</far></clip></camera>
      <plugin filename="gz-sim-thermal-sensor-system" name="gz::sim::systems::ThermalSensor">
        <min_temp>250</min_temp><max_temp>350</max_temp><resolution>0.01</resolution></plugin>
    </sensor></link>
  <link name="thermal_core_link"><pose>0.32 0 0.07 0 0 0</pose>
    <sensor name="thermal_core" type="thermal_camera"><always_on>1</always_on><update_rate>9</update_rate>
      <camera><horizontal_fov>0.9948376736367679</horizontal_fov><image><width>160</width><height>120</height>
        <format>L16</format></image><clip><near>0.1</near><far>60</far></clip></camera>
      <plugin filename="gz-sim-thermal-sensor-system" name="gz::sim::systems::ThermalSensor">
        <min_temp>250</min_temp><max_temp>650</max_temp><resolution>0.01</resolution></plugin>
    </sensor></link>
</model></sdf>
""")


def world_sdf(spec: dict[str, str], textures: Path) -> str:
    x0 = float(spec.get("x0", 1.0))
    length = float(spec.get("len", 3.0))
    height = float(spec.get("height", 1.5))
    smoke_z = float(spec.get("smokez", height))
    if spec.get("inside") == "1":
        length += x0 + 3.0
        x0 = -3.0
    extra = ""
    if "particles" in spec:
        extra += f"""
  <model name="smoke"><static>true</static><pose>{x0 + length / 2} 0 {smoke_z} 0 0 0</pose><link name="link">
    <particle_emitter name="smoke" type="box">
      <emitting>true</emitting><size>{length} 6 3</size><particle_size>1 1 1</particle_size>
      <lifetime>12</lifetime><min_velocity>0.02</min_velocity><max_velocity>0.15</max_velocity>
      <scale_rate>0.1</scale_rate><rate>{spec['particles']}</rate><particle_scatter_ratio>{spec.get('ratio', 0.65)}</particle_scatter_ratio>
      <material><diffuse>0.7 0.7 0.7 1</diffuse><specular>0.2 0.2 0.2 1</specular>
        <pbr><metal><albedo_map>{FOG}/fog.png</albedo_map></metal></pbr></material>
      <color_range_image>{FOG}/fogcolors.png</color_range_image>
    </particle_emitter></link></model>"""
    wall_x = float(spec.get("wall", 8.0)) + 0.05
    fog = (f"<fog><color>0.6 0.6 0.6 1</color><type>exp</type><density>{spec['fog']}</density></fog>"
           if "fog" in spec else "")
    corridor = ""
    if "corridor" in spec:
        half = float(spec["corridor"]) / 2.0
        for side, y in (("left", half + 0.05), ("right", -half - 0.05)):
            corridor += f"""
  <model name="side_{side}"><static>true</static><pose>{(wall_x - 6.0) / 2.0} {y} 3 0 0 0</pose><link name="l">
    <collision name="c"><geometry><box><size>{wall_x + 6.0} 0.1 6</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>{wall_x + 6.0} 0.1 6</size></box></geometry>
      <material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{textures}/wall.png</albedo_map></metal></pbr></material>
    </visual></link></model>"""
        if "ceiling" in spec:
            corridor += f"""
  <model name="ceiling"><static>true</static><pose>{(wall_x - 6.0) / 2.0} 0 {float(spec['ceiling']) + 0.05} 0 0 0</pose><link name="l">
    <collision name="c"><geometry><box><size>{wall_x + 6.0} {2 * half} 0.1</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>{wall_x + 6.0} {2 * half} 0.1</size></box></geometry>
      <material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{textures}/floor.png</albedo_map></metal></pbr></material>
    </visual></link></model>"""
    if "panel" in spec:
        albedo = float(spec.get("albedo", 1.0))
        wall_x = 60.0
        extra += f"""
  <model name="panel"><static>true</static><pose>{0.32 + float(spec['panel']) + 0.05} 0 {height} 0 0 0</pose><link name="l">
    <visual name="v"><geometry><box><size>0.1 4 4</size></box></geometry>
      <material><ambient>0 0 0 1</ambient><diffuse>{albedo} {albedo} {albedo} 1</diffuse><specular>0 0 0 1</specular>
        <pbr><metal><metalness>0</metalness><roughness>1</roughness></metal></pbr></material>
    </visual></link></model>"""
    return f"""<?xml version="1.0"?>
<sdf version="1.10"><world name="smoke_rig">
  <physics name="p" type="ode"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <!-- No gravity: the carrier floats where it is set (a static model does not move when set, a falling one would
       not stay); its IMU is computed, gravity included. -->
  <gravity>0 0 0</gravity>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <plugin filename="gz-sim-particle-emitter-system" name="gz::sim::systems::ParticleEmitter"/>
  <scene><ambient>0 0 0 1</ambient><background>0 0 0 1</background><grid>false</grid>{fog}</scene>
  <atmosphere type="adiabatic"><temperature>288</temperature><temperature_gradient>0</temperature_gradient></atmosphere>
  <model name="floor"><static>true</static><pose>4 0 -0.05 0 0 0</pose><link name="l">
    <collision name="c"><geometry><box><size>40 12 0.1</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>40 12 0.1</size></box></geometry>
      <material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{textures}/floor.png</albedo_map></metal></pbr></material>
    </visual></link></model>
  {corridor}
  <model name="wall"><static>true</static><pose>{wall_x} 0 3 0 0 0</pose><link name="l">
    <collision name="c"><geometry><box><size>0.1 12 6</size></box></geometry></collision>
    <visual name="v"><geometry><box><size>0.1 12 6</size></box></geometry>
      <material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{textures}/wall.png</albedo_map></metal></pbr></material>
      <plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal"><temperature>310</temperature></plugin>
    </visual></link></model>{extra}
  <include><uri>model://carrier</uri><pose>0 0 {height} 0 0 0</pose></include>
</world></sdf>
"""


def heat_source_sdf(x: float, z: float, radius: float, kelvin: float, textures: Path) -> str:
    """A heat source (roadmap item 32): a static textured sphere the pair and the lidar see as a body, tagged
    with its surface temperature for the thermal camera (the world's temperatures are tags, not physics)."""
    return f"""<?xml version="1.0"?>
<sdf version="1.10"><model name="heat_source"><static>true</static><pose>{x} 0 {z} 0 0 0</pose><link name="l">
  <collision name="c"><geometry><sphere><radius>{radius}</radius></sphere></geometry></collision>
  <visual name="v"><geometry><sphere><radius>{radius}</radius></sphere></geometry>
    <material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{textures}/wall.png</albedo_map></metal></pbr></material>
    <plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal"><temperature>{kelvin}</temperature></plugin>
  </visual></link></model></sdf>"""


def textures(out: Path) -> Path:
    """A wall and a floor of random texture (seeded), the matcher's work in the light."""
    directory = out / "textures"
    if not (directory / "wall.png").exists():
        directory.mkdir(parents=True, exist_ok=True)
        rng = np.random.default_rng(18)
        for name, scale in (("wall", 8), ("floor", 16)):
            coarse = rng.random((512 // scale, 512 // scale))
            fine = np.kron(coarse, np.ones((scale, scale)))[:512, :512]
            grey = (40 + 180 * (0.6 * fine + 0.4 * rng.random((512, 512)))).astype(np.uint8)
            write_png(directory / f"{name}.png", np.stack([grey] * 3, axis=-1))
    return directory


def write_png(path: Path, rgb: np.ndarray) -> None:
    import struct
    import zlib
    height, width, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[row].tobytes() for row in range(height))
    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def decode(key: str, message) -> np.ndarray:
    if isinstance(message, Image):
        if key == "depth":
            return np.frombuffer(message.data, dtype=np.float32).reshape(message.height, message.width).copy()
        if key in ("thermal", "thermal_right", "thermal_core"):
            return np.frombuffer(message.data, dtype=np.uint16).reshape(message.height, message.width).copy()
        return np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3).copy()
    offsets = {field.name: field.offset for field in message.field}
    raw = np.frombuffer(message.data, dtype=np.uint8).reshape(-1, message.point_step)
    xyz = np.stack([raw[:, offsets[axis]:offsets[axis] + 4].copy().view(np.float32)[:, 0] for axis in "xyz"], axis=-1)
    if "intensity" in offsets:
        # The return's intensity as the relay writes it (roadmap item 18 stage 2's smoke signature).
        intensity = raw[:, offsets["intensity"]:offsets["intensity"] + 4].copy().view(np.float32)[:, 0]
        return np.concatenate([xyz, intensity[:, None]], axis=-1).reshape(message.height, message.width, 4)
    return xyz.reshape(message.height, message.width, 3)


class Collector:
    def __init__(self, node: Node, topics: dict[str, str]):
        self.frames: dict[str, list] = {key: [] for key in topics}
        self.lock = threading.Lock()
        self.armed = False
        self.latest_s = 0.0
        for key, topic in topics.items():
            node.subscribe(SENSORS[key][1], topic, self._callback(key))

    def _callback(self, key):
        def handle(message):
            if isinstance(message, Image):
                with self.lock:
                    self.latest_s = max(self.latest_s, message.header.stamp.sec + 1e-9 * message.header.stamp.nsec)
            if not self.armed:
                return
            with self.lock:
                if len(self.frames[key]) < FRAMES:
                    self.frames[key].append(decode(key, message))
        return handle

    def complete(self) -> bool:
        with self.lock:
            return all(len(frames) >= FRAMES for frames in self.frames.values())


class Flight:
    """The carrier's path along x: rest, a smooth start, a cruise at 1 m/s, a smooth stop."""
    START_X, END_X, SPEED, RAMP_S, REST_S = -6.0, 5.0, 1.0, 2.0, 2.0

    def __init__(self):
        ramp_m = self.SPEED * self.RAMP_S / 2.0
        self.cruise_s = (self.END_X - self.START_X - 2.0 * ramp_m) / self.SPEED
        self.duration_s = self.REST_S + 2.0 * self.RAMP_S + self.cruise_s + self.REST_S

    def state(self, t: float) -> tuple[float, float, float]:
        """Position, velocity and acceleration along x at t seconds from the start: the speed rises and falls
        as half a cosine."""
        v, ramp = self.SPEED, self.RAMP_S
        ramp_m = v * ramp / 2.0
        t -= self.REST_S
        if t <= 0.0:
            return self.START_X, 0.0, 0.0
        if t < ramp:
            u = t / ramp
            return (self.START_X + v * (t / 2.0 - ramp / (2.0 * np.pi) * np.sin(np.pi * u)),
                    v * (1.0 - np.cos(np.pi * u)) / 2.0, v * np.pi / (2.0 * ramp) * np.sin(np.pi * u))
        t -= ramp
        if t < self.cruise_s:
            return self.START_X + ramp_m + v * t, v, 0.0
        t -= self.cruise_s
        if t < ramp:
            u = t / ramp
            return (self.START_X + ramp_m + v * self.cruise_s + v * (t / 2.0 + ramp / (2.0 * np.pi) * np.sin(np.pi * u)),
                    v * (1.0 + np.cos(np.pi * u)) / 2.0, -v * np.pi / (2.0 * ramp) * np.sin(np.pi * u))
        return self.END_X, 0.0, 0.0


def move(start_s: float) -> None:
    """The flight of `move=1`, in a process of its own (the Python bindings serialize the rig's callbacks with a
    mover thread until the mover starves): sets the floating carrier where the flight has it by the simulation's
    clock (every step of it), a hundred times a second, from START_S to the flight's end."""
    flight = Flight()
    node = Node()
    lock = threading.Lock()
    clock = {}

    def on_clock(message) -> None:
        with lock:
            clock["sim_s"] = message.sim.sec + 1e-9 * message.sim.nsec

    node.subscribe(Clock, "/world/smoke_rig/clock", on_clock)
    pose = Pose()
    pose.name = "carrier"
    pose.position.z = 1.5
    pose.orientation.w = 1.0
    while True:
        with lock:
            now = clock.get("sim_s")
        if now is not None:
            pose.position.x = float(flight.state(now - start_s)[0])
            node.request("/world/smoke_rig/set_pose", pose, Pose, Boolean, 200)
            if now > start_s + flight.duration_s + 2.0:
                return
        time.sleep(0.01)


class Recorder:
    """The pair as the camera driver passes it on (gazebo_stereo_depth_main.cpp): grey, the left frame's 95th
    percentile lifted to 200 (gain between one and eight, following at 0.3 a frame), both frames by one gain."""

    def __init__(self, node: Node, topics: dict[str, str], directory: Path, seconds: float,
                 start_s: float | None = None, flight: "Flight | None" = None):
        directory.mkdir(parents=True, exist_ok=True)
        self.directory = directory
        self.seconds = seconds
        self.start_s = start_s
        self.flight = flight
        self.lock = threading.Lock()
        self.pending: dict[float, dict[str, np.ndarray]] = {}
        self.gain = 1.0
        self.first = None
        self.last = None
        self.done = False
        self.rows = []
        self.files = {side: (directory / f"{side}.bin").open("wb") for side in ("left", "right")}
        for side in ("left", "right"):
            node.subscribe(Image, topics[side], self._callback(side))

    def _callback(self, side):
        def handle(message):
            if self.done:
                return
            stamp = message.header.stamp.sec + 1e-9 * message.header.stamp.nsec
            rgb = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3)
            grey = np.round(rgb @ np.array([0.299, 0.587, 0.114])).astype(np.uint8)
            with self.lock:
                if self.done:
                    return
                self.pending.setdefault(stamp, {})[side] = grey
                if len(self.pending[stamp]) == 2:
                    self._write(stamp, self.pending.pop(stamp))
        return handle

    def _write(self, stamp: float, pair: dict[str, np.ndarray]) -> None:
        if self.start_s is not None and stamp < self.start_s:
            return
        if self.first is None:
            self.first = stamp
        if stamp - self.first > self.seconds:
            self.done = True
            return
        wanted = float(np.clip(200.0 / max(1.0, np.percentile(pair["left"][::4, ::4], 95)), 1.0, 8.0))
        self.gain = 0.7 * self.gain + 0.3 * wanted
        table = np.minimum(255.0, np.round(np.arange(256) * self.gain)).astype(np.uint8)
        for side in ("left", "right"):
            frame = table[pair[side]] if self.gain > 1.001 else pair[side]
            offset = self.files[side].tell()
            self.files[side].write(frame.tobytes())
            self.rows.append(f"{side},{stamp:.9f},0,{frame.shape[1]},{frame.shape[0]},{offset}")
        self.last = stamp

    def finish(self) -> None:
        with self.lock:
            self.done = True
            for handle in self.files.values():
                handle.close()
            (self.directory / "frames.csv").write_text(
                "side,stamp_s,wall_s,width,height,byte_offset\n" + "\n".join(self.rows) + "\n")
            if self.first is None:
                return
            # The IMU (forward, left, up) reads the carrier's motion along x and gravity; still, gravity alone.
            stamps = np.arange(self.first - 2.0, self.last + 1.0, 0.005)
            states = [self.flight.state(s - self.start_s) if self.flight else (0.0, 0.0, 0.0) for s in stamps]
            (self.directory / "imu_gz.csv").write_text("stamp_s,gx,gy,gz,ax,ay,az\n" + "".join(
                f"{s:.6f},0,0,0,{state[2]:.6f},0,9.80665\n" for s, state in zip(stamps, states)))
            (self.directory / "truth.csv").write_text("stamp_s,x,y,z,qw,qx,qy,qz\n" + "".join(
                f"{s:.6f},{state[0]:.6f},0,1.5,1,0,0,0\n" for s, state in list(zip(stamps, states))[::4]))


def run_level(out: Path, level: str) -> None:
    name, *pairs = level.split(",")
    spec = dict(pair.split("=", 1) for pair in pairs)
    light = spec.get("light", "pair")
    models = out / "models"
    carrier(models, light, spec.get("noise"))
    world = out / f"{name}.sdf"
    world.write_text(world_sdf(spec, textures(out)))
    env = dict(os.environ)
    env["GZ_SIM_RESOURCE_PATH"] = f"{models}:{REPOSITORY}/drone_city_nav/models:" + env.get("GZ_SIM_RESOURCE_PATH", "")
    log = (out / f"{name}.log").open("w")
    rendering = [] if spec.get("windowed") == "1" else ["--headless-rendering"]
    processes = [subprocess.Popen(["gz", "sim", "-s", "-r", *rendering, str(world)], env=env,
                                  stdout=log, stderr=subprocess.STDOUT, start_new_session=True)]
    sigma = float(spec.get("sigma", 0.0))
    x0 = float(spec.get("x0", 1.0))
    length = float(spec.get("len", 3.0))
    if spec.get("inside") == "1":
        length += x0 + 3.0
        x0 = -3.0
    modelled = "particles" not in spec and "panel" not in spec and "fog" not in spec
    try:
        if modelled:
            smoke_z = float(spec.get("smokez", spec.get("height", 1.5)))
            zones = ["-p", f"zones:={x0 + length / 2},0,{smoke_z},{length},6,3,{sigma}"] if sigma > 0 else []
            processes.append(subprocess.Popen(
                [str(NODE), "--ros-args", "-p", "world_name:=smoke_rig", "-p", "model_name:=carrier",
                 "-p", "model_uri:=model://carrier", *zones, "-p", "seed:=18"],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            share = 0.0 if light == "off" else 1.0
            processes.append(subprocess.Popen(
                ["ros2", "topic", "pub", "-r", "2", "/evaluation/light_share", "std_msgs/msg/Float64",
                 f"{{data: {share}}}"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True))
        node = Node()
        for _ in range(60):
            time.sleep(1.0)
            listed = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True).stdout.split()
            if any(topic.endswith("stereo_left/image") for topic in listed):
                break
        prefix = "/world/smoke_rig/model/carrier/link/"
        topics = {key: prefix + tail + ("/smoke" if modelled and key in RELAYED else "")
                  for key, (tail, _) in SENSORS.items()}
        collector = Collector(node, topics)
        # The carried light as a flight sets it (scripts/carried_light.py); the model's light is dark until set.
        intensity = 0.0 if light == "off" else 2.0
        message = light_message(Light(), intensity)
        # The service replaces the whole light, its place on the carrier as well.
        message.pose.position.x = 0.02
        if light == "offset":
            message.pose.position.y, message.pose.position.z = 0.45, -0.55
        for _ in range(10):
            ok, reply = node.request("/world/smoke_rig/light_config", message, Light, Boolean, 2000)
            if ok and reply.data:
                break
            time.sleep(1.0)
        if "heat" in spec:
            # The source as a flight creates it (simulation_smoke_sensor_node): the running world's create service.
            kelvin, radius = float(spec["heat"]), float(spec.get("heatr", 0.3))
            source = EntityFactory()
            source.sdf = heat_source_sdf(float(spec.get("heatx", 4.0)), float(spec.get("heatz", spec.get("height", 1.5))), radius,
                                         kelvin, textures(out))
            source.name = "heat_source"
            source.allow_renaming = False
            ok, reply = node.request("/world/smoke_rig/create", source, EntityFactory, Boolean, 5000)
            if not (ok and reply.data):
                raise RuntimeError("the heat source could not be created")
        time.sleep(WARM_UP_S)
        collector.armed = True
        deadline = time.time() + 60.0
        while time.time() < deadline and not collector.complete():
            time.sleep(0.5)
        if "record" in spec:
            flight = Flight() if spec.get("move") == "1" else None
            start_s = None
            if flight:
                with collector.lock:
                    start_s = collector.latest_s + 3.0
                processes.append(subprocess.Popen([sys.executable, __file__, "--move", str(start_s)], env=env,
                                                  stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            recorder = Recorder(node, topics, out / f"{name}_vio",
                                flight.duration_s if flight else float(spec["record"]), start_s, flight)
            deadline = time.time() + 3.0 * (recorder.seconds + 3.0) + 30.0
            while time.time() < deadline and not recorder.done:
                time.sleep(0.5)
            recorder.finish()
        with collector.lock:
            arrays = {key: np.stack(frames) for key, frames in collector.frames.items() if frames}
        meta = {"sigma": sigma, "x0": x0, "len": length, "light": light, "particles": float(spec.get("particles", 0)),
                "heat": float(spec.get("heat", 0)), "heatr": float(spec.get("heatr", 0.3)), "heatx": float(spec.get("heatx", 4.0)), "heatz": float(spec.get("heatz", spec.get("height", 1.5))),
                "panel": float(spec.get("panel", 0)), "albedo": float(spec.get("albedo", 0)),
                "noise": spec.get("noise", "model"), "wall": float(spec.get("wall", 8.0)),
                "height": float(spec.get("height", 1.5))}
        np.savez_compressed(out / f"{name}.npz", **arrays, meta=np.array([str(meta)]))
        print(f"{name}: " + ", ".join(f"{key} {len(frames)}" for key, frames in collector.frames.items()), flush=True)
    finally:
        # The next level's world has the same topics: nothing of this one may listen on.
        if "node" in locals():
            for topic in node.subscribed_topics():
                node.unsubscribe(topic)
        for process in reversed(processes):
            try:
                os.killpg(process.pid, 15)
                process.wait(timeout=20)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                os.killpg(process.pid, 9)
        log.close()


def main() -> int:
    if sys.argv[1] == "--move":
        move(float(sys.argv[2]))
        return 0
    out = Path(sys.argv[1]).resolve()
    out.mkdir(parents=True, exist_ok=True)
    for level in sys.argv[2:]:
        run_level(out, level)
    return 0


if __name__ == "__main__":
    sys.exit(main())
