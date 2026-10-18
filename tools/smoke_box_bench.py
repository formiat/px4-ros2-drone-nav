#!/usr/bin/env python3
"""smoke_box_bench.py OUT: how a see-through smoke box renders in a dark world under the vehicle's own spot light,
material by material (the recording's smoke boxes, K26): one gz run a variant, one camera frame a run, PNGs and the
frames' mean brightness in OUT. Measured on 2026-10-18: emissive 0.2 with the transparency of 1/m smoke is a grey veil
the wall shows through (mean 80 against 63 without a box), emissive 0.62 all but a wall (100), a plain or unlit
material invisible (63). Inside the container: python3 tools/smoke_box_bench.py log/tools/release/v0.6.0/box_bench"""
import os, subprocess, sys, time, threading
from pathlib import Path
import numpy as np
sys.path.insert(0, "/workspace/tools")
import smoke_rig  # textures, write_png, decode
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

OUT = Path(sys.argv[1]); OUT.mkdir(parents=True, exist_ok=True)
GREY = "0.62 0.62 0.66"
VARIANTS = {
    "plain": f"<ambient>{GREY} 0.39</ambient><diffuse>{GREY} 0.39</diffuse><specular>0 0 0 1</specular>",
    "emissive02": f"<ambient>{GREY} 0.39</ambient><diffuse>{GREY} 0.39</diffuse><specular>0 0 0 1</specular><emissive>0.2 0.2 0.21 1</emissive>",
    "emissive062": f"<ambient>{GREY} 0.39</ambient><diffuse>{GREY} 0.39</diffuse><specular>0 0 0 1</specular><emissive>{GREY} 1</emissive>",
    "unlit": f"<lighting>false</lighting><ambient>{GREY} 0.39</ambient><diffuse>{GREY} 0.39</diffuse>",
    "emissive02_notransp": f"<ambient>{GREY} 1</ambient><diffuse>{GREY} 1</diffuse><emissive>0.2 0.2 0.21 1</emissive>",
}

def world(material: str, transparency: float, textures: Path) -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.10"><world name="bench">
  <physics name="p" type="ode"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <gravity>0 0 0</gravity>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <scene><ambient>0 0 0 1</ambient><background>0 0 0 1</background><grid>false</grid></scene>
  <model name="floor"><static>true</static><pose>6 0 -0.05 0 0 0</pose><link name="l">
    <visual name="v"><geometry><box><size>30 16 0.1</size></box></geometry>
      <material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{textures}/floor.png</albedo_map></metal></pbr></material></visual></link></model>
  <model name="wall"><static>true</static><pose>10 0 3 0 0 0</pose><link name="l">
    <visual name="v"><geometry><box><size>0.1 16 6</size></box></geometry>
      <material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{textures}/wall.png</albedo_map></metal></pbr></material></visual></link></model>
  <model name="smoke"><static>true</static><pose>5 0.8 1.5 0 0 0</pose><link name="l">
    <visual name="v"><geometry><box><size>3 3 3</size></box></geometry><material>{material}</material>
      <transparency>{transparency}</transparency><cast_shadows>false</cast_shadows></visual></link></model>
  <model name="eye"><static>true</static><pose>0 0 1.5 0 0 0</pose><link name="l">
    <light name="lamp" type="spot"><pose>0.3 0 0 0 0 0</pose><diffuse>1 1 1 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>40</range><constant>0.2</constant><linear>0.02</linear><quadratic>0.004</quadratic></attenuation>
      <direction>1 0 0</direction><spot><inner_angle>1.4</inner_angle><outer_angle>1.8</outer_angle><falloff>1</falloff></spot>
      <intensity>2</intensity><cast_shadows>false</cast_shadows></light>
    <sensor name="cam" type="camera"><always_on>1</always_on><update_rate>5</update_rate><camera>
      <horizontal_fov>1.5</horizontal_fov><image><width>640</width><height>480</height></image>
      <clip><near>0.1</near><far>60</far></clip></camera></sensor></link></model>
</world></sdf>
"""

def capture(name: str, material: str, transparency: float) -> None:
    sdf = OUT / f"{name}.sdf"; sdf.write_text(world(material, transparency, smoke_rig.textures(OUT)))
    log = (OUT / f"{name}.log").open("w")
    proc = subprocess.Popen(["gz", "sim", "-s", "-r", "--headless-rendering", str(sdf)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    node = Node(); frames = []; lock = threading.Lock()
    def cb(message):
        with lock:
            if len(frames) < 12: frames.append(smoke_rig.decode("left", message))
    topic = "/world/bench/model/eye/link/l/sensor/cam/image"
    try:
        for _ in range(60):
            time.sleep(1.0)
            if topic in subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True).stdout.split():
                break
        node.subscribe(Image, topic, cb)
        deadline = time.time() + 30
        while time.time() < deadline:
            with lock:
                if len(frames) >= 12: break
            time.sleep(0.5)
        with lock:
            if frames:
                frame = frames[-1]
                rgb = frame if frame.ndim == 3 else np.stack([frame] * 3, axis=-1)
                smoke_rig.write_png(OUT / f"{name}.png", rgb.astype(np.uint8))
                print(f"{name}: {len(frames)} frames, mean {frame.mean():.1f}", flush=True)
            else:
                print(f"{name}: no frame", flush=True)
    finally:
        for t in node.subscribed_topics(): node.unsubscribe(t)
        os.killpg(proc.pid, 15); proc.wait(timeout=20); log.close()

for name, material in VARIANTS.items():
    capture(name, material, 0.0 if name.endswith("notransp") else 0.61)
