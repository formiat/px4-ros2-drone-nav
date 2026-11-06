#!/usr/bin/env python3
"""body_contact_rig.py OUT_DIR (inside the container, after `make build`): a walking body's touch is a contact the
vehicle's contact system reports (roadmap item 21, the bench before the first flight with a body).

A world of its own: a floor, a probe standing on it that carries the vehicle's contact system
(`libdrone_contact_system.so`, the plugin of the vehicle's models, publishing on the flight's topic), and one body
of scripts/moving_bodies.py walking through the probe's place at 1 m/s as it walks in a flight (created in the
running world, dropped onto the floor, its pose set fifty times a second). The contacts the system publishes are
read for 40 s: the bench passes when one names the body's collision, and prints the first.
"""
from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[1]
WORLD = "body_contact_rig"
TOPIC = "/drone_city_nav/drone_contacts"

WORLD_SDF = f"""<?xml version="1.0"?>
<sdf version="1.10"><world name="{WORLD}">
  <physics name="p" type="ode"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
  <model name="floor"><static>true</static><pose>0 0 -0.05 0 0 0</pose><link name="l">
    <collision name="c"><geometry><box><size>20 20 0.1</size></box></geometry></collision></link></model>
  <model name="probe"><pose>0 0 0.2 0 0 0</pose><link name="base_link">
    <inertial><mass>2.0</mass><inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.03</izz><ixy>0</ixy><ixz>0</ixz>
      <iyz>0</iyz></inertia></inertial>
    <collision name="frame"><geometry><box><size>0.5 0.5 0.3</size></box></geometry></collision></link>
    <plugin filename="libdrone_contact_system.so" name="drone_city_nav::DroneContactSystem">
      <topic>{TOPIC}</topic></plugin>
  </model>
</world></sdf>
"""


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    out = Path(sys.argv[1]).resolve()
    out.mkdir(parents=True, exist_ok=True)
    world = out / f"{WORLD}.sdf"
    world.write_text(WORLD_SDF)
    env = dict(os.environ)
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = (f"{REPOSITORY}/install/drone_city_nav/lib:"
                                        + env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""))
    log = (out / f"{WORLD}.log").open("w")
    processes = [subprocess.Popen(["gz", "sim", "-s", "-r", str(world)], env=env, stdout=log,
                                  stderr=subprocess.STDOUT, start_new_session=True)]
    touched: list[str] = []
    lock = threading.Lock()
    try:
        from gz.msgs10.contacts_pb2 import Contacts
        from gz.transport13 import Node

        node = Node()

        def on_contacts(message: Contacts) -> None:
            with lock:
                for contact in message.contact:
                    if "warm_body" in contact.collision2.name and not touched:
                        touched.append(f"{contact.collision1.name} against {contact.collision2.name}")

        node.subscribe(Contacts, TOPIC, on_contacts)
        time.sleep(3.0)
        processes.append(subprocess.Popen(
            [sys.executable, str(REPOSITORY / "scripts/moving_bodies.py"), "--world", WORLD,
             "--bodies=-3,0,3,0,0,1.0", "--seed", "1", "--output", str(out / "moving_bodies.csv")],
            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.time() + 40.0
        while time.time() < deadline:
            with lock:
                if touched:
                    break
            time.sleep(0.2)
    finally:
        for process in reversed(processes):
            try:
                os.killpg(process.pid, 15)
                process.wait(timeout=20)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                os.killpg(process.pid, 9)
        log.close()
    if touched:
        print(f"BODY_CONTACT reported=true {touched[0]}")
        return 0
    print("BODY_CONTACT reported=false: no contact named the body in 40 s")
    return 1


if __name__ == "__main__":
    sys.exit(main())
