# Security Policy

## Scope

This repository is a PX4/Gazebo simulation project for drone navigation
research and development. It is not certified for real aircraft operation and
does not provide real-world flight safety guarantees.

Security-sensitive areas include:

- scripts that launch containers, Gazebo, PX4 SITL, or ROS 2 processes;
- files that may contain logs, bags, or generated simulator artifacts;
- configuration that affects offboard control or terminal behavior;
- dependency setup scripts that fetch external projects.

## Reporting

Please report security concerns privately to the maintainer before public
disclosure. If you do not have a direct contact channel, open a GitHub security
advisory or a private report through the hosting platform if available.

Do not include secrets, private logs, or sensitive environment details in a
public issue.

## Safety Notice

This project is intended for simulation. Running any part of this stack on real
hardware requires an independent safety review, hardware-specific failsafes,
controlled test procedures, and compliance with applicable laws and
regulations.
