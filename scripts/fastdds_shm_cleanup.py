#!/usr/bin/env python3
"""Remove the shared-memory files Fast DDS processes left behind when they were stopped from outside.

fastdds_shm_cleanup.py [--directory DIR] [--dry-run]

Every participant of a flight holds a segment in /dev/shm (64 MB each with docker/fastdds_profiles.xml, which the
container shares with the host) and a lock file beside it, `<segment>_el`; a port has `<port>_el` or `<port>_sl` and a
mutex. A process killed with its container leaves them: after ten hours of flights and replays 241 files held 5.6 GB
of the host's memory (2026-11-06). A lock file no living process holds marks a zombie, as Fast DDS's own
`fastdds shm clean` reads it (the tool lives in the container, this runs on the host after a simulation is stopped):
the lock, its segment or port, and the port's mutex go; what a living process holds stays.
"""

from __future__ import annotations

import argparse
import fcntl
import re
import sys
from pathlib import Path

SEGMENT_LOCK = re.compile(r"^fastrtps_[0-9a-z]{16}(_el|_sl)$")
PORT_LOCK = re.compile(r"^fastrtps_port\d{1,5}(_el|_sl)$")


def held(lock: Path) -> bool:
    """Whether a living process holds the lock file."""
    try:
        with lock.open("ab") as handle:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return False
    except OSError:
        return True


def zombies(directory: Path) -> list[Path]:
    """The files of the segments and ports no living process holds."""
    found: list[Path] = []
    for lock in sorted(directory.iterdir()):
        is_port = PORT_LOCK.match(lock.name) is not None
        if not (is_port or SEGMENT_LOCK.match(lock.name)) or held(lock):
            continue
        owned = directory / lock.name[:-3]
        found += [lock, owned]
        if is_port:
            found.append(directory / f"sem.{owned.name}_mutex")
    return found


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--directory", type=Path, default=Path("/dev/shm"))
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    if not args.directory.is_dir():
        return 0
    files = [path for path in zombies(args.directory) if path.exists()]
    freed = sum(path.stat().st_blocks * 512 for path in files)
    if not args.dry_run:
        for path in files:
            try:
                path.unlink()
            except OSError:
                pass
    if files:
        verb = "would remove" if args.dry_run else "removed"
        print(f"Fast DDS shared memory: {verb} {len(files)} files of stopped processes, {freed / 1.0e6:.0f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
