#!/usr/bin/env python3
"""record_index.py SCENARIO RUN DIRECTORY: one row of a recording batch's index (DIRECTORY/index.md), from the flight's
check (log/tools/run_RUN.log), the host's verdict and the recorder's output (log/tools/record_RUN.log). Prints "RETRY"
when the flight failed its check or the recording is not one, so that the batch flies the scenario again."""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

HEADER = ("| Scenario | Run | Mission check | Speed, m/s | True position, m | Gave the goal up | Host (A7) "
          "| Recording | Files |\n|---|---|---|---|---|---|---|---|---|\n")


def row(scenario: str, run: str, directory: Path) -> tuple[str, bool]:
    check = Path(f"log/tools/run_{run}.log").read_text(errors="ignore")
    record = Path(f"log/tools/record_{run}.log")
    recording = record.read_text(errors="ignore") if record.exists() else ""
    fails = [line[6:].strip() for line in check.splitlines()
             if line.startswith("FAIL:") and "clean" not in line]
    speed = re.search(r"mean flight speed is ([\d.]+) m/s", check)
    truths = re.findall(r"the true position is ([\d.]+) m from the goal", check)
    trigger = re.search(r"given up by the (\w+) trigger", check)
    whole = "vehicle is whole" in check
    host = subprocess.run([sys.executable, "tools/host_verdict.py", run],
                          capture_output=True, text=True, check=False).stdout.strip()
    files = sorted(path.name for path in directory.glob(f"*_{run}_*.mp4"))
    good = recording.count("RECORDING ok") == 2
    passed = not fails and "crash was reported" not in "".join(fails)
    verdict = "pass" if passed else "FAIL: " + "; ".join(fails)[:120]
    if whole:
        verdict += " (whole)"
    text = (f"| {scenario} | {run} | {verdict} | {speed.group(1) if speed else '-'} | "
            f"{' / '.join(truths) or '-'} | {trigger.group(1) if trigger else '-'} | "
            f"{host.replace('HOST ', '').replace(run, '').strip(' :')} | "
            f"{'ok' if good else 'BAD'} | {', '.join(files) or '-'} |\n")
    return text, passed and good


def main() -> int:
    scenario, run, directory = sys.argv[1], sys.argv[2], Path(sys.argv[3])
    index = directory / "index.md"
    if not index.exists():
        index.write_text("# Recorded flights\n\nA recorded flight is flown slowed against the wall clock and judged like "
                         "any other (specification A7, A11): the host's verdict is in its row.\n\n" + HEADER)
    text, good = row(scenario, run, directory)
    with index.open("a") as stream:
        stream.write(text)
    print("OK" if good else "RETRY")
    return 0


if __name__ == "__main__":
    sys.exit(main())
