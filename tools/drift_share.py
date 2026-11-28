#!/usr/bin/env python3
"""drift_share.py RUN...: the estimate's horizontal error against the truth as a share of the path flown
(specification K95: what the published sigma of an estimate with no absolute position is set by). Each flight's flown
estimate (mppi/mppi_track.jsonl) is set against its true pose (gz_pose.csv) as tools/survey_sheets.py sets them,
sampled every 5 s past the flight's first 50 m of true path; printed a flight: the path, the error's median, most and
end, and the share's median, 95th percentile and most; then the same over all the samples. A RUN is looked for in
log/runs and then in the kept records (log/tools/gpscl/part5/runs). Host.

  python3 tools/drift_share.py r4235 r4240 r4241 r4242 r4312 r4313
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPOSITORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY / "tools"))
import survey_sheets  # noqa: E402

SAMPLE_S = 5.0
FIRST_PATH_M = 50.0


def shares(run_directory: Path) -> tuple[np.ndarray, np.ndarray, float]:
    """The error's share of the path at the samples, the horizontal error over the flight, the path."""
    stamps, error, true = survey_sheets.errors(run_directory)
    path = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(true[:, :2], axis=0), axis=1))])
    horizontal = np.linalg.norm(error[:, :2], axis=1)
    pick = np.searchsorted(stamps, np.arange(stamps[0], stamps[-1], SAMPLE_S))
    pick = pick[path[pick] > FIRST_PATH_M]
    return horizontal[pick] / path[pick], horizontal, float(path[-1])


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    samples = []
    for run in sys.argv[1:]:
        directory = next((root / run for root in (REPOSITORY / "log/runs", REPOSITORY / "log/tools/gpscl/part5/runs")
                          if (root / run / "mppi/mppi_track.jsonl").is_file()), None)
        if directory is None:
            print(f"{run}: no records")
            continue
        share, horizontal, path = shares(directory)
        samples.append(share)
        print(f"{run}: path {path:.0f} m, error p50 {np.median(horizontal):.2f} max {horizontal.max():.2f} end "
              f"{horizontal[-1]:.2f} m; share of the path p50 {100 * np.median(share):.2f} p95 "
              f"{100 * np.percentile(share, 95):.2f} max {100 * share.max():.2f} %")
    if samples:
        every = np.concatenate(samples)
        print(f"all {len(every)} samples: share of the path p50 {100 * np.median(every):.2f} p90 "
              f"{100 * np.percentile(every, 90):.2f} p95 {100 * np.percentile(every, 95):.2f} p99 "
              f"{100 * np.percentile(every, 99):.2f} max {100 * every.max():.2f} %")
    return 0


if __name__ == "__main__":
    sys.exit(main())
