#!/usr/bin/env python3
"""smoke_signature_report.py OUT_DIR [NAME...]: the signature of smoke in the stereo pair's frame, from the levels
of tools/smoke_rig.py (roadmap item 18, the smoke face detector). On the host (OpenCV, python3 -I).

For each level, over its saved frames, on the depth node's stride grid (one cell per 4 px, the grid of the silent
patches, K36): the share of cells the pair's matcher leaves silent (the driver's gain, the matcher and its noise
mask as smoke_rig_report.py repeats them); the brightness of the silent cells and of the matched cells, raw (the
frame before the driver's gain, what the node reads as the brightness over the gain, K15) and gained; the share
of the silent cells brighter than each of several raw levels; the flicker of the silent cells between frames (the
standard deviation of a cell's raw brightness over the frames, the median over the cells; smoke churns, A13); the
front time-of-flight unit's returns. The centre region (the middle half of the frame each way) is reported beside
the whole frame: a plume ahead stands in the centre, the floor and the dark beyond the light at the frame's border.
"""
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from smoke_rig_report import gained, load, stereo  # noqa: E402


def contrast(gained_left: np.ndarray) -> tuple[np.ndarray, float]:
    """The local standard deviation the noise mask judges a match by (9 px window) and the frame's noise."""
    laplacian = cv2.filter2D(gained_left, cv2.CV_32F, np.array([[1, -2, 1], [-2, 4, -2], [1, -2, 1]], np.float32))
    noise = 1.4826 * (np.floor(np.median(np.abs(laplacian[1:-1:4, 1:-1:4]))) + 0.5) / 6.0
    grey = gained_left.astype(np.float32)
    mean = cv2.boxFilter(grey, cv2.CV_32F, (9, 9))
    variance = cv2.boxFilter(grey * grey, cv2.CV_32F, (9, 9)) - mean * mean
    return np.sqrt(np.maximum(variance, 0.0)), float(max(noise, 1.0))

STRIDE = 4
LEVELS = (20, 40, 60, 80, 100, 120)
# The silent patch's judgment as stereo_silent_patches.cpp makes it (specification K36): cells of the stride grid,
# the rim the hits beside them, 200 cells or more, 20 rim hits, 6 of 8 sectors, the rim's median within the
# confident depth and its quartiles within 0.5 m. A hit is a match within the confident depth; a match beyond it
# is a free ray, neither silence nor rim.
LEAST_CELLS, LEAST_RIM_HITS, LEAST_RIM_SECTORS, RIM_FLAT_M, CONFIDENT_DEPTH_M = 200, 20, 6, 0.5, 5.2


def patches(silent: np.ndarray, depth: np.ndarray) -> list[dict]:
    """The frame's silent patches on the stride grid, judged as the depth node judges them."""
    hit = (~silent) & np.isfinite(depth) & (depth <= CONFIDENT_DEPTH_M)
    count, labels = cv2.connectedComponents(silent.astype(np.uint8), connectivity=4)
    rows, columns = silent.shape
    found = []
    kernel = np.ones((3, 3), np.uint8)
    for label in range(1, count):
        cells = labels == label
        size = int(cells.sum())
        if size < LEAST_CELLS:
            continue
        ring = cv2.dilate(cells.astype(np.uint8), kernel).astype(bool) & ~cells & hit
        rim_rows, rim_columns = np.nonzero(ring)
        cell_rows, cell_columns = np.nonzero(cells)
        patch = {"cells": size, "mask": cells, "rim_hits": int(ring.sum()), "rim_depth": float("nan"),
                 "rim_spread": float("nan"), "closed": False}
        at_border = cell_rows.min() == 0 or cell_columns.min() == 0 or cell_rows.max() == rows - 1 or \
            cell_columns.max() == columns - 1
        centre_row, centre_column = cell_rows.mean(), cell_columns.mean()
        angles = np.arctan2(rim_rows - centre_row, rim_columns - centre_column)
        sectors = len(set(np.floor((angles + np.pi) / (np.pi / 4.0)).astype(int) % 8))
        if at_border:
            patch["reason"] = "at_border"
        elif patch["rim_hits"] < LEAST_RIM_HITS:
            patch["reason"] = "rim_unmatched"
        elif sectors < LEAST_RIM_SECTORS:
            patch["reason"] = "rim_not_round"
        else:
            depths = np.sort(depth[ring])
            quantile = lambda share: depths[int(share * (len(depths) - 1))]  # noqa: E731
            patch["rim_depth"] = float(quantile(0.5))
            patch["rim_spread"] = float(quantile(0.75) - quantile(0.25))
            if patch["rim_depth"] > CONFIDENT_DEPTH_M:
                patch["reason"] = "rim_beyond_reach"
            elif patch["rim_spread"] > RIM_FLAT_M:
                patch["reason"] = "rim_recedes"
            else:
                patch["reason"] = "closed"
                patch["closed"] = True
        found.append(patch)
    return found


def grey_of(rgb: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)


def percentiles(values: np.ndarray) -> str:
    if values.size == 0:
        return "-"
    p10, p50, p90 = np.percentile(values, (10, 50, 90))
    return f"{p50:.0f} ({p10:.0f}..{p90:.0f})"


def level_row(out: Path, name: str) -> list[str]:
    data, meta = load(out, name)
    frames = min(len(data["left"]), len(data["right"]))
    silent_stack, raw_stack, depth_stack, contrast_stack, gain_values = [], [], [], [], []
    for index in range(frames):
        left = grey_of(data["left"][index])
        right = grey_of(data["right"][index])
        gained_left, gained_right = gained(left, right)
        gain_values.append(float(np.clip(200.0 / max(1.0, np.percentile(left[::4, ::4], 95)), 1.0, 8.0)))
        matched, depth = stereo(gained_left, gained_right)
        deviation, noise = contrast(gained_left)
        contrast_stack.append(deviation[::STRIDE, ::STRIDE] / noise)
        silent_stack.append(~matched[::STRIDE, ::STRIDE])
        raw_stack.append(left[::STRIDE, ::STRIDE].astype(np.float32))
        depth_stack.append(depth[::STRIDE, ::STRIDE])
    silent = np.stack(silent_stack)  # frames x rows x columns
    raw = np.stack(raw_stack)
    rows, columns = silent.shape[1:]
    centre = np.zeros((rows, columns), dtype=bool)
    centre[rows // 4: 3 * rows // 4, columns // 4: 3 * columns // 4] = True
    cells = []
    for region_name, region in (("whole", np.ones_like(centre)), ("centre", centre)):
        mask = silent & region[None]
        silent_share = mask.sum() / (region.sum() * frames)
        silent_raw = raw[mask]
        matched_raw = raw[(~silent) & region[None]]
        brighter = " ".join(f"{np.mean(silent_raw > level):.2f}" if silent_raw.size else "-" for level in LEVELS)
        # The flicker: cells silent in every frame, their raw brightness's deviation over the frames.
        always = mask.all(axis=0)
        flicker = np.median(raw[:, always].std(axis=0)) if always.any() else float("nan")
        cells.append(f"{silent_share:.2f} | {percentiles(silent_raw)} | {percentiles(matched_raw)} | {brighter} | "
                     f"{flicker:.1f}")
    # The frame's silent patches judged as K36 does: how many of each verdict over the frames, and the closed
    # ones (a surface or a smoke face at the rim's depth): cells, rim depth and spread, the raw brightness of
    # their cells. The frames' values are medians.
    verdicts: dict[str, int] = {}
    closed_rows = []
    for index in range(frames):
        for patch in patches(silent[index], depth_stack[index]):
            verdicts[patch["reason"]] = verdicts.get(patch["reason"], 0) + 1
            if patch["closed"]:
                mask = patch["mask"]
                ring = cv2.dilate(mask.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool) & ~mask
                rim_contrast = float(np.median(contrast_stack[index][ring & ~silent[index]]))
                # Matches the patch encloses (holes in the silence) beyond the rim: the pair sees through it.
                # A hole is a region of non-patch cells that does not reach the frame's border through
                # non-patch cells.
                count, labels = cv2.connectedComponents((~mask).astype(np.uint8), connectivity=4)
                border = set(np.unique(np.concatenate([labels[0], labels[-1], labels[:, 0], labels[:, -1]])))
                holes = ~mask & ~np.isin(labels, list(border))
                beyond = int(np.sum(holes & ~silent[index] & (depth_stack[index] > patch["rim_depth"] + 1.0)))
                closed_rows.append((patch["cells"], patch["rim_depth"], patch["rim_spread"],
                                    float(np.median(raw[index][mask])), rim_contrast, beyond))
    verdict_text = " ".join(f"{reason} {count / frames:.1f}" for reason, count in sorted(verdicts.items())) or "-"
    if closed_rows:
        closed = np.median(np.array(closed_rows), axis=0)
        closed_text = (f"{closed[0]:.0f} | {closed[1]:.1f} | {closed[2]:.2f} | {closed[3]:.0f} | {closed[4]:.1f} | "
                       f"{closed[5]:.0f}")
    else:
        closed_text = "- | - | - | - | - | -"
    patch_text = f"{verdict_text} | {closed_text}"
    tof_cloud = data["tof_front"]
    tof = np.linalg.norm(tof_cloud[..., :3], axis=-1)
    returns = np.isfinite(tof) & (tof < 2.8)
    tof_text = f"{returns.mean():.2f}" + (f" at {np.median(tof[returns]):.2f} m" if returns.any() else "")
    if tof_cloud.shape[-1] == 4 and returns.any():
        tof_text += f", weak {np.mean(tof_cloud[..., 3][returns] < 0.5):.2f}"
    spec = f"sigma {meta['sigma']:g}"
    if meta["sigma"] > 0:
        spec += f" x0 {meta['x0']:g} len {meta['len']:g}"
    if meta.get("panel"):
        spec += f" panel {meta['panel']} albedo {meta.get('albedo', 1.0)}"
    spec += f" wall {meta.get('wall', 8.0):g}"
    return [name, spec, f"{np.mean(gain_values):.1f}"] + cells + [patch_text, tof_text]


def main() -> int:
    out = Path(sys.argv[1])
    names = sys.argv[2:] or sorted(path.stem for path in out.glob("*.npz"))
    header = ("| level | spec | gain | silent (whole) | silent raw p50 (p10..p90) | matched raw p50 | share of silent "
              "brighter than " + "/".join(map(str, LEVELS)) + " | flicker | silent (centre) | silent raw | matched raw "
              "| brighter | flicker | patches a frame by verdict | closed: cells | rim m | rim spread m "
              "| raw p50 | rim contrast / noise | matches beyond rim in holes | ToF front |")
    print(header)
    print("|" + "---|" * (header.count("|") - 1))
    for name in names:
        try:
            print("| " + " | ".join(level_row(out, name)) + " |")
        except (FileNotFoundError, KeyError, ValueError) as error:
            print(f"| {name} | not read: {error} |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
