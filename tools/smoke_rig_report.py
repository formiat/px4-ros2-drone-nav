#!/usr/bin/env python3
"""smoke_rig_report.py OUT_DIR NAME... [--baseline NAME] | --calibrate NAME...: what the sensors saw at each level
of tools/smoke_rig.py, on the host (OpenCV), averaged over the saved frames (roadmap item 18 stage 1).

Per level: the wall's brightness and contrast in the middle of the left frame (before the camera's gain); the
share of that region the pair's matcher matches as the vehicle's camera path does (the driver's automatic gain,
the depth node's semi-global matcher and its noise mask), and of it the matches in the smoke and on the wall; corners the feature tracker would take (its detector: quality 0.01, 20 px apart); the front time-of-flight
unit's zones returning and where; the lidar's forward rays (20 degrees each side, 10 up and down) returning from
the smoke, from the wall, and none, rays the baseline level returns within 0.7 m (the carrier itself) left out;
the thermal camera on the wall.
--calibrate: the light's response, the linear image value of a matte panel per unit albedo by distance
(simulation_smoke_model.cpp, lightResponse).
--noise: the pair's noise, the deviation of the difference of two frames of the still scene over the square root of
two, by the brightness of the pixel (the camera's noise setting, specification A12).
--vio OUT_DIR NAME...: the camera estimator replayed over a level's record (NAME_vio/replay_smoke.csv): the features
tracked, the position error at the end and at most, the path flown.
--reach OUT_DIR NAME...: the carried light's reach by the carrier's height over the floor (roadmap item 31, the root
goal of 2026-10-24 part 2.0; levels of `height=Z,wall=60`): over the whole left frame, before the gain, its mean and
95th percentile of grey and the share of its pixels under 6 levels (the unlit grey of K12); after the gain the share
the pair matches, the matched depth's median against the evaluation depth's at the same pixels, and the corners the
tracker's detector takes; and, where NAME_vio/replay_reach.csv exists, the features the estimator's tracker held over
the record (mean, least).
"""
import csv
import ast
import sys

import cv2
import numpy as np

FOCAL_PX = 640 / np.tan(np.radians(60))
BASELINE_M = 0.2
PAIR_X_M = 0.32
ROI = (slice(380, 580), slice(540, 740))


def linear(levels):
    value = np.asarray(levels, dtype=float) / 255.0
    return np.where(value <= 0.04045, value / 12.92, ((value + 0.055) / 1.055) ** 2.4)


def load(out, name):
    data = np.load(f"{out}/{name}.npz")
    return data, ast.literal_eval(str(data["meta"][0]))


def gained(left, right):
    """The camera driver's automatic gain (gazebo_stereo_depth_main.cpp): the left frame's 95th percentile of
    brightness lifted to 200, between one and eight; both frames take it."""
    gain = float(np.clip(200.0 / max(1.0, np.percentile(left[::4, ::4], 95)), 1.0, 8.0))
    table = np.minimum(255.0, np.round(np.arange(256) * gain)).astype(np.uint8)
    return table[left], table[right]


def stereo(left, right):
    """The depth node's matcher and its noise mask (stereo_depth_node.cpp): a match stands only where the 9 px
    neighbourhood's deviation clears three times the frame's noise (Immerkaer's operator, at least a level)."""
    matcher = cv2.StereoSGBM_create(0, 128, 5, 8 * 25, 32 * 25, 1, 31, 10, 100, 2, cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    disparity = matcher.compute(left, right).astype(np.float32) / 16.0
    laplacian = cv2.filter2D(left, cv2.CV_32F, np.array([[1, -2, 1], [-2, 4, -2], [1, -2, 1]], np.float32))
    noise = 1.4826 * (np.floor(np.median(np.abs(laplacian[1:-1:4, 1:-1:4]))) + 0.5) / 6.0
    grey = left.astype(np.float32)
    mean = cv2.boxFilter(grey, cv2.CV_32F, (9, 9))
    variance = cv2.boxFilter(grey * grey, cv2.CV_32F, (9, 9)) - mean * mean
    matched = (disparity > 0.5) & (variance >= (3.0 * max(noise, 1.0)) ** 2)
    return matched, np.where(matched, FOCAL_PX * BASELINE_M / np.maximum(disparity, 1e-3), np.nan)


def forward_rays(cloud):
    """The lidar's forward cone as ranges, frame by frame."""
    rows, columns = cloud.shape[1:3]
    azimuth = np.linspace(-np.pi, np.pi, columns)
    elevation = np.linspace(-np.pi / 2, np.pi / 2, rows)
    a, e = np.meshgrid(azimuth, elevation)
    cone = (np.abs(a) < np.radians(20)) & (np.abs(e) < np.radians(10))
    return np.linalg.norm(cloud[..., :3], axis=-1)[:, cone]


def report(out, names, baseline):
    base, _ = load(out, baseline)
    own = np.nanmin(np.where(np.isfinite(forward_rays(base["lidar"])), forward_rays(base["lidar"]), np.inf), axis=0) < 0.7
    print("| level | sigma 1/m | light | wall brightness | contrast | matched | in smoke | at wall | match depth m "
          "| corners | ToF front zones | lidar smoke / wall / none | thermal K |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for name in names:
        data, meta = load(out, name)
        x0, length = meta["x0"], meta["len"]
        wall_m = meta.get("wall", 8.0)
        near, far = x0 - PAIR_X_M - 0.3, x0 + length - PAIR_X_M + 0.3
        rows = []
        for index in range(min(len(data["left"]), len(data["right"]))):
            left = cv2.cvtColor(data["left"][index], cv2.COLOR_RGB2GRAY)
            right = cv2.cvtColor(data["right"][index], cv2.COLOR_RGB2GRAY)
            matched, depth = stereo(*gained(left, right))
            region = depth[ROI]
            corners = cv2.goodFeaturesToTrack(left, 400, 0.01, 20)
            rows.append([left[ROI].mean(), left[ROI].std(), matched[ROI].mean(),
                         np.mean((region > max(near, 0.1)) & (region < far)), np.mean(np.abs(region - (wall_m - PAIR_X_M)) < 0.6),
                         np.nanmedian(region) if np.isfinite(region).any() else np.nan,
                         0 if corners is None else len(corners)])
        camera = np.nanmean(np.array(rows, dtype=float), axis=0)
        tof_cloud = data["tof_front"]
        tof = np.linalg.norm(tof_cloud[..., :3], axis=-1)
        tof_returns = np.isfinite(tof) & (tof < 2.8)
        tof_text = f"{tof_returns.mean():.2f}" + (f" at {np.median(tof[tof_returns]):.2f} m" if tof_returns.any() else "")
        if tof_cloud.shape[-1] == 4 and tof_returns.any():
            # The smoke signature (roadmap item 18 stage 2): returns of under half a surface's intensity.
            weak = tof_cloud[..., 3][tof_returns] < 0.5
            tof_text += f", weak {weak.mean():.2f} (intensity p50 {np.median(tof_cloud[..., 3][tof_returns]):.3f})"
        lidar = forward_rays(data["lidar"])[:, ~own]
        lidar_x = lidar + 0.0  # the lidar stands over the carrier's origin
        smoke = np.mean((lidar_x > x0 - 0.3) & (lidar_x < x0 + length + 0.3))
        wall = np.mean(np.abs(lidar_x - wall_m) < 0.8)
        none = np.mean(~np.isfinite(lidar_x))
        thermal = data["thermal"][:, 150:350, 220:420] / 100.0
        light = meta["light"] + (f", particles {meta['particles']:g}/s" if meta["particles"] else "")
        print(f"| {name} | {meta['sigma']:g} | {light} | {camera[0]:.0f} | {camera[1]:.1f} | {camera[2]:.2f} | "
              f"{camera[3]:.2f} | {camera[4]:.2f} | {camera[5]:.2f} | {camera[6]:.0f} | {tof_text} | "
              f"{smoke:.2f} / {wall:.2f} / {none:.2f} | {thermal.mean():.1f} |")


def calibrate(out, names):
    print("| distance m | albedo | centre level | response |")
    print("|---|---|---|---|")
    for name in names:
        data, meta = load(out, name)
        centre = data["left"][:, 470:490, 630:650].astype(float)
        print(f"| {meta['panel']:g} | {meta['albedo']:g} | {centre.mean():.1f} | "
              f"{linear(centre).mean() / meta['albedo']:.3f} |")


def noise(out, names):
    bands = ((10, 30), (30, 50), (50, 80), (80, 120), (120, 200))
    print("| level | noise setting | " + " | ".join(f"grey {low}-{high}" for low, high in bands) + " |")
    print("|---|---|" + "---|" * len(bands))
    for name in names:
        data, meta = load(out, name)
        first = data["left"][0][..., 0].astype(float)
        second = data["left"][1][..., 0].astype(float)
        cells = []
        for low, high in bands:
            inside = (first >= low) & (first < high) & (second >= low) & (second < high)
            cells.append(f"{(first - second)[inside].std() / np.sqrt(2):.2f}" if inside.sum() > 1000 else "-")
        print(f"| {name} | {meta.get('noise', 'model')} | " + " | ".join(cells) + " |")


def vio(out, names):
    print("| level | features, mean (least) | error at the end | largest error | path |")
    print("|---|---|---|---|---|")
    for name in names:
        rows = list(csv.DictReader(open(f"{out}/{name}_vio/replay_smoke.csv")))[1:]  # the first is before the start
        estimate = np.array([[float(row[key]) for key in ("px", "py", "pz")] for row in rows])
        truth = np.array([[float(row[key]) for key in ("tx", "ty", "tz")] for row in rows])
        error = np.linalg.norm(estimate - truth, axis=1)
        tracked = np.array([float(row["tracked"]) for row in rows])
        path = np.sum(np.linalg.norm(np.diff(truth, axis=0), axis=1))
        print(f"| {name} | {tracked.mean():.0f} ({tracked.min():.0f}) | {error[-1]:.3f} m | {error.max():.3f} m | "
              f"{path:.1f} m |")


def reach(out, names):
    """The light's reach by height: the whole frame, not the wall's region."""
    print("| level | height m | raw grey mean / p95 | under 6 levels | matched share | matched depth p50 m (true) "
          "| corners | tracked (least) |")
    print("|---|---|---|---|---|---|---|---|")
    for name in names:
        data, meta = load(out, name)
        rows = []
        for index in range(min(len(data["left"]), len(data["right"]))):
            left = cv2.cvtColor(data["left"][index], cv2.COLOR_RGB2GRAY)
            right = cv2.cvtColor(data["right"][index], cv2.COLOR_RGB2GRAY)
            gained_left, gained_right = gained(left, right)
            matched, depth = stereo(gained_left, gained_right)
            corners = cv2.goodFeaturesToTrack(gained_left, 400, 0.01, 20)
            true_depth = data["depth"][index]
            rows.append([left.mean(), np.percentile(left, 95), np.mean(left < 6), matched.mean(),
                         np.nanmedian(depth[matched]) if matched.any() else np.nan,
                         np.nanmedian(true_depth[matched]) if matched.any() else np.nan,
                         0 if corners is None else len(corners)])
        camera = np.nanmean(np.array(rows, dtype=float), axis=0)
        tracked = "-"
        replay = f"{out}/{name}_vio/replay_reach.csv"
        try:
            tracks = np.array([float(row["tracked"]) for row in list(csv.DictReader(open(replay)))[1:]])
            tracked = f"{tracks.mean():.0f} ({tracks.min():.0f})"
        except (FileNotFoundError, ValueError):
            pass
        print(f"| {name} | {float(meta.get('height', name.lstrip('h') or 1.5)):g} | {camera[0]:.1f} / {camera[1]:.0f} | {camera[2]:.2f} | "
              f"{camera[3]:.3f} | {camera[4]:.2f} ({camera[5]:.2f}) | {camera[6]:.0f} | {tracked} |")


def thermal(out, names):
    """Thermal stereo on the rig's pair of thermal cameras (roadmap item 18 stage 3): the pair's 16-bit frames
    (0.01 K a level) stretched over 280 to 330 K to 8 bits, matched as the visible pair is, against the
    wall's true distance from the thermal cameras (the wall's face less their 0.32 m)."""
    import cv2
    print("| level | sigma 1/m | wall m | matched share | matched within 0.25 m | depth p50 m | thermal spread K |")
    print("|---|---|---|---|---|---|---|")
    focal = 320.0 / np.tan(1.658 / 2.0)
    for name in names:
        data, meta = load(out, name)
        sigma = float(meta.get("sigma", 0.0)); wall = float(meta.get("wall", 8.0)) - 0.32
        matched, within, depths, spreads = [], [], [], []
        to8 = lambda a: np.clip((0.01 * a.astype(np.float64) - 280.0) / 50.0 * 255.0, 0, 255).astype(np.uint8)
        for left16, right16 in zip(data["thermal"], data["thermal_right"]):
            kelvin = 0.01 * left16.astype(np.float64)
            spreads.append(float(np.percentile(kelvin, 95) - np.percentile(kelvin, 5)))
            matcher = cv2.StereoSGBM_create(0, 64, 5, 8 * 25, 32 * 25, 1, 31, 10, 100, 2, cv2.STEREO_SGBM_MODE_SGBM_3WAY)
            disparity = matcher.compute(to8(left16), to8(right16)).astype(np.float64) / 16.0
            valid = disparity[:, 64:] > 0
            matched.append(valid.mean())
            depth = focal * 0.20 / np.where(valid, disparity[:, 64:], np.nan)
            depths.append(np.nanmedian(depth) if valid.any() else float("nan"))
            within.append(float(np.mean(np.abs(depth[valid] - wall) <= 0.25)) if valid.any() else float("nan"))
        print(f"| {name} | {sigma:g} | {wall:.2f} | {np.mean(matched):.3f} | {np.nanmean(within):.2f} | "
              f"{np.nanmedian(depths):.2f} | {np.mean(spreads):.2f} |")


def main():
    arguments = sys.argv[1:]
    if arguments and arguments[0] == "--thermal":
        thermal(arguments[1], arguments[2:])
        return 0
    if arguments and arguments[0] == "--vio":
        vio(arguments[1], arguments[2:])
        return 0
    if arguments and arguments[0] == "--reach":
        reach(arguments[1], arguments[2:])
        return 0
    if arguments and arguments[0] == "--noise":
        noise(arguments[1], arguments[2:])
        return 0
    if arguments and arguments[0] == "--calibrate":
        calibrate(arguments[1], arguments[2:])
        return 0
    baseline = "none"
    if "--baseline" in arguments:
        at = arguments.index("--baseline")
        baseline = arguments[at + 1]
        del arguments[at:at + 2]
    report(arguments[0], arguments[1:], baseline)
    return 0


if __name__ == "__main__":
    sys.exit(main())
