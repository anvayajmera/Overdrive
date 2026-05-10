import math
import time

import cv2
import numpy as np
from cv2 import ximgproc

from classes.Robot import Robot
from constants import (
    GUI,
    LINE_DETECT_SIZE,
    TIMESTEP,
)
from linetrace.linetrace import reset_path
from linetrace.linetrace_helpers import (
    bezier_curve,
    build_mst,
    draw_stanley_overlay,
    extract_path,
    extract_skeleton_points,
)

# ── GUI debug helpers ─────────────────────────────────────────────────────────

_OBS_WINDOW = "Obstacle Debug"
_SCAN_WINDOW = "Line Direction Scan"


def _init_windows():
    if not GUI:
        return
    cv2.namedWindow(_OBS_WINDOW, cv2.WINDOW_NORMAL)
    cv2.namedWindow(_SCAN_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(_OBS_WINDOW, 480, 360)
    cv2.resizeWindow(_SCAN_WINDOW, 480, 360)


def _show_status(r: Robot, phase: str, extras: dict | None = None):
    """
    Draw a live status panel on top of the camera frame and show it in
    the dedicated Obstacle Debug window.
    """
    if not GUI:
        return

    if r.frame is not None and hasattr(r.frame, "shape") and r.frame.size > 0:
        frame = r.frame.copy()
    else:
        frame = np.zeros((360, 640, 3), dtype=np.uint8)

    h, w = frame.shape[:2]

    # Semi-transparent dark banner at the top
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, 200), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    def put(text, y, color=(255, 255, 255)):
        cv2.putText(
            frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA
        )

    # Phase
    put(f"PHASE: {phase}", 30, (0, 200, 255))

    # Obstacle
    obs_col = (0, 60, 255) if r.obs_detected else (0, 220, 0)
    put(f"Obstacle: {'DETECTED' if r.obs_detected else 'clear'}", 60, obs_col)

    # Front sensor readings
    front_str = "  ".join(
        f"[{d:.1f}cm]" if d < 1000 else "[---]" for d in r.front_distances
    )
    put(f"Front: {front_str}", 90, (255, 220, 0))

    # Line size vs threshold
    line_ratio = r.line_size / LINE_DETECT_SIZE if LINE_DETECT_SIZE else 0
    bar_w = int(min(line_ratio, 1.0) * (w - 20))
    bar_col = (0, 200, 0) if line_ratio >= 1.0 else (0, 140, 255)
    cv2.rectangle(frame, (10, 105), (10 + bar_w, 120), bar_col, -1)
    cv2.rectangle(frame, (10, 105), (w - 10, 120), (180, 180, 180), 1)
    put(
        f"Line: {r.line_size:.0f} / {LINE_DETECT_SIZE}  ({line_ratio * 100:.0f}%)",
        145,
        (0, 200, 0) if line_ratio >= 1.0 else (0, 140, 255),
    )

    # Yaw
    put(f"Yaw: {r.yaw:.1f} deg", 170, (200, 200, 255))

    # Extra key-value pairs
    if extras:
        y = 200
        for key, val in extras.items():
            put(f"  {key}: {val}", y, (220, 220, 180))
            y += 25

    cv2.imshow(_OBS_WINDOW, frame)
    cv2.waitKey(1)


def _show_direction_scan(frame_with_path: np.ndarray, alpha_deg: float, direction: str):
    """
    Show the annotated linetrace path on top of the real camera image.
    draw_stanley_overlay already draws the bezier curve, start point,
    lookahead point, and tangent arrow — we just add the angle & direction
    text on top of that.
    """
    if not GUI:
        return

    dbg = frame_with_path.copy()
    h, w = dbg.shape[:2]

    dir_colors = {
        "left": (255, 120, 0),
        "right": (0, 120, 255),
        "straight": (0, 220, 220),
    }
    col = dir_colors.get(direction, (255, 255, 255))

    cv2.putText(
        dbg,
        f"alpha: {alpha_deg:.1f} deg",
        (10, h - 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        dbg,
        f"DIR: {direction.upper()}",
        (10, h - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        col,
        2,
    )

    cv2.imshow(_SCAN_WINDOW, dbg)
    cv2.waitKey(1)


# ── core helpers ──────────────────────────────────────────────────────────────


def _sleep(seconds: float):
    """GUI-aware sleep."""
    if GUI:
        end = time.time() + seconds
        while time.time() < end:
            cv2.waitKey(1)
    else:
        time.sleep(seconds)


def _drive_step(r: Robot):
    """Drive one timestep forward."""
    r.forward()
    if GUI:
        cv2.waitKey(max(1, int(TIMESTEP * 1000)))
    else:
        time.sleep(TIMESTEP)


def _line_visible(r: Robot) -> bool:
    return r.line_size >= LINE_DETECT_SIZE


# ── direction detection ───────────────────────────────────────────────────────

# Dead-band: only classify as a turn when the line is clearly curving.
# Small wiggles or slight misalignment should not trigger the turn bypass.
_DIR_THRESHOLD_DEG = 30.0


def _detect_line_direction(r: Robot) -> str:
    """
    Run the same pipeline as linetrace to build the path, then read the
    Stanley-controller tangent angle (alpha) at a lookahead point.

    alpha > +threshold → line curves RIGHT
    alpha < -threshold → line curves LEFT
    otherwise          → straight

    This is orientation-independent: extract_path always anchors at the
    bottom-centre of the frame (robot position) and traces forward, so it
    does not matter if the robot is slightly misaligned.
    """
    path_points = []
    h, w = 0, 0

    # Try up to 15 frames to find a solid line. Sometimes the camera is dark
    # right after an action or a dropped frame makes it empty.
    for attempt in range(15):
        r.update()

        binary = r.binary_frame
        if binary is None or not hasattr(binary, "shape") or binary.size == 0:
            if GUI:
                cv2.waitKey(max(1, int(TIMESTEP * 1000)))
            else:
                time.sleep(TIMESTEP)
            continue

        if len(binary.shape) == 3:
            binary = binary[:, :, 0]

        h, w = binary.shape

        # ── same pre-processing as linetrace() ───────────────────────────────────
        thinned = ximgproc.thinning(binary)
        cv2.rectangle(thinned, (0, 0), (w - 1, h - 1), 0, 3)  # clear border noise

        key_points = extract_skeleton_points(thinned)
        if len(key_points) < 10:
            if GUI:
                cv2.waitKey(max(1, int(TIMESTEP * 1000)))
            else:
                time.sleep(TIMESTEP)
            continue

        nodes = build_mst(key_points)
        path_points = extract_path(nodes, w, h)

        if len(path_points) >= 5:
            break  # Found a good frame!

        if GUI:
            cv2.waitKey(max(1, int(TIMESTEP * 1000)))
        else:
            time.sleep(TIMESTEP)

    if len(path_points) < 5:
        r.status.log(
            "OBSTACLE",
            "Could not find a valid line in 15 frames – defaulting straight",
            level="WARN",
            force=True,
        )
        if r.frame is not None and hasattr(r.frame, "shape") and r.frame.size > 0:
            _show_direction_scan(r.frame.copy(), 0.0, "straight")
        return "straight"

    # ── orient path from robot (bottom) to ahead (top) ───────────────────────
    # extract_path anchors at bottom-centre, but double-check the flip just
    # like linetrace.py does (without the stale _prev_path_start continuity).
    p0 = np.array(path_points[0], dtype=float)
    p_end = np.array(path_points[-1], dtype=float)
    if p_end[1] > p0[1]:  # last point is lower (closer to robot) → flip
        path_points = path_points[::-1]
        p0 = np.array(path_points[0], dtype=float)
        p_end = np.array(path_points[-1], dtype=float)

    # ── macroscopic direction (start to end) ──────────────────────────────────
    # Instead of a local tangent, look at the overall displacement from the
    # robot to the furthest visible point on the line.
    v = p_end - p0
    macro_alpha = math.atan2(v[0], -v[1])
    alpha_deg = math.degrees(macro_alpha)

    # We still fit the bezier and draw the overlay just so the debug window
    # shows what the robot sees, but we use the macroscopic angle for the decision.
    bez = bezier_curve(path_points)
    if r.frame is not None and hasattr(r.frame, "shape") and r.frame.size > 0:
        vis_frame = r.frame.copy()
    else:
        vis_frame = np.zeros((h, w, 3), dtype=np.uint8)

    if len(bez) >= 2:
        vis_frame, _alpha, offset = draw_stanley_overlay(vis_frame, bez)

    if alpha_deg < -_DIR_THRESHOLD_DEG:
        direction = "left"
    elif alpha_deg > _DIR_THRESHOLD_DEG:
        direction = "right"
    else:
        direction = "straight"

    r.status.log(
        "OBSTACLE",
        f"Path scan: macro_alpha={alpha_deg:.1f}°  → {direction}",
        force=True,
    )

    # Draw a bold line showing the macro vector on the debug overlay
    if GUI:
        cv2.arrowedLine(
            vis_frame,
            tuple(p0.astype(int)),
            tuple(p_end.astype(int)),
            (0, 165, 255),  # Orange macro vector
            3,
            tipLength=0.1,
        )

    _show_direction_scan(vis_frame, alpha_deg, direction)
    return direction


# ── snap bypass (for clearly-turning lines) ──────────────────────────────────

SNAP_ANGLE = 70  # degrees to snap toward the turn side


def _bypass_turn(r: Robot, direction: str):
    """
    Fast bypass used when the line is clearly turning.

    Instead of the full S-curve, the robot just:
      1. Snaps ~SNAP_ANGLE° toward the turn direction to cut around the corner
      2. Drives forward until the line is visible again
      3. Snaps ~SNAP_ANGLE° back to roughly align with the (now-turned) line

    No precise heading restoration is needed because linetrace takes over
    as soon as reset_path() is called and the robot is roughly on the line.
    """
    snap = -SNAP_ANGLE if direction == "left" else SNAP_ANGLE
    merge = SNAP_ANGLE if direction == "left" else -SNAP_ANGLE

    r.status.log(
        "OBSTACLE",
        f"Turn bypass [{direction}] snap={snap}° merge={merge}°",
        force=True,
    )

    r.forward()
    if GUI:
        cv2.waitKey(600)
    else:
        time.sleep(0.6)

    r.stop()

    # ── 1. Snap toward the turn ───────────────────────────────────────────────
    _show_status(
        r, f"TURN BYPASS – SNAP {direction.upper()}", {"snap_angle": f"{snap}°"}
    )
    r.turn(snap)

    # ── 2. Drive until the line is back in view ───────────────────────────────
    BYPASS_MIN_S = 0.3
    BYPASS_MAX_S = 4.0
    t0 = time.time()
    while True:
        r.update()
        elapsed = time.time() - t0
        _show_status(
            r,
            f"TURN BYPASS – DRIVING ({direction})",
            {
                "line_size": f"{r.line_size:.0f}",
                "need": LINE_DETECT_SIZE,
                "elapsed": f"{elapsed:.1f}s",
            },
        )
        if elapsed > BYPASS_MAX_S:
            r.status.log("OBSTACLE", "Turn bypass timeout", level="WARN", force=True)
            break
        if elapsed > BYPASS_MIN_S and _line_visible(r):
            r.status.log("OBSTACLE", "Line found – snapping to merge", force=True)
            break
        _drive_step(r)
    r.stop()

    # # ── 3. Snap back to hook onto the line ───────────────────────────────────
    # _show_status(r, "TURN BYPASS – MERGE SNAP", {"merge_angle": f"{merge}°"})
    # r.turn(merge)

    # Small nudge to settle onto the line
    t0 = time.time()
    while time.time() - t0 < 1.0:
        r.update()
        _show_status(r, "TURN BYPASS – SETTLE", {"line_size": f"{r.line_size:.0f}"})
        if _line_visible(r):
            break
        _drive_step(r)
    r.stop()


# ── S-curve bypass (for straight lines) ──────────────────────────────────────


def _bypass(r: Robot, direction: str):
    """
    S-curve (car-style) bypass in three legs:

      LEFT bypass example
      ───────────────────
      Leg 1  Veer LEFT  75°  – drive diagonally away from obstacle
      Leg 2  Straighten      – brief burst at original heading (parallel)
      Leg 3  Veer RIGHT 75°  – drive diagonally back toward the original path
      Leg 4  Restore heading – turn back to original yaw

    The robot moves forward throughout; it never drives purely sideways.
    """
    veer_out = -75 if direction == "left" else 75
    veer_in = 75 if direction == "left" else -75

    original_yaw = r.yaw
    r.status.log(
        "OBSTACLE",
        f"Car bypass: direction={direction}  original_yaw={original_yaw:.1f}",
        force=True,
    )

    # ── Leg 1: veer out ───────────────────────────────────────────────────────
    _show_status(r, "LEG 1 – VEER OUT", {"dir": direction, "veer_angle": veer_out})
    r.turn(veer_out)

    # Drive diagonally outward for a fixed duration to ensure a massive
    # displacement. We don't check r.obs_detected here because pointing 75
    # degrees away means the sensors instantly point at empty space, causing
    # it to exit instantly without actually displacing far enough.
    VEER_OUT_S = 1.5
    t0 = time.time()
    while time.time() - t0 < VEER_OUT_S:
        r.update()
        _show_status(
            r,
            "LEG 1 – VEER OUT (driving)",
            {"elapsed": f"{time.time() - t0:.1f}s / {VEER_OUT_S}s"},
        )
        _drive_step(r)
    r.stop()
    _sleep(0.1)

    # ── Leg 2: straighten & run parallel ─────────────────────────────────────
    _show_status(r, "LEG 2 – PARALLEL", {"target_yaw": f"{original_yaw:.1f}"})
    r.turnToTarget(original_yaw)

    # Massive parallel run to completely clear the obstacle length
    PARALLEL_S = 3.5
    t0 = time.time()
    while time.time() - t0 < PARALLEL_S:
        r.update()
        _show_status(
            r,
            "LEG 2 – PARALLEL (driving)",
            {"elapsed": f"{time.time() - t0:.1f}s / {PARALLEL_S}s"},
        )
        _drive_step(r)
    r.stop()
    _sleep(0.1)

    # ── Leg 3: veer back toward original path ────────────────────────────────
    _show_status(r, "LEG 3 – VEER IN", {"veer_angle": veer_in})
    r.turn(veer_in)

    # The veer-in needs more time because the robot has to cover the same horizontal
    # distance it gained in Leg 1, and it might be angled slightly differently.
    # We raise the MIN time so it gets a solid chunk of distance before checking,
    # and the MAX time heavily to guarantee it makes it all the way back.
    VEER_IN_MIN_S = 1.0
    VEER_IN_MAX_S = 5.0
    t0 = time.time()
    while True:
        r.update()
        elapsed = time.time() - t0
        _show_status(
            r,
            "LEG 3 – VEER IN (driving)",
            {
                "line_size": f"{r.line_size:.0f}",
                "need": LINE_DETECT_SIZE,
                "elapsed": f"{elapsed:.1f}s",
            },
        )
        if elapsed > VEER_IN_MAX_S:
            r.status.log("OBSTACLE", "Veer-in timeout", level="WARN", force=True)
            break
        if elapsed > VEER_IN_MIN_S and _line_visible(r):
            r.status.log("OBSTACLE", "Line visible – veer-in done", force=True)
            break
        _drive_step(r)
    r.stop()

    # ── Leg 4: restore original heading ──────────────────────────────────────
    _show_status(r, "LEG 4 – RESTORE HEADING", {"target_yaw": f"{original_yaw:.1f}"})
    r.turnToTarget(original_yaw)

    # Brief nudge to make sure we're squarely on the line
    nudge_end = time.time() + 1.0
    while time.time() < nudge_end:
        r.update()
        _show_status(r, "LEG 4 – NUDGE ONTO LINE", {"line_size": f"{r.line_size:.0f}"})
        if _line_visible(r):
            break
        _drive_step(r)
    r.stop()


# ── main entry-point ──────────────────────────────────────────────────────────


def obstacle():
    r = Robot()

    if not r.obs_detected:
        return

    _init_windows()
    r.status.log("OBSTACLE", "Obstacle detected – starting avoidance", force=True)
    r.stop()
    _sleep(0.3)

    # ── Phase 1: push obstacle forward ───────────────────────────────────────
    r.status.log("OBSTACLE", "Pushing obstacle forward…", force=True)
    push_start = time.time()
    PUSH_S = 0.8
    while time.time() - push_start < PUSH_S:
        r.update()
        _show_status(
            r, "PUSHING", {"elapsed": f"{time.time() - push_start:.1f}s / {PUSH_S}s"}
        )
        r.forward()
        if GUI:
            cv2.waitKey(max(1, int(TIMESTEP * 1000)))
        else:
            time.sleep(TIMESTEP)
    r.stop()
    _sleep(0.3)

    # ── Phase 2: move backward ───────────────────────────────────────
    r.status.log("OBSTACLE", "Moving backward…", force=True)
    push_start = time.time()
    PUSH_S = 0.7
    while time.time() - push_start < PUSH_S:
        r.update()
        _show_status(
            r, "PUSHING", {"elapsed": f"{time.time() - push_start:.1f}s / {PUSH_S}s"}
        )
        r.backward()
        if GUI:
            cv2.waitKey(max(1, int(TIMESTEP * 1000)))
        else:
            time.sleep(TIMESTEP)
    r.stop()
    _sleep(0.5)

    # ── Phase 3: scan line direction ──────────────────────────────────────────
    _show_status(r, "SCANNING LINE DIRECTION")
    direction = _detect_line_direction(r)

    r.status.log("OBSTACLE", f"Line direction detected: '{direction}'", force=True)
    _show_status(
        r, f"DIRECTION: {direction.upper()}", {"threshold": f"±{_DIR_THRESHOLD_DEG}°"}
    )
    _sleep(0.8)  # pause so the scan debug frame is readable

    # ── Phase 4: choose bypass strategy ──────────────────────────────────────
    #   Turn detected (|alpha| > threshold) → fast snap bypass toward the turn
    #   Straight / ambiguous              → full S-curve, default left
    if direction in ("left", "right"):
        r.status.log(
            "OBSTACLE", f"Turn bypass on {direction} (snap ±{SNAP_ANGLE}°)", force=True
        )
        _bypass_turn(r, direction)
    else:
        r.status.log("OBSTACLE", "Straight – using S-curve bypass (left)", force=True)
        _bypass(r, "left")

    # ── Phase 4: resume linetrace ─────────────────────────────────────────────
    _show_status(r, "COMPLETE – resuming linetrace")
    r.status.log("OBSTACLE", "Avoidance complete", force=True)
    reset_path()
