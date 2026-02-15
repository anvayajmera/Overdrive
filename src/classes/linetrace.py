import math
from itertools import combinations

import cv2
import numpy as np
from cv2 import ximgproc

from classes.constants import BASE_SPEED_PIXEL, CROSSTRACK_GAIN, GUI
from classes.Robot import Robot


# =========================
# Data Structures
# =========================
class Node:
    def __init__(self, point):
        self.point = point
        self.connected = []


class Edge:
    def __init__(self, n1, n2, weight):
        self.n1 = n1
        self.n2 = n2
        self.weight = weight


# =========================
# Union-Find (Disjoint Set)
# =========================
def find_root(parent, i):
    if parent[i] != i:
        parent[i] = find_root(parent, parent[i])
    return parent[i]


def union_sets(parent, rank, x, y):
    rx, ry = find_root(parent, x), find_root(parent, y)
    if rx != ry:
        if rank[rx] < rank[ry]:
            parent[rx] = ry
        elif rank[rx] > rank[ry]:
            parent[ry] = rx
        else:
            parent[ry] = rx
            rank[rx] += 1


# =========================
# Geometry Helpers
# =========================
def calculate_angle(b1, b2, b3):
    v1 = b1 - b2
    v2 = b3 - b2
    dot = np.dot(v1, v2)
    norm = np.linalg.norm(v1) * np.linalg.norm(v2)
    if norm == 0:
        return 0
    return np.degrees(np.arccos(np.clip(dot / norm, -1, 1)))


# =========================
# Build MST
# =========================
def build_mst(points, max_dist=100):
    n_points = len(points)
    nodes = [Node(p) for p in points]
    edges = []

    # Vectorized distance calculation for efficiency
    pts = np.array(points)
    for i in range(n_points):
        dists = np.linalg.norm(pts[i + 1 :] - pts[i], axis=1)
        for j_idx, d in enumerate(dists):
            j = i + 1 + j_idx
            if d <= max_dist:
                edges.append(Edge(i, j, d))

    edges.sort(key=lambda e: e.weight)
    parent = list(range(n_points))
    rank = [0] * n_points

    for e in edges:
        if find_root(parent, e.n1) != find_root(parent, e.n2):
            nodes[e.n1].connected.append(e.n2)
            nodes[e.n2].connected.append(e.n1)
            union_sets(parent, rank, e.n1, e.n2)

    return nodes


# =========================
# Key Point Extraction
# =========================
def extract_skeleton_points(skeleton, step=15):
    """
    Fast extraction of points from the skeleton using stride-based sampling.
    """
    coords = np.column_stack(np.where(skeleton > 0))
    if len(coords) == 0:
        return []

    # Fast sampling: use every N-th point from the skeleton coordinates
    # This is O(N) and much faster than distance-based filtering in Python
    sampled = coords[::step]
    return [np.array([p[1], p[0]]) for p in sampled]


# =========================
# Draw Graph Helpers
# =========================
def draw_graph_on_frame(frame, graph):
    overlay = frame.copy()
    seen = set()
    if not graph:
        return overlay

    max_conn = max(len(n.connected) for n in graph)
    for i, node in enumerate(graph):
        color = (255, 0, 0) if len(node.connected) >= 3 else (0, 0, 255)
        cv2.circle(overlay, tuple(node.point), 4, color, -1)
        for j in node.connected:
            key = tuple(sorted((i, j)))
            if key not in seen:
                cv2.line(
                    overlay, tuple(node.point), tuple(graph[j].point), (0, 255, 0), 2
                )
                seen.add(key)
    return overlay


def init():
    if GUI:
        cv2.namedWindow("Original", cv2.WINDOW_NORMAL)


# =========================
# Image Processing
# =========================
def process_image(img):
    blur = cv2.blur(img, (3, 3))
    f = blur.astype(np.float32) + 1
    log_img = cv2.log(f)
    log_img = cv2.normalize(log_img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)  # type: ignore

    # Faster thresholding
    b, g, r = cv2.split(log_img)
    _, b_bin = cv2.threshold(b, 200, 255, cv2.THRESH_BINARY_INV)
    _, g_bin = cv2.threshold(g, 200, 255, cv2.THRESH_BINARY_INV)
    _, r_bin = cv2.threshold(r, 200, 255, cv2.THRESH_BINARY_INV)

    binary = cv2.bitwise_and(cv2.bitwise_and(b_bin, g_bin), r_bin)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    closing = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return closing

    largest = max(contours, key=cv2.contourArea)
    mask = np.zeros_like(closing)
    cv2.drawContours(mask, [largest], -1, 255, cv2.FILLED)  # type: ignore

    # Remove edge artifacts by blacking out a 15px border
    # This prevents the skeleton from branching or flattening at the frame edges
    h, w = mask.shape
    cv2.rectangle(mask, (0, 0), (w - 1, h - 1), 0, 30)

    return cv2.bitwise_and(closing, mask)


# =========================
# Bezier Curve
# =========================
def bezier_curve(control_points, num_points=100):
    if len(control_points) < 2:
        return np.zeros((0, 2), dtype=np.float32)

    cp = np.asarray(control_points, dtype=np.float32)
    n = cp.shape[0] - 1

    t = np.linspace(0.0, 1.0, num_points, dtype=np.float32)
    curve = np.zeros((num_points, 2), dtype=np.float32)

    for i in range(n + 1):
        coef = float(math.comb(n, i))
        b = (coef * (1.0 - t) ** (n - i) * (t**i)).astype(np.float32)  # type: ignore
        curve += b[:, None] * cp[i]

    return np.round(curve).astype(np.int32)


# =========================
# Stanley Controller Overlay
# =========================
def draw_stanley_overlay(frame, path_points):
    h, w, _ = frame.shape
    if len(path_points) < 2:
        return frame, 0, 0

    # Nearest point (fixed index for simplicity in this version)
    idx = min(len(path_points) - 1, 40)
    target = path_points[idx]

    # Draw path
    for p in path_points:
        cv2.circle(frame, tuple(p.astype(int)), 2, (0, 255, 0), -1)

    cv2.circle(frame, tuple(target.astype(int)), 6, (0, 255, 255), -1)

    # Tangent calc: points from current point toward the next point in the sequence
    if idx < len(path_points) - 1:
        tangent = path_points[idx + 1] - target
    else:
        # Use previous point if we're at the end
        tangent = target - path_points[idx - 1]

    tangent = tangent / (np.linalg.norm(tangent) + 1e-6)

    alpha = math.atan2(tangent[0], -tangent[1])
    offset = target[0] - (w // 2)

    # Visualization of tangent line (Arrowhead)
    arrow_len = 40
    p2 = target + (tangent * arrow_len).astype(int)
    cv2.arrowedLine(
        frame,
        tuple(target.astype(int)),
        tuple(p2.astype(int)),
        (255, 0, 255),
        3,
        tipLength=0.3,
    )

    # Visualization
    cv2.line(frame, (w // 2, target[1]), tuple(target), (0, 0, 255), 2)
    cv2.putText(
        frame,
        f"Off: {offset}",
        (w // 2 - 40, target[1] - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 0, 255),
        2,
    )

    return frame, alpha, offset


# =========================
# Main Linetrace Logic
# =========================
def linetrace():
    r = Robot()

    ret, frame = r.line_cam.read()
    if not ret:
        return

    frame = cv2.flip(frame, -1)

    og_frame = frame.copy()

    binary_path = process_image(frame)
    thinned = cv2.ximgproc.thinning(binary_path)

    # Fast point extraction from skeleton
    key_points = extract_skeleton_points(thinned)

    nodes = []
    path_points = []
    if len(key_points) >= 2:
        nodes = build_mst(key_points)

        # Path Traversal: Start from bottom-center and traverse the main spine.
        # This is more robust than Y-sorting when branches or artifacts exist.
        start_idx = 0
        min_dist = float("inf")
        for i, n in enumerate(nodes):
            d = np.linalg.norm(n.point - np.array([480, 540]))
            if d < min_dist:
                min_dist = d
                start_idx = i

        # Find the longest path in the MST starting from the robot's position.
        # This identifies the "main spine" and ignores small side branches or artifacts.
        def get_longest_branch(curr_idx, visited):
            visited.add(curr_idx)
            best_path = [nodes[curr_idx].point]

            for neighbor in nodes[curr_idx].connected:
                if neighbor not in visited:
                    # Explore this branch
                    branch_path = get_longest_branch(neighbor, visited.copy())
                    # Keep the branch that has the most points (longest skeleton distance)
                    if len(branch_path) + 1 > len(best_path):
                        best_path = [nodes[curr_idx].point] + branch_path
            return best_path

        path_points = get_longest_branch(start_idx, set())

        # Ensure path ordering: Always start from bottom (High Y) to top (Low Y)
        # start_idx is already nearest the robot, but we double check the endpoints
        if len(path_points) > 1:
            if path_points[0][1] < path_points[-1][1]:
                path_points = path_points[::-1]

    if len(path_points) > 5:
        bez = bezier_curve(path_points)
        frame, theta, offset_error = draw_stanley_overlay(frame, bez)

        output = math.degrees(
            theta + math.atan2(CROSSTRACK_GAIN * offset_error, BASE_SPEED_PIXEL)
        )
        r.set_motor_output(round(output))

        if GUI:
            cv2.putText(
                frame,
                f"Angle: {round(math.degrees(theta), 1)}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                frame,
                f"Output: {round(output, 1)}",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

    if GUI:
        if nodes:
            node_frame = draw_graph_on_frame(og_frame, nodes)
            cv2.imshow("Nodes", node_frame)
        cv2.imshow("Original", frame)
        cv2.imshow("Thinning", thinned)
