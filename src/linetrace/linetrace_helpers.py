import math

import cv2
import numpy as np

from constants import GUI


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

    # Lookahead point based on physical distance from start
    lookahead_dist = 120
    idx = 0
    p0 = path_points[0]
    for i, p in enumerate(path_points):
        if np.linalg.norm(p - p0) >= lookahead_dist:
            idx = i
            break

    if idx == 0:
        idx = min(len(path_points) - 1, 50)

    target = path_points[idx]

    # Draw path
    for p in path_points:
        cv2.circle(frame, tuple(p.astype(int)), 2, (0, 255, 0), -1)

    # Distinctly draw the start point in red
    cv2.circle(frame, tuple(path_points[0].astype(int)), 8, (0, 0, 255), -1)

    cv2.circle(frame, tuple(target.astype(int)), 6, (0, 255, 255), -1)

    # Tangent calc: smooth over 5 points to reduce integer rounding noise
    idx_next = min(len(path_points) - 1, idx + 5)
    idx_prev = max(0, idx - 5)

    if idx_next > idx_prev:
        tangent = path_points[idx_next].astype(float) - path_points[idx_prev].astype(
            float
        )
    else:
        tangent = path_points[-1].astype(float) - path_points[0].astype(float)

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


def extract_path(nodes: list[Node], w: int, h: int):
    # Fast point extraction from skeleton
    path_points = []
    if len(nodes) >= 2:
        # Path Traversal: Start from bottom-center and traverse the main spine.
        # This is more robust than Y-sorting when branches or artifacts exist.
        start_idx = 0
        min_dist = float("inf")
        for i, n in enumerate(nodes):
            d = np.linalg.norm(n.point - np.array([w // 2, h]))
            if d < min_dist:
                min_dist = d
                start_idx = i

        # Find the best path in the MST starting from the robot's position.
        # This identifies the "main spine", ignores side branches, and
        # forces the robot to go straight through cross intersections.
        def get_longest_branch(curr_idx, prev_idx, visited):
            visited.add(curr_idx)
            best_score = 1
            best_path = [nodes[curr_idx].point]

            is_junction = len(nodes[curr_idx].connected) >= 3

            for neighbor in nodes[curr_idx].connected:
                if neighbor not in visited:
                    # Explore this branch
                    branch_score, branch_path = get_longest_branch(
                        neighbor, curr_idx, visited.copy()
                    )

                    bonus = 0.0
                    if is_junction and prev_idx is not None:
                        # Use macro-direction from the robot to avoid pixel aliasing
                        v_in = nodes[curr_idx].point - nodes[start_idx].point

                        # Look further down the branch to avoid pixel-level junction noise
                        lookahead_idx = min(len(branch_path) - 1, 10)
                        v_out = branch_path[lookahead_idx] - nodes[curr_idx].point

                        norm_in = np.linalg.norm(v_in)
                        norm_out = np.linalg.norm(v_out)
                        if norm_in > 1e-6 and norm_out > 1e-6:
                            dot = np.dot(v_in / norm_in, v_out / norm_out)
                            if dot > 0.0:  # Continuous massive bonus for straightness
                                bonus = dot * 100000.0

                    # Keep the branch with the highest score
                    if branch_score + 1 + bonus > best_score:
                        best_score = branch_score + 1 + bonus
                        best_path = [nodes[curr_idx].point] + branch_path
            return best_score, best_path

        start_visited = {start_idx}
        branches = []
        for neighbor in nodes[start_idx].connected:
            branches.append(
                get_longest_branch(neighbor, start_idx, start_visited.copy())
            )

        branches.sort(key=lambda x: x[0], reverse=True)

        if len(branches) >= 2:
            path_points = (
                branches[1][1][::-1] + [nodes[start_idx].point] + branches[0][1]
            )
        elif len(branches) == 1:
            path_points = [nodes[start_idx].point] + branches[0][1]
        else:
            path_points = [nodes[start_idx].point]

    return path_points
