import math
from itertools import combinations

import cv2
import numpy as np
from scipy.interpolate import UnivariateSpline  # unchanged


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
def build_mst(points, max_dist=80):
    nodes = [Node(p) for p in points]
    edges = []

    for i, j in combinations(range(len(points)), 2):
        d = np.linalg.norm(points[i] - points[j])
        if d <= max_dist:
            edges.append(Edge(i, j, d))
        elif d <= 100:
            for k in range(len(points)):
                if k not in (i, j):
                    angle = calculate_angle(points[k], points[i], points[j])
                    if 130 <= angle <= 230:
                        edges.append(Edge(i, j, d))
                        break

    edges.sort(key=lambda e: e.weight)
    parent = list(range(len(points)))
    rank = [0] * len(points)

    for e in edges:
        if find_root(parent, e.n1) != find_root(parent, e.n2):
            nodes[e.n1].connected.append(e.n2)
            nodes[e.n2].connected.append(e.n1)
            union_sets(parent, rank, e.n1, e.n2)

    return nodes


# =========================
# Draw Graph
# =========================
def draw_graph(graph, shape):
    img = np.zeros((*shape, 3), dtype=np.uint8)
    seen = set()
    max_conn = max(len(n.connected) for n in graph)

    for i, node in enumerate(graph):
        color = (255, 0, 0) if len(node.connected) == max_conn else (0, 0, 255)
        cv2.circle(img, tuple(node.point), 2, color, -1)
        for j in node.connected:
            key = tuple(sorted((i, j)))
            if key not in seen:
                cv2.line(img, tuple(node.point), tuple(graph[j].point), (0, 255, 0), 1)
                seen.add(key)
    return img


# =========================
# Image Processing
# =========================
def process_image(img):
    blur = cv2.blur(img, (3, 3))
    f = blur.astype(np.float32) + 1
    log_img = cv2.log(f)
    log_img = cv2.normalize(log_img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    gray = cv2.cvtColor(log_img, cv2.COLOR_BGR2GRAY)
    # Split BGR channels
    b, g, r = cv2.split(log_img)

    # Threshold each channel individually
    _, b_bin = cv2.threshold(b, 200, 255, cv2.THRESH_BINARY_INV)
    _, g_bin = cv2.threshold(g, 200, 255, cv2.THRESH_BINARY_INV)
    _, r_bin = cv2.threshold(r, 200, 255, cv2.THRESH_BINARY_INV)

    # Combine: black only if ALL channels are dark
    binary = cv2.bitwise_and(b_bin, g_bin)
    binary = cv2.bitwise_and(binary, r_bin)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    closing = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return closing

    largest = max(contours, key=cv2.contourArea)
    mask = np.zeros_like(closing)
    cv2.drawContours(mask, [largest], -1, 255, cv2.FILLED)
    return cv2.bitwise_and(closing, mask)


# =========================
# Controlled Thinning
# =========================
def thin_binary_path(bin_img):
    bin_img = (bin_img > 0).astype(np.uint8)
    skeleton = np.zeros_like(bin_img)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))

    while True:
        eroded = cv2.erode(bin_img, element)
        temp = cv2.dilate(eroded, element)
        temp = cv2.subtract(bin_img, temp)
        skeleton = cv2.bitwise_or(skeleton, temp)
        bin_img = eroded.copy()
        if cv2.countNonZero(bin_img) == 0:
            break

    return (skeleton * 255).astype(np.uint8)


# =========================
# Branch Points
# =========================
def find_branch_points(path_img, return_graph=False):
    points = []
    h, w = path_img.shape
    for y in range(1, h - 1):
        for x in range(1, w - 1):
            if path_img[y, x]:
                neighbors = np.sum(path_img[y - 1 : y + 2, x - 1 : x + 2] > 0) - 1
                if neighbors > 3:
                    p = np.array([x, y])
                    if all(np.linalg.norm(p - q) > 20 for q in points):
                        points.append(p)

    if len(points) < 2:
        return None

    graph = build_mst(points)

    if return_graph:
        return graph
    else:
        return draw_graph(graph, path_img.shape)


# =========================
# Draw Graph on Frame
# =========================
def draw_graph_on_frame(frame, graph):
    overlay = frame.copy()
    seen = set()
    max_conn = max(len(n.connected) for n in graph)

    for i, node in enumerate(graph):
        color = (255, 0, 0) if len(node.connected) == max_conn else (0, 0, 255)
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
# NEW: Draw Intersection Points on Original Image (3+ connections only)
# =========================
def draw_intersections_on_frame(frame, graph):
    overlay = frame.copy()
    for node in graph:
        if len(node.connected) >= 3:  # only branch points with 3 or more connections
            cv2.circle(overlay, tuple(node.point), 6, (255, 0, 0), -1)  # blue dot
    return overlay


# =========================
# Parametric Polynomial Fit
# =========================
def fit_parametric_polynomial(points, degree=3):
    d = np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))
    d = np.insert(d, 0, 0)
    t = d / d[-1]

    T = np.vander(t, degree + 1)
    cx = np.linalg.lstsq(T, points[:, 0], rcond=None)[0]
    cy = np.linalg.lstsq(T, points[:, 1], rcond=None)[0]
    return cx, cy


def sample_parametric_curve(cx, cy, num_points=200):
    t = np.linspace(0, 1, num_points)
    T = np.vander(t, len(cx))
    x = T @ cx
    y = T @ cy
    return np.vstack([x, y]).T.astype(np.int32)


# =========================
# Bezier Curve (ADDED)
# =========================
def bezier_curve(control_points, num_points=200):
    control_points = np.asarray(control_points, dtype=np.float32)
    n = len(control_points) - 1
    if n < 2:
        return None

    t = np.linspace(0, 1, num_points)
    curve = np.zeros((num_points, 2), dtype=np.float32)

    for i in range(n + 1):
        curve += (
            math.comb(n, i)
            * ((1 - t) ** (n - i))[:, None]
            * (t**i)[:, None]
            * control_points[i]
        )

    return curve.astype(np.int32)


# =========================
# Webcam Loop
# =========================
def main():
    cap = cv2.VideoCapture(1)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.resize(frame, (500, 500))
        binary_path = process_image(frame)
        thinned_path = thin_binary_path(binary_path)

        skeleton_graph = find_branch_points(thinned_path, return_graph=True)
        if skeleton_graph is not None:
            # Original overlay with MST edges
            overlay = draw_graph_on_frame(frame, skeleton_graph)
            cv2.imshow("Graph", overlay)

            # NEW: Intersection points overlay on original frame
            intersections_overlay = draw_intersections_on_frame(frame, skeleton_graph)
            cv2.imshow("Intersection Points", intersections_overlay)

            raw_points = np.array([n.point for n in skeleton_graph])
            if len(raw_points) > 5:
                # Parametric polynomial
                cx, cy = fit_parametric_polynomial(raw_points)
                curve = sample_parametric_curve(cx, cy)

                curve_img = np.zeros_like(thinned_path)
                for i in range(len(curve) - 1):
                    cv2.line(curve_img, tuple(curve[i]), tuple(curve[i + 1]), 255, 1)
                cv2.imshow("Best Fit Curve", curve_img)

                # Bezier curve
                bez = bezier_curve(raw_points)
                if bez is not None:
                    bez_img = np.zeros_like(thinned_path)
                    for i in range(len(bez) - 1):
                        cv2.line(bez_img, tuple(bez[i]), tuple(bez[i + 1]), 255, 1)
                    cv2.imshow("Bezier Curve", bez_img)

        cv2.imshow("Original", frame)
        cv2.imshow("Binary", binary_path)
        cv2.imshow("Skeleton", thinned_path)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
