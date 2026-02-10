import cv2
import numpy as np
import math
from cv2 import ximgproc

from classes.Robot import Robot
from classes.constants import GUI, CROSSTRACK_GAIN, BASE_SPEED_PIXEL

def init():
    if GUI:
        cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Original", 960, 540)

def draw_points(img, pts, color=(0, 255, 0), r=4, thickness=-1):
    pts = np.asarray(pts)
    if pts.size == 0:
        return img
    pts_i = np.round(pts).astype(np.int32)

    for (x, y) in pts_i:
        cv2.circle(img, (int(x), int(y)), r, color, thickness)
    return img

# =========================
# Image Processing
# =========================
def process_image(img):
   blur = cv2.blur(img, (3,3))
   f = blur.astype(np.float32) + 1
   log_img = cv2.log(f)
   log_img = cv2.normalize(log_img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8) # type: ignore
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

   kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
   closing = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

   contours, _ = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
   if not contours:
       return closing

   largest = max(contours, key=cv2.contourArea)
   mask = np.zeros_like(closing)
   cv2.drawContours(mask, [largest], -1, 255, cv2.FILLED) # type: ignore
   return cv2.bitwise_and(closing, mask)

# =========================
# Bezier Curve
# =========================
def bezier_curve(control_points, num_points=200):
# sanitize
    pts = []
    for p in control_points:
        if p is None:
            continue
        if isinstance(p, (tuple, list)) and len(p) == 2:
            pts.append((float(p[0]), float(p[1])))
        elif isinstance(p, np.ndarray) and p.shape == (2,):
            pts.append((float(p[0]), float(p[1])))

    if len(pts) < 2:
        return np.zeros((0, 2), dtype=np.float32)

    cp = np.asarray(pts, dtype=np.float32)  # (N,2)
    n = cp.shape[0] - 1

    t = np.linspace(0.0, 1.0, num_points, dtype=np.float32)   # (num,)
    curve = np.zeros((num_points, 2), dtype=np.float32)

    for i in range(n + 1):
        coef = float(math.comb(n, i))  # force float early
        b = (coef * (1.0 - t) ** (n - i) * (t ** i)).astype(np.float32, copy=False)
        curve = curve + b[:, None] * cp[i]   # avoid += which is stricter about casting

    return np.round(curve).astype(np.int32)

# =========================
# Stanley Controller Overlay (USING BEZIER CURVE)
# =========================
def draw_stanley_overlay(frame, path_points, k=1.0):
    h, w, _ = frame.shape
    robot_pos = np.array([w // 2, h - 1], dtype=np.float32)  # bottom-center

    # Draw all path points
    for p in path_points:
        cv2.circle(frame, tuple(p.astype(int)), 2, (0, 255, 0), -1)

    # Nearest path point
    # dists = np.lin
    # alg.norm(path_points - robot_pos, axis=1)
    nearest_idx = 120
    nearest_point = path_points[nearest_idx]


    # Draw nearest point
    cv2.circle(frame, tuple(nearest_point.astype(int)), 5, (0, 255, 255), -1)

    # Compute local tangent at nearest point and flip 180 degrees
    if nearest_idx < len(path_points) - 1:
        tangent = path_points[nearest_idx + 1] - nearest_point
    else:
        tangent = nearest_point - path_points[nearest_idx - 1]

    tangent = -tangent / (np.linalg.norm(tangent) + 1e-6)  # <-- FLIPPED

    # Draw red arrow along path tangent
    length = 50
    end_point = nearest_point + tangent * length
    cv2.arrowedLine(frame, tuple(nearest_point.astype(int)), tuple(end_point.astype(int)), (0, 0, 255), 2)
    
   

    alpha = math.atan2(tangent[0], -tangent[1])
    offset = nearest_point[0] - (w // 2)
    
     # Line to center of screen
    cv2.line(frame, (w//2, nearest_point.astype(int)[1]), tuple(nearest_point.astype(int)), (0, 0, 255), 2)
    centerOfLine = tuple(((nearest_point.astype(int)[0] + w//2)//2 - 20, nearest_point.astype(int)[1] - 10))
    cv2.putText(frame, f"{offset}", centerOfLine, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
    # offset *= -1 if robot_pos[0] > nearest_point[0] else 0

    return frame, alpha, offset
# =========================
# Center of Mass of Black Line
# =========================
def find_line_com(binary_img, bands=27):
    """
    binary_img: single-channel image where line pixels are nonzero (e.g. your process_image output)
    returns: (cx, cy) in image coords, or None if no pixels
    """
    H, W = binary_img.shape

    pts = []
    band_h = max(1, H // bands)
    
    prev_com_x = 0
    
    y_step = (W // (bands*2))


    for bi in range(bands, -1, -1):
        y1 = bi * band_h
        y2 = H if bi == bands - 1 else (bi + 1) * band_h
        
        band = binary_img[y1:y2, :]
        
        m = cv2.moments(band, binaryImage=True)
        black_pixels = cv2.countNonZero(band)
        
        if black_pixels > 9000 and cv2.countNonZero(binary_img[0:band_h, :]) < 500:
            # fallback on vertical slices
            left = False
            if cv2.countNonZero(binary_img[:, W-y_step:W]) == 0:
                left = True
            if not left and cv2.countNonZero(binary_img[:, 0:y_step]) > 0:
                if prev_com_x > (W // 2): 
                    left = True
            
            ybands = (W - prev_com_x) // y_step
            if left: 
                ybands = prev_com_x // y_step
                prev_com_x = 0
            # print(f"Prev com x: {prev_com_x}")
            for vi in range(ybands):
                if left:
                    vi = ybands-vi-1
                x1 = vi * y_step + prev_com_x
                x2 = W if vi == ybands - 1 else (vi + 1) * y_step + prev_com_x
                if left:
                    x2 = 0 if vi == ybands - 1 else (vi + 1) * y_step + prev_com_x
                # print(f"Band #{vi}: ({x1}, {x2})")
                
                
                vband = binary_img[:, x1:x2]
                
                m = cv2.moments(vband, binaryImage=True)  
                
                if m["m00"] == 0:
                    continue
                cx = int(m["m10"] / m["m00"])
                cy = int(m["m01"] / m["m00"])   
                
                pts.append((cx+x1, cy))
            # print(pts)
            break
        
        # print(f"Band #{bi}: {black_pixels}")
        
        if m["m00"] == 0:
            continue
        cx = int(m["m10"] / m["m00"])
        cy = int(m["m01"] / m["m00"])
        
        prev_com_x = cx
        
        pts.append((cx, cy+y1))
    pts.reverse()
    return pts
# =========================
# Webcam Loop
# =========================
def linetrace():
    r = Robot()
    
    ret, frame = r.line_cam.read()
    if not ret:
        return
    frame = cv2.flip(frame, -1)   # flip across y-axis (mirror left-right)
    frame = cv2.resize(frame, (960, 540))
    binary_path = process_image(frame)
    # thinned_path = thin_binary_path(binary_path)

    # --- NEW: draw COM on a displayable mask image ---
    com = find_line_com(binary_path)
    com_frame = draw_points(frame.copy(), com)

    if len(com) > 5:
        # Bezier curve
        bez = bezier_curve(com)

        # Ackermann controller overlay on original frame
        frame, theta, offset_error = draw_stanley_overlay(frame, bez)
        
        output = math.degrees(theta + math.atan2(CROSSTRACK_GAIN*offset_error, BASE_SPEED_PIXEL))
        
        r.set_motor_output(round(output))
        
        if GUI:
            cv2.putText(frame, f"Theta: {math.degrees(theta)}", (0, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
            cv2.putText(frame, f"Output: {output}", (0, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2 )

    if GUI:
        cv2.imshow("Original", frame)
        # cv2.imshow("Line COM", com_frame) 