import time, cv2, math  
import numpy as np
from .Robot import Robot
from .constants import GUI, BLACK_THRESHOLD, ANGLE_MULTIPLIER

def draw_fitline(vx, vy, x0, y0, roi_top: int, roi_h: int, thickness=2):
    # Choose two y's in ROI coords: top and bottom of ROI
    y1 = 0
    y2 = roi_h - 1

    # Compute corresponding x's on the line: x = x0 + (y - y0) * vx/vy
    if abs(vy) < 1e-6:
        return ((0, 0), (0, 0))# avoid divide by zero (nearly horizontal in ROI coords)

    x1 = int(x0 + (y1 - y0) * (vx / vy))
    x2 = int(x0 + (y2 - y0) * (vx / vy))

    # Convert ROI coords -> full frame coords by adding roi_top to y
    p1 = (x1, roi_top + y1)
    p2 = (x2, roi_top + y2)
    
    return (p2, p1)

# =========================================================
# BLACK LINE OFFSET + VISUALIZATION
# =========================================================
def get_line_control(frame: np.ndarray, blurred: np.ndarray, prevStart: tuple[int, int], prevEnd: tuple[int, int], roi_top_ratio=0.3, lookahead_ratio=0.35):
    h, w = frame.shape[:2] 
    roi_top = int(h * roi_top_ratio) 
    roi = blurred[roi_top:h, :] # Only fetch bottom of screen

     # Use brightness channel instead of per-channel AND (more robust)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # Binary mask of dark pixels
    mask = cv2.inRange(gray, 0, BLACK_THRESHOLD) # type: ignore
    
    # Clean up noise + fill small gaps
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return 0, 0.0, mask, None, ((0, 0), (0, 0))
    
    # pick largest contour 
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < 200:  # small sanity cutoff
        return 0, 0.0, mask, None, ((0, 0), (0, 0))
    
     # Fit a line to the contour points
    # returns a direction vector (vx,vy) and a point on the line (x0,y0) in ROI coordinates
    vx, vy, x0, y0 = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01).flatten()

    # Lookahead y (in ROI coords)
    roi_h = h - roi_top # Max height of the ROI
    y_la = int(roi_h * lookahead_ratio) # The portion of the ROI that we are looking at to predict

    t = (y_la - y0) / vy # the so called time, that it would take ot reach that point following hte line
    x_la = int(x0 + t * vx) # The corresponding x value for the predicted point

    (p1, p2) = draw_fitline(vx, vy, x0, y0, roi_top, roi_h)
    
    newStart = (0, 0)
    newEnd = (0, 0)
    
    if math.dist(prevStart, p1) < math.dist(prevStart, p2):
        newStart=p1
        newEnd=p2
    else:
        newStart=p2
        newEnd=p1

    # Convert lookahead point back to full-frame coords
    pt = (x_la, roi_top + y_la)

    center_x = w // 2
    error_px = x_la - center_x

    # Angle relative to "straight ahead" (vertical). Positive means tilting right.
    
    angle = 0.0
    
    
    start = np.array(newStart)
    end = np.array(newEnd)
    
    vec = end - start
    
    mid = (0, h)
    
    angle = np.arccos((vec @ mid) / (np.linalg.norm(vec) * np.linalg.norm(mid)))
    
    if newStart[1] > newEnd[1]:
        angle = math.pi - angle
    
    if newStart[0] > newEnd[0]:
        angle *= -1
    
    if GUI:
        cv2.drawContours(frame, [c], -1, (0, 0, 255), 2, offset=(0, roi_top))
        
        cv2.line(frame, (w//2, 0), (w//2, h), (255, 0, 0), 2)    
        cv2.line(frame, newStart, newEnd, (0, 255, 255), 2)
    
        cv2.circle(frame, newStart, 20, (255, 0, 255), -1)
        cv2.circle(frame, newEnd, 20, (0, 120, 255), -1)
        
    
        # if pt is not None:
        #     cv2.circle(frame, pt, 7, (0, 0, 255), -1)
        
        cv2.rectangle(frame, (0, roi_top), (w, h), (0, 255, 0), 2)
        
        cv2.putText(frame, f"offset:{error_px} ang:{math.degrees(angle):.2f}", (20, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,0,255), 4)

    return error_px, angle, mask, pt, (p1, p2)
# =========================================================
# MAIN
# =========================================================
def linetrace():
    r = Robot()
    ret, frame = r.line_cam.read()
    if not ret:
        return

    # Match your original C++ behavior
    frame = cv2.flip(frame, -1)
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    
    h, w = frame.shape[:2]
    
    error, angle, black_mask, pt, (p1, p2) = get_line_control(frame, blurred, r.lt_start, r.lt_end)
    
    r.lt_start = p1
    r.lt_end = p2
    
    # top, left, right = count_black_pixels(frame, blurred)
    
    # Combine error + angle (feed-forward helps corners a LOT)
    steer = int(error + ANGLE_MULTIPLIER * angle)   

    if GUI:
        cv2.putText(frame, f"Control: {steer}", (20, h - 80), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,0,255), 4)
        cv2.imshow("Camera + Overlays", frame)

    r.set_motor_output(steer)
    
    return steer


# if __name__ == "__main__":
#     linetrace()


def init():
    if GUI:
        cv2.namedWindow("Camera + Overlays", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("Black Mask", cv2.WINDOW_NORMAL)
        
        cv2.resizeWindow("Camera + Overlays", 960, 540)
        # cv2.resizeWindow("Black Mask", 960, 540)
