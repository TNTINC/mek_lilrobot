import math
import cv2
import numpy as np
from enum import IntEnum
from dataclasses import dataclass

class Color(IntEnum):
    RED = 1
    YELLOW = 2
    GREEN = 3
    BLUE = 4

@dataclass
class Detection:
    x: float
    y: float
    color: Color

def rgb_to_hue(r, g, b):
    r, g, b = r/255.0, g/255.0, b/255.0
    mx = max(r, g, b)
    mn = min(r, g, b)
    df = mx-mn
    if mx == mn:
        h = 0
    elif mx == r:
        h = (60 * ((g-b)/df) + 360) % 360
    elif mx == g:
        h = (60 * ((b-r)/df) + 120) % 360
    elif mx == b:
        h = (60 * ((r-g)/df) + 240) % 360
    return h

def classify_color(img_bgr, contour) -> Color:
    """Return the color of a given contour by masking it out of the image and computing its mean"""
    mask = np.zeros(img_bgr.shape[:2], dtype="uint8")
    cv2.drawContours(mask, [contour], 0, 255, -1)
    mask = cv2.erode(mask, None, iterations=2)
    mean_color = cv2.mean(img_bgr, mask=mask)
    mean_hue = rgb_to_hue(mean_color[2], mean_color[1], mean_color[0])/2
    if 0 <= mean_hue < 15 or 150 <= mean_hue <= 180:
        return Color.RED
    elif 15 <= mean_hue < 45:
        return Color.YELLOW
    elif 45 <= mean_hue < 90:
        return Color.GREEN
    elif 90 <= mean_hue < 150:
        return Color.BLUE

def process(frame, min_sat=50):
    into_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(into_hsv, np.array([0, min_sat, 0]), np.array([180, 255, 255]))
    cv2.GaussianBlur(mask, (3, 3), 0, mask)
    cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10)),
        mask,
    )
    cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10)),
        mask,
    )

    contours, _ = cv2.findContours(mask, 1, 2)
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 1)
    img_width = frame.shape[1]
    img_height = frame.shape[0]
    color = None
    x_norm = None
    y_norm = None
    try:
        large = (cnt for cnt in contours if cv2.contourArea(cnt) > 100)
        with_moments = ((cnt, cv2.moments(np.float32(cnt))) for cnt in large)
        with_centroids = [
            (cnt, (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])))
            for cnt, M in with_moments
        ]
        with_distance = [
            (cnt, math.dist((img_width / 2, img_height / 2), (cx, cy)))
            for cnt, (cx, cy) in with_centroids
        ]
        most_central = min(with_distance, key=lambda t: t[1])[0]
        chosen = most_central
        M = cv2.moments(chosen)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        #hull = cv2.convexHull(chosen)
        #cv2.drawContours(frame, [hull], 0, (0, 0, 255), 1)
        #cv2.circle(frame, (cx, cy), 3, (255, 0, 0), -1)
        # cv2.putText(det, f"{area}", (cx, cy), 0, 1, (0,0,255))
        #cv2.line(frame, (cx, 0), (cx, img_height - 1), (255, 0, 0), 1)
        #cv2.putText(frame, f"{deviation}", (cx, cy), 0, 0.5, (0, 0, 255))
        color = classify_color(frame, chosen)
        x_norm = (cx - img_width / 2) / (640/2)
        y_norm = (cy - img_height / 2) / (480/2)
    except Exception as e:
        print(e)
    cv2.imshow("Camera", frame)
    cv2.waitKey(1)
    return Detection(x_norm, y_norm, color) # x,y coords are [-1,1] with 0,0 at center of img
