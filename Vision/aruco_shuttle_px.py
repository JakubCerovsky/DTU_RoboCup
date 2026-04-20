#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import time as t
from scam import cam
from uservice import service

# --- ARUCO CONFIGURATION ---
ARUCO_DICT = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv.aruco.DetectorParameters()
MARKER_ID = 5           

# --- PIXEL-BASED SETTINGS ---
# Adjust these values based on your tests
TARGET_PIXEL_WIDTH = 120  # Equivalent width in pixels at 20cm distance
FAST_ZONE_WIDTH = 80      # If width < 80px (further away), go fast
K_P_TURN = 0.004          # Steering gain

# --- MOVEMENT SPEEDS ---
FAST_APPROACH = 0.30    
SLOW_APPROACH = 0.12    

def loop():
    print(f"% START (Pixel Mode): Waiting for ArUco ID {MARKER_ID}...")
    
    while not service.stop:
        ok, img, imgTime = cam.getImage()
        if not ok:
            t.sleep(0.01)
            continue

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        corners, ids, _ = cv.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)

        target_found = False

        if ids is not None:
            for i in range(len(ids)):
                if ids[i][0] == MARKER_ID:
                    target_found = True
                    
                    # 1. Lateral Error Calculation (Center of marker)
                    c = corners[i][0]
                    marker_center_x = int((c[0][0] + c[2][0]) / 2)
                    img_center_x = img.shape[1] // 2
                    error_x = marker_center_x - img_center_x
                    
                    # 2. Distance Estimation via Pixel Width
                    # We calculate the distance between the top-left and top-right corners
                    current_pixel_width = np.linalg.norm(c[0] - c[1])

                    # 3. Control Logic
                    if current_pixel_width >= TARGET_PIXEL_WIDTH:
                        # TARGET REACHED (The larger the pixel width, the closer the object)
                        service.send("robobot/cmd/ti", "rc 0.00 0.00")
                        print(f"Target reached! Width: {current_pixel_width:.1f}px. Stopping.")
                        return 

                    else:
                        # If the marker is small (far), go fast. If it gets bigger, slow down.
                        speed = FAST_APPROACH if current_pixel_width < FAST_ZONE_WIDTH else SLOW_APPROACH
                        turn = -error_x * K_P_TURN
                        service.send("robobot/cmd/ti", f"rc {speed:.2f} {turn:.2f}")
                    
                    break 

        if not target_found:
            service.send("robobot/cmd/ti", "rc 0.00 0.00")

        t.sleep(0.03)

if __name__ == "__main__":
    service.setup("localhost")
    if service.connected:
        try:
            loop()
        except KeyboardInterrupt:
            pass
    
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    service.terminate()