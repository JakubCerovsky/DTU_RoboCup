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
MARKER_SIZE = 0.04      # 4cm in meters

# --- MOVEMENT SETTINGS ---
TARGET_DIST = 0.20      # Stop point: 20cm
FAST_APPROACH = 0.30    # High speed to catch the moving shuttle
SLOW_APPROACH = 0.12    # Precision crawl speed
K_P_TURN = 0.004        # Steering gain for lateral tracking

def loop():
    print(f"% START: Waiting for ArUco ID {MARKER_ID} to appear...")
    
    try:
        mtx = np.loadtxt("calib_mtx.txt")
        dist = np.loadtxt("calib_dist.txt")
    except:
        print("! Warning: Calibration files missing. Distance will be estimated.")
        mtx, dist = None, None

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
                    
                    # 1. Lateral Error Calculation
                    c = corners[i][0]
                    marker_center_x = int((c[0][0] + c[2][0]) / 2)
                    img_center_x = img.shape[1] // 2
                    error_x = marker_center_x - img_center_x
                    
                    # 2. Distance Calculation (PnP)
                    current_dist = 999.0
                    if mtx is not None:
                        rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners[i], MARKER_SIZE, mtx, dist)
                        current_dist = tvec[0][0][2] 

                    # 3. Control Logic
                    if current_dist <= TARGET_DIST:
                        # TARGET REACHED
                        service.send("robobot/cmd/ti", "rc 0.00 0.00")
                        print(f"Target reached at {current_dist:.2f}m. Stopping.")
                        return 

                    else:
                        # Approach logic: Fast approach until 45cm, then slow down
                        speed = FAST_APPROACH if current_dist > 0.45 else SLOW_APPROACH
                        turn = -error_x * K_P_TURN
                        service.send("robobot/cmd/ti", f"rc {speed:.2f} {turn:.2f}")
                    
                    break # Target found, move to next frame

        if not target_found:
            # Stay still and wait for the marker to pass by
            service.send("robobot/cmd/ti", "rc 0.00 0.00")

        t.sleep(0.03)

if __name__ == "__main__":
    service.setup("localhost")
    if service.connected:
        try:
            loop()
        except KeyboardInterrupt:
            pass
    
    # Safety shutdown
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    service.terminate()