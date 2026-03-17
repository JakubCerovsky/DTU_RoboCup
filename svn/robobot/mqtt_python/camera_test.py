#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import time as t
from scam import cam
from uservice import service
from sservo import servo

# Orange color range in HSV
LOWER_ORANGE = np.array([5, 150, 100])
UPPER_ORANGE = np.array([25, 255, 255])

# Distance Thresholds (Area in pixels - adjust based on your camera)
AREA_1 = 6400        # Estimated area at 20cm distance
AREA_2 = 10000       # Final target area at 15cm distance
CENTER_TOLERANCE = 20   # Narrower tolerance for the precision phase

# Speed Settings
SEARCH_TURN_SPEED = 0.30 # High speed rotation for searching
FAST_APPROACH = 0.10    # Fast speed to reach the 15cm mark
SLOW_APPROACH = 0.050     # Precision speed for the final 5cm
K_P_TURN = 0.0025        # High gain for snappy centering

def flash_leds(count=5):
    """Signals mission completion by flashing RED LEDs"""
    print("% MISSION COMPLETE: Flashing LEDs...")
    for _ in range(count):
        service.send("robobot/cmd/T0", "leds 16 30 0 0") # RED
        t.sleep(0.15)
        service.send("robobot/cmd/T0", "leds 16 0 0 0")  # OFF
        t.sleep(0.15)

def loop():
    state = "SEARCHING"
    
    try:
        mtx = np.loadtxt("calib_mtx.txt")
        dist = np.loadtxt("calib_dist.txt")
    except:
        mtx, dist = None, None

    while not service.stop:
        ok, img, imgTime = cam.getImage()
        if not ok:
            t.sleep(0.02)
            continue

        # Undistort for accurate centering
        frame = cv.undistort(img, mtx, dist) if mtx is not None else img
        width = frame.shape[1]
        img_center_x = width // 2
        
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, LOWER_ORANGE, UPPER_ORANGE)
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv.contourArea)
            area = cv.contourArea(largest)
            
            if area > 500: # Threshold to ignore noise
                M = cv.moments(largest)
                ball_x = int(M["m10"] / M["m00"])
                error_x = ball_x - img_center_x
                
                # PHASE 1: Fast approach to 15cm
                if area < AREA_1:
                    state = "FAST_APPROACH"
                    turn = -error_x * K_P_TURN
                    service.send("robobot/cmd/ti", f"rc {FAST_APPROACH} {turn:.2f}")
                    service.send("robobot/cmd/T0", "leds 16 0 30 0") # GREEN

                # PHASE 2: Precise centering and final 5cm approach
                elif area < AREA_2:
                    state = "PRECISION_APPROACH"
                    # Prioritize centering before the final 5cm move
                    if abs(error_x) > CENTER_TOLERANCE:
                        turn = -error_x * K_P_TURN
                        service.send("robobot/cmd/ti", f"rc 0.00 {turn:.2f}")
                        service.send("robobot/cmd/T0", "leds 16 30 30 0") # YELLOW: Centering
                    else:
                        # Once centered, move the last 5cm slowly
                        service.send("robobot/cmd/ti", f"rc {SLOW_APPROACH} 0.00")
                        service.send("robobot/cmd/T0", "leds 16 0 0 30") # BLUE: Final crawl

                # PHASE 3: Target Reached (10cm total)
                else:
                    service.send("robobot/cmd/ti", "rc 0.00 0.00")
                    servo.servo_change_position(210)
                    flash_leds()
                    break
            else:
                # Rotate to find ball if area too small
                service.send("robobot/cmd/ti", f"rc 0.00 {SEARCH_TURN_SPEED}")
        else:
            # Active search rotation
            service.send("robobot/cmd/ti", f"rc 0.00 {SEARCH_TURN_SPEED}")
            service.send("robobot/cmd/T0", "leds 16 0 30 30") # CYAN: Searching

        t.sleep(0.05)

if __name__ == "__main__":
    service.setup("localhost")
    if service.connected:
        try:
            loop()
        except KeyboardInterrupt:
            pass
    
    # Cleanup
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    service.terminate()