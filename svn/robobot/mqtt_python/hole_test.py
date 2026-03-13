#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import time as t
from scam import cam
from uservice import service
from sservo import servo # Assicurati che sservo sia presente

# --- CALIBRATION DATA ---
mtx = np.array([[953.422, 0.000, 614.889],
                [0.000, 952.264, 562.618],
                [0.000, 0.000, 1.000]])
dist = np.array([-0.3859649, 0.1904913, -0.0001064, -0.0004107, -0.05403132])

# --- PHYSICAL CONSTANTS ---
HOLE_DIAMETER_CM = 5.2   
DIST_APPROACH_CM = 20.0  
DIST_STOP_CM = 12.0 # Ridotto a 12cm per essere pronti al saltino    
CENTER_TOLERANCE = 15    

# --- CONTROL PARAMETERS ---
K_P_TURN = 0.0020        
FAST_SPEED = 0.20        
SLOW_SPEED = 0.10        
SEARCH_TURN = 0.30

def find_hole_strictly(frame):
    """
    Directly searches for the hole using Circularity and Aspect Ratio.
    """
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    blurred = cv.GaussianBlur(gray, (7, 7), 0)
    edged = cv.Canny(blurred, 50, 150)
    
    kernel = np.ones((3,3), np.uint8)
    edged = cv.dilate(edged, kernel, iterations=1)
    contours, _ = cv.findContours(edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    best_candidate = None
    for cnt in contours:
        area = cv.contourArea(cnt)
        if area < 400: continue
        peri = cv.arcLength(cnt, True)
        if peri == 0: continue
        
        circularity = 4 * np.pi * (area / (peri * peri))
        x, y, w, h = cv.boundingRect(cnt)
        aspect_ratio = float(w)/h
        
        if circularity > 0.75 and 0.8 < aspect_ratio < 1.2:
            diameter_px = (w + h) / 2
            center_x = x + (w / 2)
            best_candidate = (center_x, y + (h / 2), diameter_px)
            break 
    return best_candidate

def loop():
    focal_length = mtx[0, 0]
    optical_center_x = mtx[0, 2]

    print("% mission-hole: Starting direct search...")
    # --- RAISE SERVO AT START ---
    print("% Raising servo...")
    servo.servo_change_position(-900) # Valore tipico per alzare, regola se necessario
    t.sleep(1.0)

    while not service.stop:
        ok, img, _ = cam.getImage()
        if not ok: continue

        frame = cv.undistort(img, mtx, dist)
        target = find_hole_strictly(frame)
        
        if target:
            tx, ty, dia_px = target
            current_dist_cm = (HOLE_DIAMETER_CM * focal_length) / dia_px
            error_x = tx - optical_center_x
            
            if current_dist_cm > DIST_APPROACH_CM:
                # Phase 1: Fast Approach
                turn = -error_x * K_P_TURN
                service.send("robobot/cmd/ti", f"rc {FAST_SPEED} {turn:.2f}")
                service.send("robobot/cmd/T0", "leds 16 0 30 0") # GREEN
            elif current_dist_cm > DIST_STOP_CM:
                # Phase 2: Centering
                if abs(error_x) > CENTER_TOLERANCE:
                    turn = -error_x * (K_P_TURN * 1.5)
                    service.send("robobot/cmd/ti", f"rc 0.00 {turn:.2f}")
                    service.send("robobot/cmd/T0", "leds 16 30 30 0") # YELLOW
                else:
                    service.send("robobot/cmd/ti", f"rc {SLOW_SPEED} 0.00")
                    service.send("robobot/cmd/T0", "leds 16 0 0 30") # BLUE
            else:
                # Phase 3: THE JUMP
                print("% Target reached! Performing the jump...")
                service.send("robobot/cmd/T0", "leds 16 30 0 30") # PURPLE
                
                # --- SMALL JUMP FORWARD ---
                service.send("robobot/cmd/ti", "rc 0.30 0.00") # Scatto a 0.3m/s
                t.sleep(0.5) # Durata del saltino (circa 15cm)
                
                service.send("robobot/cmd/ti", "rc 0.00 0.00")
                print("% Mission Complete.")
                break
        else:
            # Search mode
            service.send("robobot/cmd/ti", f"rc 0.00 {SEARCH_TURN}")
            service.send("robobot/cmd/T0", "leds 16 0 30 30") # CYAN

        t.sleep(0.05)

if __name__ == "__main__":
    service.setup("localhost")
    if service.connected:
        try: loop()
        except KeyboardInterrupt: pass
    
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    service.terminate()