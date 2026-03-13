#!/usr/bin/env python3

# 1. Look around for the ball
# 2. Go to 20cm from the ball
# 3. Center respect to the ball and go to 15cm 


#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import time as t
from scam import cam
from uservice import service

# --- CONSTANTS (Keep them global or move inside the function) ---
LOWER_ORANGE = np.array([5, 150, 100])
UPPER_ORANGE = np.array([25, 255, 255])
AREA_1 = 7000 #15 cm        
AREA_2 = 10000 #20 cm     AREA = pi*((d_ball*f)/(2*distance))^2      
CENTER_TOLERANCE = 20   
SEARCH_TURN_SPEED = 0.30 
FAST_APPROACH = 0.22     
SLOW_APPROACH = 0.10     
K_P_TURN = 0.0025        

def flash_leds(count=5):
    """Signals mission completion by flashing RED LEDs"""
    for _ in range(count):
        service.send("robobot/cmd/T0", "leds 16 30 0 0") 
        t.sleep(0.15)
        service.send("robobot/cmd/T0", "leds 16 0 0 0")  
        t.sleep(0.15)

def find_and_catch_Orange():
    """Main mission function to be called from the main loop"""
    servo.servo_change_position(-900)

    # Calibration loading
    try:
        mtx = np.loadtxt("calib_mtx.txt")
        dist = np.loadtxt("calib_dist.txt")
    except:
        mtx, dist = None, None

    # The loop continues until the mission is finished (break) or service stops
    while not service.stop:
        ok, img, imgTime = cam.getImage()
        if not ok:
            t.sleep(0.02)
            continue

        frame = cv.undistort(img, mtx, dist) if mtx is not None else img
        width = frame.shape[1]
        img_center_x = width // 2
        
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, LOWER_ORANGE, UPPER_ORANGE)
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv.contourArea)
            area = cv.contourArea(largest)
            
            if area > 500: 
                M = cv.moments(largest)
                ball_x = int(M["m10"] / M["m00"])
                error_x = ball_x - img_center_x
                
                if area < AREA_1:
                    turn = -error_x * K_P_TURN
                    service.send("robobot/cmd/ti", f"rc {FAST_APPROACH} {turn:.2f}")
                    service.send("robobot/cmd/T0", "leds 16 0 30 0") 
                elif area < AREA_2:
                    if abs(error_x) > CENTER_TOLERANCE:
                        turn = -error_x * K_P_TURN
                        service.send("robobot/cmd/ti", f"rc 0.00 {turn:.2f}")
                    else:
                        service.send("robobot/cmd/ti", f"rc {SLOW_APPROACH} 0.00")
                        service.send("robobot/cmd/T0", "leds 16 0 0 30") 
                else:
                    # MISSION COMPLETE
                    service.send("robobot/cmd/ti", "rc 0.00 0.00")
                    servo.servo_change_position(200)
                    flash_leds()
                    return True # Return success to the main script
            else:
                service.send("robobot/cmd/ti", f"rc 0.00 {SEARCH_TURN_SPEED}")
        else:
            service.send("robobot/cmd/ti", f"rc 0.00 {SEARCH_TURN_SPEED}")
            service.send("robobot/cmd/T0", "leds 16 0 30 30") 

        t.sleep(0.05)
    
    return False # If service.stop was triggered

if __name__ == "__main__":
    # Inizializza la connessione MQTT
    service.setup('localhost')
    
    if service.connected:
        print("% Starting Orange Ball Mission...")
        try:
            find_and_catch_Orange()
        except KeyboardInterrupt:
            print("% Mission interrupted by user")
    else:
        print("% Error: Could not connect to robot service")
    
    # Spegne tutto alla fine
    service.send("robobot/cmd/ti", "rc 0.00 0.00")
    service.terminate()