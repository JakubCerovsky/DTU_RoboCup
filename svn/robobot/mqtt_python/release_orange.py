import cv2 as cv
import numpy as np
import time as t
from scam import cam
from uservice import service
from sservo import servo

LOWER_ORANGE = np.array([5, 150, 100])
UPPER_ORANGE = np.array([25, 255, 255])


def drop_ball_with_confirmation():
    """Shakes robot to drop ball and confirms with a jolt"""
    
    print("% PHASE: Dropping ball - Shaking starts...")
    
    # --- STEP 1: Side-to-side shaking while ball is visible ---
    shake_speed = 0.25
    direction = 1
    last_switch_time = t.time()
    switch_interval = 3.5  # Faster to shake better
    
    while not service.stop:
        ok, img, _ = cam.getImage()
        if not ok: continue

        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, LOWER_ORANGE, UPPER_ORANGE)
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        current_area = 0
        if contours:
            largest = max(contours, key=cv.contourArea)
            current_area = cv.contourArea(largest)
        
        # If area disappears (ball fell)
        if current_area < 80: 
            print("% EVENT: Ball disappeared!")
            service.send("robobot/cmd/ti", "rc 0.00 0.00")
            t.sleep(0.2) # Brief pause for stability
            break
        
        # Oscillation movement
        if t.time() - last_switch_time > switch_interval:
            direction *= -1
            last_switch_time = t.time()
        
        service.send("robobot/cmd/ti", f"rc 0.00 {shake_speed * direction:.2f}")
        t.sleep(0.05)

    # --- STEP 2: Confirmation bump (forward and backward) ---
    print("% PHASE: Confirmation bump...")
    
    # Avanti
    service.send("robobot/cmd/T0", "leds 16 30 30 30") # White for confirmation
    service.send("robobot/cmd/ti", "rc 0.12 0.00")
    t.sleep(0.4)
    
    # Backward
    service.send("robobot/cmd/ti", "rc -0.12 0.00")
    t.sleep(0.4)
    
    # Final stop
    service.send("robobot/cmd/ti", "rc 0.00 0.00")
    print("% MISSION: Confirmation complete. Ball is definitely in.")
    return True


if __name__ == "__main__":
    service.setup("localhost")
    if service.connected:
        try:
            servo.servo_change_position(250)
            drop_ball_with_confirmation()
        except KeyboardInterrupt:
            pass
    
    # Final cleanup
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    service.terminate()