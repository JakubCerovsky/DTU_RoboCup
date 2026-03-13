#!/usr/bin/env python3

# 1. Try to find the closer complete gate and get closer to the center
# 2. If to close to find the all the gate, find the right column of the gate and positioning at 15cm from it
# 
 
import cv2 as cv
import numpy as np
import time as t
from scam import cam
from uservice import service

# --- SETTINGS ---
# HSV range for yellow gates
LOWER_YELLOW = np.array([20, 100, 100])
UPPER_YELLOW = np.array([35, 255, 255])

# Control parameters
K_P_TURN = 0.0018
CRUISE_SPEED = 0.12
# Lateral safety distance: 15cm converted to pixels (~380px at close range)
SIDE_OFFSET_PX = 380 

def get_gate_error(frame, optical_center_x):
    """
    Analyzes the frame to find the gate using a dual-stage strategy:
    1. Center between two pillars if both are visible.
    2. Maintain a safe offset if only one pillar is visible.
    """
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
    
    # Slight dilation to clean up pillar edges
    mask = cv.dilate(mask, np.ones((3,3), np.uint8), iterations=1)
    
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    pillars = []
    for cnt in contours:
        x, y, w, h = cv.boundingRect(cnt)
        # Filter: must be a vertical object (pillar)
        if h > 80 and h > w:
            pillars.append({'x': x, 'y': y, 'w': w, 'h': h, 'cx': x + w//2})
    
    # Sort pillars from left to right based on X coordinate
    pillars.sort(key=lambda p: p['cx'])

    # --- CASE A: Full Gate Detection (Two pillars visible) ---
    if len(pillars) >= 2:
        p_sx = pillars[0]['cx']
        p_dx = pillars[-1]['cx']
        target_x = (p_sx + p_dx) // 2
        return target_x - optical_center_x, "TWO_PILLARS"

    # --- CASE B: Fallback (Only one pillar visible) ---
    elif len(pillars) == 1:
        p = pillars[0]
        # If the pillar is in the right half, assume it's the RIGHT pillar
        if p['cx'] > optical_center_x:
            # Target: stay 15cm (SIDE_OFFSET_PX) to the LEFT of the pillar
            target_x = p['x'] - SIDE_OFFSET_PX
        else:
            # If it's the LEFT pillar, stay 15cm to the RIGHT of the pillar
            target_x = p['x'] + p['w'] + SIDE_OFFSET_PX
            
        return target_x - optical_center_x, "SINGLE_PILLAR"

    return None, "LOST"

def loop():
    # Calibrated optical center from your matrix
    optical_center_x = 614.9
    blink_state = False
    
    print("% mission-gate: Searching for yellow markers...")

    while not service.stop:
        ok, img, imgTime = cam.getImage()
        if not ok: continue

        frame = img 

        error_x, mode = get_gate_error(frame, optical_center_x)
        
        if mode != "LOST":
            # Target found: calculate turn and drive
            turn = -error_x * K_P_TURN
            turn = max(min(turn, 0.4), -0.4) # Clamp turn speed
            
            service.send("robobot/cmd/ti", f"rc {CRUISE_SPEED} {turn:.2f}")
            service.send("robobot/cmd/T0", "leds 16 30 30 0") # Steady YELLOW when locked
        else:
            # TARGET LOST: Stop motors and blink yellow
            service.send("robobot/cmd/ti", "rc 0.00 0.00")
            
            # Simple blink logic using time
            if int(t.time() * 4) % 2 == 0:
                service.send("robobot/cmd/T0", "leds 16 30 30 0") # Yellow
            else:
                service.send("robobot/cmd/T0", "leds 16 0 0 0") # Off
                
            print("% WARNING: Gate lost! Stopping and blinking...")

        t.sleep(0.05)

if __name__ == "__main__":
    service.setup("localhost")
    if service.connected:
        try: loop()
        except KeyboardInterrupt: pass
    
    # Safety stop on exit
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    service.terminate()