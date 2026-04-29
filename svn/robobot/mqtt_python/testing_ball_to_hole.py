#!/usr/bin/env python3

import time as t
import cv2 as cv
import numpy as np

from uservice import service
from motion_helper import turn_with_feedback
from sgpio import gpio
from sedge import edge
from scam import cam
from sservo import servo

# Test sequence tuning
TURN_RIGHT_DEG = -120
TURN_RATE_DEG_S = 45.0
LINE_VALID_MIN = 4
LOST_DEBOUNCE_COUNT = 10
SEARCH_SPEED = 0.18
FOLLOW_SPEED = 0.18

# Orange color range in HSV
LOWER_ORANGE = np.array([5, 150, 100])
UPPER_ORANGE = np.array([25, 255, 255])

# Distance Thresholds (Area in pixels)
AREA_1 = 6400        # Estimated area at 20cm distance
AREA_2 = 10200       # Final target area at 15cm distance
CENTER_TOLERANCE = 20   # Narrower tolerance for the precision phase

# Speed Settings during ball searching
SEARCH_TURN_SPEED = 0.30 # High speed rotation for searching
FAST_APPROACH = 0.10    # Fast speed to reach the 15cm mark
SLOW_APPROACH = 0.050     # Precision speed for the final 5cm
K_P_TURN = 0.0025        # High gain for snappy centering


def set_line_leds(r, g, b):
    for led_id in (14, 15, 16):
        service.send("robobot/cmd/T0", f"leds {led_id} {r} {g} {b}")


def stop_requested():
    if service.stop:
        return True
    if gpio.test_stop_button():
        service.stop = True
        return True
    return False


def wait_with_stop(duration_s, step_s=0.02):
    end_time = t.monotonic() + duration_s
    while t.monotonic() < end_time:
        if stop_requested():
            return False
        t.sleep(min(step_s, end_time - t.monotonic()))
    return True

def rc_for(forward_m_s, turn_rad_s, duration_s, stop_after=False):
    if stop_requested():
        return False

    service.send("robobot/cmd/ti", f"rc {forward_m_s:.3f} {turn_rad_s:.3f}")
    ok = wait_with_stop(duration_s)

    if stop_after or not ok:
        service.send("robobot/cmd/ti", "rc 0.0 0.0")

    return ok


def ball_visible(min_area=20):
    ok, img, _ = cam.getImage()
    if not ok:
        return True  # fail-safe

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, LOWER_ORANGE, UPPER_ORANGE)
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    if not contours:
        return False

    largest = max(contours, key=cv.contourArea)
    return cv.contourArea(largest) >= min_area


def rc_watch(forward_m_s, turn_rad_s, duration_s, check_dt=0.05):
    """
    Run an rc command while repeatedly checking whether the ball is still visible.
    Returns True if hole found (ball disappeared), False otherwise.
    """
    end_time = t.monotonic() + duration_s

    while t.monotonic() < end_time:
        if stop_requested():
            return False

        service.send("robobot/cmd/ti", f"rc {forward_m_s:.3f} {turn_rad_s:.3f}")

        if not ball_visible(min_area=20):
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            print("% line-test: ball disappeared -> hole found!")
            return True

        t.sleep(check_dt)

    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    return False

def locate_ball(contours, img_center_x):
    if contours:
        largest = max(contours, key=cv.contourArea)
        area = cv.contourArea(largest)
        
        if area > 500: # Threshold to ignore noise
            M = cv.moments(largest)
            ball_x = int(M["m10"] / M["m00"])
            error_x = ball_x - img_center_x
            
            # PHASE 1: Fast approach to 15cm
            if area < AREA_1:
                turn = -error_x * K_P_TURN
                service.send("robobot/cmd/ti", f"rc {FAST_APPROACH} {turn:.2f}")
                service.send("robobot/cmd/T0", "leds 16 0 30 0") # GREEN

            # PHASE 2: Precise centering and final 5cm approach
            elif area < AREA_2:
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
                servo.servo_change_position(250)
                service.send("robobot/cmd/T0", "leds 16 0 30 0")
                return True
        else:
            # Rotate to find ball if area too small
            service.send("robobot/cmd/ti", f"rc 0.00 {SEARCH_TURN_SPEED}")
            return False
    else:
        # Active search rotation
        service.send("robobot/cmd/ti", f"rc 0.00 {SEARCH_TURN_SPEED}")
        service.send("robobot/cmd/T0", "leds 16 0 30 30") # CYAN: Searching
        return False


def setup_follow_and_locate():
    """Follow line after roundabout until split, then locate and approach ball - EXACT code from half_way_competition.py"""
    print("% line-test: following line to split detection")
    edge.lineControl(FOLLOW_SPEED, False, 2)
    
    # STATE_FOLLOWING_AFTER_ROUNDABOUT
    while not stop_requested():
        if edge.splitDetected:
            print("% mission-run: second branch/split detected -> locate ball")
            edge.lineControl(0, True)
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            break
        t.sleep(0.05)
    
    if stop_requested():
        print("% line-test: stop requested during line follow")
        return
    
    # STATE_LOCATE_BALL
    while not stop_requested():
        try:
            mtx = np.loadtxt("calib_mtx.txt")
            dist = np.loadtxt("calib_dist.txt")
        except:
            mtx, dist = None, None
            
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
        
        if locate_ball(contours, img_center_x):
            break
            
        t.sleep(0.05)
    
    print("% line-test: ball located, waiting for servo to complete")
    t.sleep(2.0)  # Wait for servo to lower the arm
    print("% line-test: servo complete, ready for test sequence")


def run_test():
    # """Run test sequence - turn, find lines, then detect hole at end."""
    # 0) Turn right 
    if stop_requested():
        print("% line-test: stop requested before turn")
        return

    print(f"% line-test: turn right {abs(TURN_RIGHT_DEG)} deg")
    turn_with_feedback(
        TURN_RIGHT_DEG,
        turn_rate_deg_s=TURN_RATE_DEG_S,
        forward_m_s=0.0,
        stop_after=True,
    )

    if stop_requested():
        print("% line-test: stop requested after turn")
        return

    # 1) Go straight and find first line.
    print("% line-test: searching first line (straight)")
    edge.lineControl(0, True)
    service.send("robobot/cmd/ti", f"rc {SEARCH_SPEED:.2f} 0.00")
    while not stop_requested() and edge.lineValidCnt <= LINE_VALID_MIN:
        t.sleep(0.05)

    if stop_requested():
        return

    # 2) Keep going straight across the first line until it is lost.
    print("% line-test: first line found -> continue straight across")
    # Turn OFF line control and go straight (no feedback)
    edge.lineControl(0, True)
    service.send("robobot/cmd/ti", f"rc {SEARCH_SPEED:.2f} 0.00")
    while not stop_requested() and edge.lineValidCnt > LINE_VALID_MIN:
        # Keep going straight without line following
        service.send("robobot/cmd/ti", f"rc {SEARCH_SPEED:.2f} 0.00")
        t.sleep(0.05)

    if not stop_requested():
        print("% line-test: first line lost")

    if stop_requested():
        return

    # STEP 3: Turn right again until line is found
    print("% line-test: step 3 - turning right until line found again")
    edge.lineControl(0, True)  # Disable line following    
    while not stop_requested() and edge.lineValidCnt <= LINE_VALID_MIN:
        service.send("robobot/cmd/ti", "rc 0.2 -0.6")  # Continue turning right and moving forward
        t.sleep(0.05)
    
    print("% line-test: line found -> step 3 complete")
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    
    if stop_requested():
        return
    
    # STEP 4: Follow line and detect bump on right side
    print("% line-test: step 4 - following line and detecting bump on right side")
    edge.lineControl(0.1, True)  # Enable line following - robot follows automatically
    t.sleep(2)  # Brief pause to stabilize on line before checking for bump
    
    print("% line-test: following line to bump detection point")
    # Bump detection: compare right vs left sensors
    # When bump is detected, right sensors (4-7) will be much higher than left (0-3)
    
    bump_found = False
    
    while not stop_requested() and not bump_found:
        # Right side sensors: indices 4-7, Left side: indices 0-3
        right_avg = sum(edge.edge_n[i] for i in range(4, 8)) / 4
        left_avg = sum(edge.edge_n[i] for i in range(0, 4)) / 4
        
        # Detect bump: right side significantly higher than left (more tape on right)
        if left_avg > 200 and right_avg > left_avg + 50:
            print("% line-test: bump detected on right side of line!")
            bump_found = True
            edge.lineControl(0, True)  # Disable line control completely
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
        
        t.sleep(0.05)
    
    t.sleep(0.05)
    
    print("% line-test: step 4 complete - bump found on right side")
    

    # STEP 5: Sweep a WiFi-logo pattern in front of the robot
    turn_with_feedback(-20, turn_rate_deg_s=20, forward_m_s=0.0, stop_after=True)
    service.send("robobot/cmd/ti", f"rc {SEARCH_SPEED:.2f} 0.0")
    t.sleep(1)
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    
    print("% line-test: step 5 - sweeping WiFi pattern to find hole")

    hole_found = False
    wifi_arcs = [
        (0.45,  0.90),   # small left
        (0.90, -0.90),   # small right (cross through center)
        (0.55,  0.70),   # medium left
        (1.10, -0.70),   # medium right
        (0.70,  0.50),   # large left
        (1.40, -0.50),   # large right
    ]

    if not hole_found:
        for i, (arc_time, turn_rate) in enumerate(wifi_arcs):
            if stop_requested():
                break

            print(f"% line-test: step 5 - wifi arc {i+1}/{len(wifi_arcs)} turn={turn_rate:.2f}")
            if rc_watch(SLOW_APPROACH, turn_rate, arc_time):
                hole_found = True
                break

    service.send("robobot/cmd/ti", "rc 0.0 0.0")

    if hole_found:
        print("% line-test: step 5 complete - hole found")
    else:
        print("% line-test: step 5 complete - pattern finished, hole not found")
    
    print("% line-test: test sequence completed")


if __name__ == "__main__":
    print("% line-test: initializing")
    service.setup("localhost")

    if not service.connected:
        print("% line-test: MQTT not connected")
    else:
        try:
            set_line_leds(0, 0, 80)
            setup_follow_and_locate()  # Follow line after roundabout, detect split, locate and approach ball
            run_test()
        finally:
            print("% line-test: cleaning up and terminating")
            set_line_leds(0, 0, 0)
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            service.terminate()
            print("% line-test: terminated")
