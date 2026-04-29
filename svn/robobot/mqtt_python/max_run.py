#!/usr/bin/env python3

import cv2 as cv
import numpy as np
from scam import cam
import threading
import math
import time as t
from uservice import service
from sedge import edge
from spose import pose
from simu import imu
from sgpio import gpio
from srobot import robot
from motion_helper import turn_with_feedback
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

# Line-follow tuning
FOLLOW_SPEED = 0.2
SEARCH_SPEED = 0.2
GO_STRAIGHT_SPEED = 0.2
CLIMB_SPEED = 0.2
REVERSE_SPEED = -0.1
CLIMB_TIMEOUT = 2         # sec safety timeout
LINE_VALID_MIN = 1
LOST_DEBOUNCE_COUNT = 8      # consecutive invalid reads before stop

# Post-flat maneuver tuning
POST_TURN_DEG = -60
POST_CIRCLE_RADIUS_M = 0.32
POST_CIRCLE_SPEED_M_S = 0.12
POST_ROUNDABOUT_DEG = 320
POST_TURN_RATE_DEG_S = 45.0
POST_SEARCH_TURN_DEG = -15
POST_SEARCH_TURN_FORWARD_SPEED = 0.06
POST_SEARCH_STRAIGHT_SPEED = 0.2
POST_SEARCH_STRAIGHT_TIME_S = 3
POST_SEARCH_LEFT_TURN_RAD_S = 0.25

# Flat detection tuning 
RUN_MAX_GYRO_DPS = 20.0
RUN_MAX_TILT_DEG = 2.3
ACC_G_MIN = 0.7
ACC_G_MAX = 1.3

# second rounabout 
SEC_ROUNDABOUT_RADIUS_M = 0.8
SEC_BEFORE_ROUNDABOUT_TURN_DEG = -90
SEC_ROUNDABOUT_TURN_DEG = 90
SEC_CIRCLE_SPEED_M_S = 0.3

# states
STATE_SEARCHING = 0
STATE_FOLLOWING = 10
STATE_CLIMB_TO_FLAT = 20
STATE_ROUNDABOUT = 30
STATE_FIND_LINE_AFTER_ROUNDABOUT = 40
STATE_FOLLOWING_AFTER_ROUNDABOUT = 50
STATE_LOCATE_BALL = 60
STATE_LOCATE_HOLE = 70
STATE_GOING_UNTIL_INTERSECTION_AFTER_BALL = 75
STATE_FOLLOWING_UNTIL_ROUNDABOUT_TWO = 80
STATE_ROUNDABOUT_TWO = 90
STATE_FIND_LINE_AFTER_ROUNDABOUT_TWO = 100
STATE_GO_TO_END = 110
STATE_FINALE_STRETCH = 120


def stop_requested():
    if gpio.test_stop_button():
        service.stop = True
        print("% mission-run: stop button pressed")
        return True
    return service.stop

def driveOneMeter(forwards=True):
  state = 0
  pose.tripBreset()
  print("% Driving 1m -------------------------")
  while not (service.stop):
    if state == 0: # wait for start signal
        if forwards:
            service.send(f"robobot/cmd/ti","rc 0.2 0.0") # (forward m/s, turn-rate rad/sec)
        else:   
            service.send(f"robobot/cmd/ti","rc -0.2 0.0") # (forward m/s, turn-rate rad/sec)
        state = 1
    elif state == 1:
      if abs(pose.tripB) > 1.0 or pose.tripBtimePassed() > 15:
        service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
        state = 2
      pass
    elif state == 2:
        print(f" {pose.velocity():.3f} m/s, {pose.tripB:.3f} m in {pose.tripBtimePassed():.3f} seconds")
        if abs(pose.velocity()) < 0.001:
            state = 99
    else:
      service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
      break;
    t.sleep(0.05)
  pass

def roundabout(roundabout_deg, pre_manouver_turn , radius_m, speed_m_s):
    """After reaching the flat area: turn right 60 deg and drive one left circle."""
    if stop_requested():
        return

    turn_with_feedback(pre_manouver_turn, turn_rate_deg_s=pre_manouver_turn, forward_m_s=0.0, stop_after=True)
    print(f"% roundabout first turn done -> turn with feedback {pre_manouver_turn} deg/s ")
    if stop_requested():
        return

    omega_rad_s = speed_m_s / radius_m
    circle_time_s = (
        (roundabout_deg / 360.0)
        * (2.0 * math.pi * radius_m)
        / speed_m_s
    )
    
    service.send("robobot/cmd/ti", f"rc {speed_m_s:.3f} {omega_rad_s:.3f}")
    end_time = t.monotonic() + circle_time_s
    while t.monotonic() < end_time and not service.stop:
        if stop_requested():
            break
        t.sleep(min(0.02, end_time - t.monotonic()))
    service.send("robobot/cmd/ti", "rc 0.0 0.0")


def set_line_leds(r, g, b):
    for led_id in (14, 15, 16):
        service.send("robobot/cmd/T0", f"leds {led_id} {r} {g} {b}")


def flat_indicator_task(stop_event):
    """Background task: show green when flat, red when not flat."""
    last_flat = None
    while not stop_event.is_set() and not service.stop:
        if imu.gyroUpdCnt > 0 and imu.accUpdCnt > 0:
            flat, _, _, _ = imu.is_flat_surface(
                max_tilt_deg=RUN_MAX_TILT_DEG,
                max_gyro_dps=RUN_MAX_GYRO_DPS,
                g_min=ACC_G_MIN,
                g_max=ACC_G_MAX,
            )
            if flat != last_flat:
                if flat:
                    set_line_leds(0, 80, 0)
                else:
                    set_line_leds(80, 0, 0)
                last_flat = flat
        stop_event.wait(0.1)


def calibrate_before_run():
    print("% mission-run: calibration mode = current pose flat reference")
    imu.set_flat_thresholds(
        max_tilt_deg=RUN_MAX_TILT_DEG,
        max_gyro_dps=RUN_MAX_GYRO_DPS,
        g_min=ACC_G_MIN,
        g_max=ACC_G_MAX,
    )

    ok = False
    try:
        ok = imu.calibrate_flat_reference_current_pose(
            samples=60,
            sample_dt=0.02,
            g_min=ACC_G_MIN,
            g_max=ACC_G_MAX,
        )
    except Exception:
        ok = False

    if ok:
        print("% mission-run: flat reference calibrated from current pose")
    else:
        print("% mission-run: flat reference calibration failed")

    return ok

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
                servo.servo_change_position(200)
                service.stop = True
                return
        else:
            # Rotate to find ball if area too small
            service.send("robobot/cmd/ti", f"rc 0.00 {SEARCH_TURN_SPEED}")
    else:
        # Active search rotation
        service.send("robobot/cmd/ti", f"rc 0.00 {SEARCH_TURN_SPEED}")
        service.send("robobot/cmd/T0", "leds 16 0 30 30") # CYAN: Searching

def loop():
    print("% mission-run: start")
    state = STATE_GOING_UNTIL_INTERSECTION_AFTER_BALL
    lost_count = 0
    searching = False
    is_after_intersection = 0
    post_roundabout_started = False 
    go_to_end_initialized = False
    go_to_end_timer = None

    edge.lineControl(0, True)

    while not service.stop:
        if stop_requested():
            break
            
        if state == STATE_SEARCHING:
            # Move slowly forward while searching for a valid line.
            if not searching:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti", f"rc {SEARCH_SPEED:.2f} 0.00")
                searching = True

            if edge.lineValidCnt > LINE_VALID_MIN:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                edge.lineControl(FOLLOW_SPEED, True)
                lost_count = 0
                searching = False
                state = STATE_FOLLOWING
                print("% mission-run: state 0 -> 10 (follow line)")

        elif state == STATE_FOLLOWING:
            # Following line. When line is lost, climb slowly until flat.
            if edge.lineValidCnt > LINE_VALID_MIN:
                lost_count = 0
            else:
                lost_count += 1
                if lost_count >= LOST_DEBOUNCE_COUNT:
                    print("% mission-run: line lost -> climb to platform")
                    edge.lineControl(0, True)
                    pose.tripBreset()
                    service.send("robobot/cmd/ti", f"rc {CLIMB_SPEED:.2f} 0.00")
                    state = STATE_CLIMB_TO_FLAT

        elif state == STATE_CLIMB_TO_FLAT:
            flat = False
            if imu.gyroUpdCnt > 0 and imu.accUpdCnt > 0:
                flat, tilt_deg, gyro_norm, acc_norm = imu.is_flat_surface(
                    max_tilt_deg=RUN_MAX_TILT_DEG,
                    max_gyro_dps=RUN_MAX_GYRO_DPS,
                    g_min=ACC_G_MIN,
                    g_max=ACC_G_MAX,
                )
                if flat:
                    print(
                        f"% mission-run: flat reached, tilt={tilt_deg:.1f}, "
                        f"gyro={gyro_norm:.2f}, acc={acc_norm:.2f}, dist={pose.tripB:.3f}"
                    )
                    state = STATE_ROUNDABOUT

            if pose.tripBtimePassed() > CLIMB_TIMEOUT:
                print(f"% mission-run: climb timeout after {pose.tripB:.3f} m -> stopping")
                state = STATE_ROUNDABOUT
                
            
        elif state == STATE_ROUNDABOUT:
            roundabout(POST_ROUNDABOUT_DEG,
                POST_TURN_DEG,
                radius_m=POST_CIRCLE_RADIUS_M,
                speed_m_s=POST_CIRCLE_SPEED_M_S)
            if service.stop:
                break
            state = STATE_FIND_LINE_AFTER_ROUNDABOUT
            post_roundabout_started = False
            searching = False

        elif state == STATE_FIND_LINE_AFTER_ROUNDABOUT:
            if not post_roundabout_started:
                turn_with_feedback(
                    POST_SEARCH_TURN_DEG,
                    turn_rate_deg_s=POST_TURN_RATE_DEG_S,
                    forward_m_s=POST_SEARCH_TURN_FORWARD_SPEED,
                    stop_after=True,
                )
                if stop_requested():
                    break

                service.send("robobot/cmd/ti", f"rc {POST_SEARCH_STRAIGHT_SPEED:.2f} 0.00")
                end_time = t.monotonic() + POST_SEARCH_STRAIGHT_TIME_S
                while t.monotonic() < end_time and not service.stop:
                    if stop_requested():
                        break
                    t.sleep(min(0.02, end_time - t.monotonic()))
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                searching = False
                post_roundabout_started = True
                if service.stop:
                    break

            if not searching:
                edge.lineControl(0, False)
                service.send(
                    "robobot/cmd/ti",
                    f"rc {SEARCH_SPEED:.2f} {POST_SEARCH_LEFT_TURN_RAD_S:.2f}",
                )
                searching = True

            if edge.lineValidCnt > LINE_VALID_MIN:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                edge.lineControl(FOLLOW_SPEED, False, 2)
                lost_count = 0
                searching = False
                state = STATE_FOLLOWING_AFTER_ROUNDABOUT
                print("% mission-run: state 40 -> 50 (follow right line)")
                
        elif state == STATE_FOLLOWING_AFTER_ROUNDABOUT:
            if edge.splitDetected:
                print("% mission-run: second branch/split detected -> locate ball")
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = STATE_LOCATE_BALL
                    
        elif state == STATE_LOCATE_BALL:
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
            locate_ball(contours, img_center_x)
            state = STATE_GOING_UNTIL_INTERSECTION_AFTER_BALL

            #  here is missing the code from testing_ball_hole, when that is finished put here until it gets line and starts to follow on the left
            
        elif state == STATE_GOING_UNTIL_INTERSECTION_AFTER_BALL:
            # Follow line until intersection is detected
            edge.lineControl(FOLLOW_SPEED, True)
            
            if edge.intersectionDetected:
                is_after_intersection += 1
                edge.lineControl(0, True)
                
            if is_after_intersection == 1 and edge.intersectionDetected:
                print("% mission-run: first intersection detected after ball -> go 2 meters straight and back")
                driveOneMeter(forwards=True)
                driveOneMeter(forwards=True)
                driveOneMeter(forwards=False)
                driveOneMeter(forwards=False)
                
                
            if is_after_intersection == 2 and edge.intersectionDetected:
                print("% mission-run: second intersection detected after ball -> turn 90 def left and follow")
                turn_with_feedback(90, turn_rate_deg_s=90, forward_m_s=0.0, stop_after=True)
            if is_after_intersection == 3 and edge.intersectionDetected:
                print("% mission-run: third intersection detected after ball -> go straight")
                service.send("robobot/cmd/ti", f"rc {FOLLOW_SPEED:.2f} 0.00")
                t.sleep(1.0)
                state = STATE_FOLLOWING_UNTIL_ROUNDABOUT_TWO    
                lost_count = 0
                searching = False
        
        elif state == STATE_FOLLOWING_UNTIL_ROUNDABOUT_TWO:
            edge.lineControl(FOLLOW_SPEED, True)
            print(f"% mission-run: following until second roundabout, lineValidCnt={edge.lineValidCnt}")
            if edge.lineValidCnt > LINE_VALID_MIN:
                lost_count = 0
            else:
                lost_count += 1
                if lost_count >= LOST_DEBOUNCE_COUNT:
                    print("% mission-run: line lost -> go reverse")
                    edge.lineControl(0, True)
                    # reverse a bit to give more space for the turn
                    service.send("robobot/cmd/ti", f"rc {REVERSE_SPEED:.2f} 0.00")
                    t.sleep(2)
                    state = STATE_ROUNDABOUT_TWO
                
        elif state == STATE_ROUNDABOUT_TWO:
            print("% mission-run: second roundabout -> turn and circle")
            roundabout(SEC_ROUNDABOUT_TURN_DEG, SEC_BEFORE_ROUNDABOUT_TURN_DEG , radius_m=SEC_ROUNDABOUT_RADIUS_M, speed_m_s=SEC_CIRCLE_SPEED_M_S)
            state = STATE_FIND_LINE_AFTER_ROUNDABOUT_TWO
            searching = True
            edge.lineControl(0, False)
            
        elif state == STATE_FIND_LINE_AFTER_ROUNDABOUT_TWO:
            if searching:
                if edge.lineValidCnt <= LINE_VALID_MIN:
                    # No valid line yet: drive straight to reacquire it.
                    service.send("robobot/cmd/ti", f"rc {GO_STRAIGHT_SPEED:.2f} 0.00")
                else:
                    # Line found: switch back to normal line following.
                    print("% mission-run: line found after second roundabout -> follow line")
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    searching = False
            else:
                edge.lineControl(FOLLOW_SPEED, False)
                t.sleep(0.5)  # small delay to allow lineValidCnt to update
                if edge.intersectionDetected:
                    print("% mission-run: intersection detected after second roundabout -> go to end")
                    edge.lineControl(0, True)
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    state = STATE_GO_TO_END

        elif state == STATE_GO_TO_END:
            # Initialize: turn left 90 deg and go straight for 3 seconds, ignoring everything
            if not go_to_end_initialized:
                print("% mission-run: going to end - turning left 90 deg")
                turn_with_feedback(110, turn_rate_deg_s=110, forward_m_s=0.0, stop_after=True)
                if stop_requested():
                    break
                
                print("% mission-run: going straight for 3 seconds (ignore all)")
                service.send("robobot/cmd/ti", f"rc {GO_STRAIGHT_SPEED:.2f} 0.00")
                go_to_end_timer = t.monotonic() + 3.0
                go_to_end_initialized = True
            
            # Phase 1: Go straight for 3 seconds without checking anything
            elif go_to_end_timer is not None and t.monotonic() < go_to_end_timer:
                # Just keep going straight, ignore everything
                pass
            
            # Phase 2: After 3 seconds, go straight until line found
            else:
                if go_to_end_timer is not None:
                    print("% mission-run: 3 seconds done - searching for line")
                    edge.lineControl(0, True)
                    service.send("robobot/cmd/ti", f"rc {GO_STRAIGHT_SPEED:.2f} 0.00")
                    go_to_end_timer = None
                
                # Go straight until line is found
                if edge.lineValidCnt <= LINE_VALID_MIN:
                    # Still searching for line
                    service.send("robobot/cmd/ti", f"rc {GO_STRAIGHT_SPEED:.2f} 0.00")
                else:
                    # Line found! Switch to following on left
                    print("% mission-run: line found at end -> following on left, go to finale stretch")
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    edge.lineControl(FOLLOW_SPEED, True)  # Follow on left
                    state = STATE_FINALE_STRETCH
                
        elif state == STATE_FINALE_STRETCH:
            # When split detected, switch to follow right
            if edge.splitDetected:
                print("% mission-run: split detected at end -> follow right line to goal")
                edge.lineControl(FOLLOW_SPEED, False)  # Follow on right
                
        t.sleep(0.05)

    set_line_leds(0, 0, 0)
    edge.lineControl(0, True)
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    gpio.set_value(20, 0)
    print("% mission-run: stopped")


if __name__ == "__main__":
    print("% mission-run: initializing")
    indicator_stop = threading.Event()
    indicator_thread = None
    service.setup("localhost")
    try:
        if not service.connected:
            print("% mission-run: MQTT not connected")
        elif service.stop or robot.hbtUpdCnt == 0:
            print("% mission-run: startup failed (no robot heartbeat)")
            print("% mission-run: start teensy_interface and try again")
        else:
            indicator_thread = threading.Thread(
                target=flat_indicator_task,
                args=(indicator_stop,),
                daemon=True,
            )
            indicator_thread.start()
            # calibrate_before_run()
            loop()
    finally:
        indicator_stop.set()
        if indicator_thread is not None:
            indicator_thread.join(timeout=1.0)
        set_line_leds(0, 0, 0)
        service.terminate()
    print("% mission-run: terminated")
