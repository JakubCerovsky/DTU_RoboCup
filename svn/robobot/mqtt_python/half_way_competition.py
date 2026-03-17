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
CLIMB_SPEED = 0.2
CLIMB_TIMEOUT = 2         # sec safety timeout
LINE_VALID_MIN = 4
LOST_DEBOUNCE_COUNT = 8      # consecutive invalid reads before stop

# Post-flat maneuver tuning
POST_TURN_DEG = -60
POST_CIRCLE_RADIUS_M = 0.32
POST_CIRCLE_SPEED_M_S = 0.12
ROUNDABOUT_DEG = 320
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

# states
STATE_SEARCHING = 0
STATE_FOLLOWING = 10
STATE_CLIMB_TO_FLAT = 20
STATE_ROUNDABOUT = 30
STATE_FIND_LINE_AFTER_ROUNDABOUT = 40
STATE_FOLLOWING_AFTER_ROUNDABOUT = 50
STATE_LOCATE_BALL = 60


def stop_requested():
    if gpio.test_stop_button():
        service.stop = True
        print("% mission-run: stop button pressed")
        return True
    return service.stop


def roundabout():
    """After reaching the flat area: turn right 60 deg and drive one left circle."""
    if stop_requested():
        return

    print(f"% mission-run: post-flat maneuver -> turn {POST_TURN_DEG} deg")
    turn_with_feedback(POST_TURN_DEG, turn_rate_deg_s=POST_TURN_RATE_DEG_S, forward_m_s=0.0, stop_after=True)

    if stop_requested():
        return

    omega_rad_s = POST_CIRCLE_SPEED_M_S / POST_CIRCLE_RADIUS_M
    circle_time_s = (
        (ROUNDABOUT_DEG / 360.0)
        * (2.0 * math.pi * POST_CIRCLE_RADIUS_M)
        / POST_CIRCLE_SPEED_M_S
    )
    
    service.send("robobot/cmd/ti", f"rc {POST_CIRCLE_SPEED_M_S:.3f} {omega_rad_s:.3f}")
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
    state = STATE_SEARCHING
    lost_count = 0
    searching = False
    post_roundabout_started = False 

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
            roundabout()
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
            if edge.lineValidCnt > LINE_VALID_MIN:
                lost_count = 0
            else:
                lost_count += 1
                if lost_count >= LOST_DEBOUNCE_COUNT:
                    print("% mission-run: line lost after roundabout -> stopping")
                    edge.lineControl(0, True)
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    service.stop = True
                    break
                
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
            calibrate_before_run()
            loop()
    finally:
        indicator_stop.set()
        if indicator_thread is not None:
            indicator_thread.join(timeout=1.0)
        set_line_leds(0, 0, 0)
        service.terminate()
    print("% mission-run: terminated")
