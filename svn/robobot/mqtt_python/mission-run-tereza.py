#!/usr/bin/env python3

import threading
import time as t

from uservice import service
from sedge import edge
from spose import pose
from simu import imu
from sgpio import gpio
from srobot import robot
from motion_helper import turn_with_feedback
from sservo import servo


# Line-follow tuning
FOLLOW_SPEED = 0.20
SEARCH_SPEED = 0.1
CLIMB_SPEED = 0.08
CLIMB_TIMEOUT = 3.0           # sec safety timeout
LINE_VALID_MIN = 4
LOST_DEBOUNCE_COUNT = 8      # consecutive invalid reads before stop

# Post-flat maneuver tuning
POST_TURN_DEG = -60
POST_CIRCLE_RADIUS_M = 0.32
POST_CIRCLE_SPEED_M_S = 0.12
POST_TURN_RATE_DEG_S = 45.0

# Flat detection tuning 
RUN_MAX_GYRO_DPS = 20.0
RUN_MAX_TILT_DEG = 1
ACC_G_MIN = 0.7
ACC_G_MAX = 1.3

# states
STATE_SEARCHING = 0
STATE_FOLLOWING = 10
STATE_CLIMB_TO_FLAT = 20
STATE_ROUNDABOUT = 30


def roundabout():
    """After reaching the flat area: turn right 60 deg and drive one left circle."""
    if service.stop or gpio.test_stop_button():
        service.stop = True
        print("% mission-run: stop requested before roundabout")
        return

    print(f"% mission-run: post-flat maneuver -> turn {POST_TURN_DEG} deg")
    turn_with_feedback(POST_TURN_DEG, turn_rate_deg_s=POST_TURN_RATE_DEG_S, forward_m_s=0.0, stop_after=True)

    if service.stop or gpio.test_stop_button():
        service.stop = True
        print("% mission-run: stop requested after turn")
        return

    omega_rad_s = POST_CIRCLE_SPEED_M_S / POST_CIRCLE_RADIUS_M
    circle_time_s = (2.0 * 3.141592653589793 * POST_CIRCLE_RADIUS_M) / POST_CIRCLE_SPEED_M_S

    print(
        f"% mission-run: left circle speed={POST_CIRCLE_SPEED_M_S:.3f} m/s, "
        f"omega={omega_rad_s:.3f} rad/s, t={circle_time_s:.2f} s"
    )
    service.send("robobot/cmd/ti", f"rc {POST_CIRCLE_SPEED_M_S:.3f} {omega_rad_s:.3f}")
    end_time = t.monotonic() + circle_time_s
    while t.monotonic() < end_time and not service.stop:
        if gpio.test_stop_button():
            service.stop = True
            print("% mission-run: stop requested during circle")
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

def loop():
    print("% mission-run: start")
    state = STATE_SEARCHING
    lost_count = 0
    searching = False

    edge.lineControl(0, True)

    while not service.stop:
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
            break 

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
            servo.servo_change_position(-900)
            loop()
    finally:
        indicator_stop.set()
        if indicator_thread is not None:
            indicator_thread.join(timeout=1.0)
        set_line_leds(0, 0, 0)
        service.terminate()
    print("% mission-run: terminated")
