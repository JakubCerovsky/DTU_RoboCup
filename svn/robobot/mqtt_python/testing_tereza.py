#!/usr/bin/env python3

import time as t

from uservice import service
from motion_helper import turn_with_feedback
from sgpio import gpio
from sservo import servo

# Roundabout test tuning
POST_TURN_DEG = -60
POST_CIRCLE_RADIUS_M = 0.32
POST_CIRCLE_SPEED_M_S = 0.12
POST_TURN_RATE_DEG_S = 45.0


def set_line_leds(r, g, b):
    """Set all three line LEDs to the same color."""
    for led_id in (14, 15, 16):
        service.send("robobot/cmd/T0", f"leds {led_id} {r} {g} {b}")


def stop_requested():
    """Check whether the service or physical stop button requested a stop."""
    if service.stop:
        return True
    if gpio.test_stop_button():
        service.stop = True
        return True
    return False


def wait_with_stop(duration_s, step_s=0.02):
    """Wait for a duration while remaining responsive to the stop button."""
    end_time = t.monotonic() + duration_s
    while t.monotonic() < end_time:
        if stop_requested():
            return False
        t.sleep(min(step_s, end_time - t.monotonic()))
    return True


def roundabout():
    """Turn right, then drive one left circle."""
    if stop_requested():
        print("% roundabout-test: stop requested before turn")
        return

    print(f"% roundabout-test: turn {POST_TURN_DEG} deg")
    turn_with_feedback(
        POST_TURN_DEG,
        turn_rate_deg_s=POST_TURN_RATE_DEG_S,
        forward_m_s=0.0,
        stop_after=True,
    )

    if stop_requested():
        print("% roundabout-test: stop requested after turn")
        return

    omega_rad_s = POST_CIRCLE_SPEED_M_S / POST_CIRCLE_RADIUS_M
    circle_time_s = (2.0 * 3.141592653589793 * POST_CIRCLE_RADIUS_M) / POST_CIRCLE_SPEED_M_S

    print(
        f"% roundabout-test: left circle speed={POST_CIRCLE_SPEED_M_S:.3f} m/s, "
        f"omega={omega_rad_s:.3f} rad/s, t={circle_time_s:.2f} s"
    )
    service.send("robobot/cmd/ti", f"rc {POST_CIRCLE_SPEED_M_S:.3f} {omega_rad_s:.3f}")
    if not wait_with_stop(circle_time_s):
        print("% roundabout-test: stop requested during circle")
    service.send("robobot/cmd/ti", "rc 0.0 0.0")

if __name__ == "__main__":
    print("% roundabout-test: initializing")
    service.setup("localhost")

    if not service.connected:
        print("% roundabout-test: MQTT not connected")
    else:
        try:
            servo.servo_change_position(-900)
            set_line_leds(0, 0, 80)
            roundabout()
        finally:
            set_line_leds(0, 0, 0)
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            service.terminate()
            print("% roundabout-test: terminated")
