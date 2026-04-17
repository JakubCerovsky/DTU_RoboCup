#!/usr/bin/env python3

import time as t

from uservice import service
from sgpio import gpio
from spose import pose


def set_leds(r, g, b):
    for led_id in (14, 15, 16):
        service.send("robobot/cmd/T0", f"leds {led_id} {r} {g} {b}")


def stop_requested():
    if service.stop:
        return True
    if gpio.test_stop_button():
        service.stop = True
        return True
    return False


def run_one_meter():
    """Drive exactly one meter forward."""
    state = 0
    pose.tripBreset()
    print("% one-meter-test: Driving 1m -------------------------")
    set_leds(0, 100, 0)  # green
    
    while not stop_requested():
        if state == 0:  # Start driving
            service.send("robobot/cmd/ti", "rc 0.2 0.0")  # (forward m/s, turn-rate rad/sec)
            state = 1
        elif state == 1:  # Check if 1m reached or timeout
            if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")  # stop
                state = 2
        elif state == 2:  # Wait for velocity to drop
            if abs(pose.velocity()) < 0.001:
                state = 99
        else:  # Finished
            print(f"% one-meter-test: drove {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds")
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            break
        
        print(f"% one-meter-test: state {state}, distance {pose.tripB:.3f}m, time {pose.tripBtimePassed():.3f}s")
        t.sleep(0.05)
    
    set_leds(0, 0, 0)  # LEDs off
    print("% one-meter-test: Driving 1m ----------------------- end")


if __name__ == "__main__":
    print("% one-meter-test: initializing")
    service.setup("localhost")

    if not service.connected:
        print("% one-meter-test: MQTT not connected")
    else:
        try:
            set_leds(0, 0, 80)  # blue: waiting
            run_one_meter()
        finally:
            set_leds(0, 0, 0)
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            service.terminate()
            print("% one-meter-test: terminated")
