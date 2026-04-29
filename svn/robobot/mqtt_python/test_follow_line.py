##!/usr/bin/env python3

import time as t
from uservice import service
from sedge import edge
from sgpio import gpio

# Parameters
FOLLOW_SPEED = 0.3
LINE_VALID_MIN = 5


def stop_requested():
    if service.stop:
        return True
    if gpio.test_stop_button():
        service.stop = True
        return True
    return False


def run_follow_line():
    print("% test-follow-line: initializing")
    edge.lineControl(FOLLOW_SPEED, True)
    while not stop_requested():
        if edge.lineValidCnt > LINE_VALID_MIN:
            print(f"% test-follow-line: line valid count {edge.lineValidCnt}")
        else:
            print(f"% test-follow-line: line lost, valid count {edge.lineValidCnt}")
        t.sleep(0.05)
    # Stop everything on exit
    edge.lineControl(0, True)
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    print("% test-follow-line: stopped")


if __name__ == "__main__":
    service.setup("localhost")
    if not service.connected:
        print("% test-follow-line: MQTT not connected")
    else:
        try:
            # edge.calibrateLineSensor(samples=100)
            run_follow_line()
        finally:
            edge.lineControl(0, True)
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            service.terminate()
            print("% test-follow-line: terminated")
