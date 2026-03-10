#!/usr/bin/env python3

import threading
import time as t

from simu import imu
from uservice import service

CALIBRATION_MAX_GYRO_DPS = 20.0
RUN_MAX_GYRO_DPS = 20.0
RUN_MAX_TILT_DEG = 1.0
ACC_G_MIN = 0.7
ACC_G_MAX = 1.3


def set_line_leds(r, g, b):
	"""Set all three line LEDs to the same color."""
	for led_id in (14, 15, 16):
		service.send("robobot/cmd/T0", f"leds {led_id} {r} {g} {b}")


def calibration_loading_lights(stop_event, led_id=16):
	"""Animate LED while IMU calibration is running."""
	steps = [
		(30, 15, 0),
		(30, 30, 0),
		(15, 30, 0),
		(0, 30, 0),
	]
	idx = 0

	while not stop_event.is_set():
		r, g, b = steps[idx]
		service.send("robobot/cmd/T0", f"leds {led_id} {r} {g} {b}")
		idx = (idx + 1) % len(steps)
		stop_event.wait(0.2)


def loop():
	print("% flat-test: running (green=flat, red=not flat)")
	last_flat = None
	last_status_print = t.time()

	while not service.stop:
		if imu.gyroUpdCnt > 0 and imu.accUpdCnt > 0:
			flat, tilt_deg, gyro_norm, acc_norm = imu.is_flat_surface()

			now = t.time()
			if now - last_status_print >= 0.5:
				state = "FLAT" if flat else "NOT FLAT"
				print(
					f"% flat-test: state={state}, tilt={tilt_deg:.1f} deg "
					f"(limit {RUN_MAX_TILT_DEG:.1f}), gyro={gyro_norm:.2f}, acc={acc_norm:.2f}, "
					f"upd(acc/gyro)=({imu.accUpdCnt}/{imu.gyroUpdCnt})"
				)
				last_status_print = now

			if flat != last_flat:
				if flat:
					set_line_leds(0, 100, 0)
					print(
						f"% flat-test: FLAT -> green (tilt={tilt_deg:.1f}, "
						f"gyro={gyro_norm:.2f}, acc={acc_norm:.2f})"
					)
				else:
					set_line_leds(100, 0, 0)
					print(
						f"% flat-test: NOT FLAT -> red (tilt={tilt_deg:.1f}, "
						f"gyro={gyro_norm:.2f}, acc={acc_norm:.2f})"
					)
				last_flat = flat

		t.sleep(0.05)


if __name__ == "__main__":
	print("% flat-test: initializing")
	service.setup("localhost")

	if not service.connected:
		print("% flat-test: MQTT not connected")
	else:
		try:
			print("% flat-test: calibration mode = current pose as flat reference")
			imu.set_flat_thresholds(
				max_tilt_deg=RUN_MAX_TILT_DEG,
				max_gyro_dps=RUN_MAX_GYRO_DPS,
				g_min=ACC_G_MIN,
				g_max=ACC_G_MAX,
			)
			print(
				f"% flat-test: accel-norm window calibration/run = "
				f"{ACC_G_MIN:.2f}..{ACC_G_MAX:.2f}"
			)
			calibration_stop = threading.Event()
			calibration_led_thread = threading.Thread(
				target=calibration_loading_lights,
				args=(calibration_stop,),
				daemon=True,
			)
			calibration_led_thread.start()
			ok = False
			try:
				ok = imu.calibrate_flat_reference_current_pose(
					samples=60,
					sample_dt=0.02,
					g_min=ACC_G_MIN,
					g_max=ACC_G_MAX,
				)
			finally:
				calibration_stop.set()
				calibration_led_thread.join(timeout=1.0)
			if ok:
				print("% flat-test: calibrated -> full line red for 3 seconds")
				set_line_leds(100, 0, 0)
				t.sleep(3.0)
			if not ok:
				print("% flat-test: calibration failed, using default axis reference")
			loop()
		finally:
			set_line_leds(0, 0, 0)
			service.terminate()
			print("% flat-test: terminated")
