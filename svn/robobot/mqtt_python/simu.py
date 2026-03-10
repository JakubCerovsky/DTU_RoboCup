#/***************************************************************************
#*   Copyright (C) 2024 by DTU
#*   jcan@dtu.dk
#*
#*
#* The MIT License (MIT)  https://mit-license.org/
#*
#* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#* and associated documentation files (the “Software”), to deal in the Software without restriction,
#* including without limitation the rights to use, copy, modify, merge, publish, distribute,
#* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
#* is furnished to do so, subject to the following conditions:
#*
#* The above copyright notice and this permission notice shall be included in all copies
#* or substantial portions of the Software.
#*
#* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
#* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#* THE SOFTWARE. */


import time as t
import math
from datetime import *

class SImu:

    gyro = [0, 0, 0]
    gyroUpdCnt = 0
    gyroTime =datetime.now()
    gyroInterval = 1

    acc  = [0, 0, 0]
    accTime = datetime.now()
    accUpdCnt = 0
    accInterval = 1
    flatRef = [0.0, 0.0, 1.0]
    flatRefValid = False
    flatMaxTiltDeg = 8.0
    flatMaxGyroDps = 5.0
    flatGMin = 8.5
    flatGMax = 11.0

    def set_flat_thresholds(self, max_tilt_deg=None, max_gyro_dps=None, g_min=None, g_max=None):
      if max_tilt_deg is not None:
        self.flatMaxTiltDeg = max_tilt_deg
      if max_gyro_dps is not None:
        self.flatMaxGyroDps = max_gyro_dps
      if g_min is not None:
        self.flatGMin = g_min
      if g_max is not None:
        self.flatGMax = g_max

    def setup(self):
      # data subscription is set in teensy_interface/build/robot.ini
      from uservice import service
      loops = 0
      while not service.stop:
        t.sleep(0.01)
        if self.gyroUpdCnt == 0 or self.accUpdCnt == 0:
          # wait for data
          pass
        else: # finished
          print(f"% IMU (simu.py):: got data stream; {loops} loops.")
          break
        loops += 1
        if loops > 20:
          print(f"% IMU (simu.py):: no data updates after {loops} wait loops (continues).")
          break
        pass
        loops += 1
      # should we calibrate the gyro
      if service.args.gyro:
        print("% Starting calibrate gyro offset.")
        # ask Teensy to calibrate
        service.send("robobot/cmd/T0", "gyroc")
        # wait for calibration to finish (average over 1s)
        t.sleep(2.5)
        # save calibrated values
        service.send("robobot/cmd/T0", "eew")
        print("% Starting calibrate gyro offset finished.")
        t.sleep(0.5)
        # all done
        service.stop = True
      pass

    def print(self):
      from uservice import service
      print("% IMU acc  " + str(self.accTime - service.startTime) + " (" +
            str(self.acc[0]) + ", " +
            str(self.acc[1]) + ", " +
            str(self.acc[2]) + f") {self.gyroInterval:.4f} sec " +
            str(self.accUpdCnt))
      print("% IMU gyro " + str(self.gyroTime - service.startTime) + " (" +
            str(self.gyro[0]) + ", " +
            str(self.gyro[1]) + ", " +
            str(self.gyro[2]) + f") {self.accInterval:.4f} sec " +
            str(self.gyroUpdCnt))

    def decode(self, topic, msg):
        # decode MQTT message
        used = True
        if topic == "T0/gyro":
          gg = msg.split(" ")
          if (len(gg) >= 4):
            t0 = self.gyroTime;
            self.gyroTime = datetime.fromtimestamp(float(gg[0]))
            self.gyro[0] = float(gg[1])
            self.gyro[1] = float(gg[2])
            self.gyro[2] = float(gg[3])
            t1 = self.gyroTime;
            if self.gyroUpdCnt == 2:
              self.gyroInterval = (t1 -t0).total_seconds()
            else:
              self.gyroInterval = (self.gyroInterval * 99 + (t1 -t0).total_seconds()) / 100
            self.gyroUpdCnt += 1
            # self.print()
        elif topic == "T0/acc":
          gg = msg.split(" ")
          if (len(gg) >= 4):
            t0 = self.accTime;
            self.accTime = datetime.fromtimestamp(float(gg[0]))
            self.acc[0] = float(gg[1])
            self.acc[1] = float(gg[2])
            self.acc[2] = float(gg[3])
            t1 = self.accTime;
            if self.accUpdCnt == 2:
               self.accInterval = (t1 -t0).total_seconds()
            else:
               self.accInterval = (self.accInterval * 99 + (t1 -t0).total_seconds()) / 100
            self.accUpdCnt += 1
            # self.print()
        else:
          used = False
        return used

    def terminate(self):
        print("% Pose terminated")
        pass
      
    def calibrate_flat_reference_current_pose(self, samples=30, sample_dt=0.02,
                   g_min=7.0, g_max=12.5):
      """
      Set the current pose as flat reference by averaging recent accelerometer
      samples. This mode intentionally ignores gyro stability.
      """
      sx = 0.0
      sy = 0.0
      sz = 0.0
      used = 0
      loops = 0
      max_loops = samples * 6

      while used < samples and loops < max_loops:
        loops += 1
        if self.accUpdCnt <= 0:
          t.sleep(sample_dt)
          continue

        ax, ay, az = self.acc
        acc_norm = math.sqrt(ax*ax + ay*ay + az*az)

        if g_min <= acc_norm <= g_max:
          sx += ax
          sy += ay
          sz += az
          used += 1

        t.sleep(sample_dt)

      if used < max(5, samples // 3):
        print(f"% IMU::current-pose calibration failed (used {used}/{samples} samples)")
        return False

      nx = sx / used
      ny = sy / used
      nz = sz / used
      n_norm = math.sqrt(nx*nx + ny*ny + nz*nz)
      if n_norm < 1e-6:
        print("% IMU::current-pose calibration failed (zero reference norm)")
        return False

      self.flatRef = [nx / n_norm, ny / n_norm, nz / n_norm]
      self.flatRefValid = True
      print(f"% IMU::flat reference set from current pose ({used} samples)")
      return True
      
    def is_flat_surface(self, max_tilt_deg=None, max_gyro_dps=None, g_min=None, g_max=None):
      """Return (is_flat, tilt_deg, gyro_norm, acc_norm)."""
      # Use object defaults unless the caller overrides for this call.
      max_tilt = self.flatMaxTiltDeg if max_tilt_deg is None else max_tilt_deg
      max_gyro = self.flatMaxGyroDps if max_gyro_dps is None else max_gyro_dps
      min_g = self.flatGMin if g_min is None else g_min
      max_g = self.flatGMax if g_max is None else g_max

      ax, ay, az = self.acc
      gx, gy, gz = self.gyro

      acc_norm = math.sqrt(ax * ax + ay * ay + az * az)
      gyro_norm = math.sqrt(gx * gx + gy * gy + gz * gz)

      if acc_norm < 1e-6:
        return False, 90.0, gyro_norm, acc_norm

      if self.flatRefValid:
        rx, ry, rz = self.flatRef
        ref_dot = ax * rx + ay * ry + az * rz
        cos_tilt = abs(ref_dot / acc_norm)
      else:
        # Fallback before calibration: compare to z-axis.
        cos_tilt = abs(az) / acc_norm

      cos_tilt = max(-1.0, min(1.0, cos_tilt))
      tilt_deg = math.degrees(math.acos(cos_tilt))

      gravity_ok = min_g <= acc_norm <= max_g
      tilt_ok = tilt_deg <= max_tilt
      gyro_ok = gyro_norm <= max_gyro
      is_flat = gravity_ok and tilt_ok and gyro_ok

      return is_flat, tilt_deg, gyro_norm, acc_norm

# create the data object
imu = SImu()

