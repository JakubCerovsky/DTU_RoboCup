#!/usr/bin/env python3

import math
import time as t

from spose import pose
from uservice import service


def rc_message(forward_m_s=0.0, turn_rate_rad_s=0.0):
  """Build an rc command payload: rc <forward m/s> <turn-rate rad/s>."""
  return f"rc {forward_m_s:.3f} {turn_rate_rad_s:.3f}"


def turn_rate_from_deg_s(deg_per_s):
  """Convert angular speed from deg/s to rad/s for the rc message."""
  return math.radians(deg_per_s)

def turn(degrees, turn_rate_deg_s=45.0, forward_m_s=0.0, stop_after=True):
  """Turn the robot by an angle in degrees using a timed open-loop command.

  This sends one rc command with the needed turn-rate and holds it for
  estimated time: |degrees| / turn_rate_deg_s.

  Returns:
    tuple: (duration_s, turn_rate_rad_s)
  """
  if turn_rate_deg_s == 0.0:
    raise ValueError("turn_rate_deg_s must be non-zero")

  rate_rad_s = abs(turn_rate_from_deg_s(turn_rate_deg_s))
  signed_rate = rate_rad_s if degrees >= 0 else -rate_rad_s
  duration_s = abs(degrees) / abs(turn_rate_deg_s)

  service.send("robobot/cmd/ti", rc_message(forward_m_s, signed_rate))
  t.sleep(duration_s)
  if stop_after:
    service.send("robobot/cmd/ti", rc_message(0.0, 0.0))

  return duration_s, signed_rate

def turn_with_feedback(degrees, turn_rate_deg_s=45.0, forward_m_s=0.0, timeout_s=8.0, stop_after=True):
  """Turn the robot by a target angle in degrees using pose feedback.

  The function commands a constant turn-rate (deg/s converted to rad/s),
  monitors pose.tripBh (radians), and stops when target angle is reached.

  Returns:
    tuple: (target_rad, reached_rad, timed_out)
  """
  target_rad = abs(math.radians(degrees))
  if target_rad == 0.0:
    if stop_after:
      service.send("robobot/cmd/ti", rc_message(0.0, 0.0))
    return 0.0, 0.0, False

  rate_rad_s = abs(turn_rate_from_deg_s(turn_rate_deg_s))
  if rate_rad_s == 0.0:
    raise ValueError("turn_rate_deg_s must be non-zero")

  signed_rate = rate_rad_s if degrees >= 0 else -rate_rad_s

  pose.tripBreset()
  service.send("robobot/cmd/ti", rc_message(forward_m_s, signed_rate))

  start = t.time()
  timed_out = False
  while not service.stop:
    if abs(pose.tripBh) >= target_rad:
      break
    if t.time() - start > timeout_s:
      timed_out = True
      break
    t.sleep(0.01)

  if stop_after:
    service.send("robobot/cmd/ti", rc_message(0.0, 0.0))

  return target_rad, abs(pose.tripBh), timed_out
