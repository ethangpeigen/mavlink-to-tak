#!/usr/bin/env python3
"""A fake vehicle for testing the bridge without a radio: sends MAVLink over UDP to 127.0.0.1.

    python sim_mavlink.py --port 14560
    (and in config.ini:  MAVLINK_PORT = udpin:127.0.0.1:14560)

Flies a slow circle around a landmark (the Washington Monument, nothing personal) as system 1, a quadcopter in LOITER.
"""
import argparse
import math
import time

from pymavlink import mavutil


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=14560)
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until Ctrl+C)")
    ap.add_argument("--lat", type=float, default=38.8895)
    ap.add_argument("--lon", type=float, default=-77.0353)
    a = ap.parse_args()
    m = mavutil.mavlink_connection("udpout:127.0.0.1:%d" % a.port, source_system=1, source_component=1)
    t0 = time.time()
    n = 0
    while True:
        t = time.time() - t0
        if a.seconds and t > a.seconds:
            break
        ang = t / 20.0
        lat = a.lat + 0.0004 * math.sin(ang)
        lon = a.lon + 0.0005 * math.cos(ang)
        hdg = (math.degrees(ang) + 90) % 360
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 5, 4)
        m.mav.gps_raw_int_send(int(t * 1e6), 3, int(lat * 1e7), int(lon * 1e7), 40000, 90, 120, 600, int(hdg * 100), 14)
        m.mav.global_position_int_send(int(t * 1000), int(lat * 1e7), int(lon * 1e7), 40000, 12000, 0, 0, 0, int(hdg * 100))
        m.mav.vfr_hud_send(6.0, 6.0, int(hdg), 55, 40.0, 0.0)
        m.mav.sys_status_send(0, 0, 0, 500, 15800, 700, 78, 0, 0, 0, 0, 0, 0)
        n += 1
        time.sleep(0.5)
    print("simulator stopped after %d cycles" % n)


if __name__ == "__main__":
    main()
