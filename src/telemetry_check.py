#!/usr/bin/env python3
"""Connect to an ArduPilot SITL instance, read telemetry, and verify
two-way MAVLink communication.

Usage:
    python telemetry_check.py                        # via MAVProxy (udpin:127.0.0.1:14550)
    python telemetry_check.py --connect tcp:127.0.0.1:5762   # direct to SITL, alongside MAVProxy
    python telemetry_check.py --watch 10             # also stream telemetry for 10 seconds

Connection notes:
  - udpin:127.0.0.1:14550  works while MAVProxy is running (it forwards there).
  - tcp:127.0.0.1:5762     is SITL's spare serial port; use it to connect
                           directly even when MAVProxy holds the main 5760 port.

Exit code 0 means all checks passed, 1 otherwise.
"""

import argparse
import sys
import time

from pymavlink import mavutil


def connect(connection_string, timeout=20):
    """Connect and wait for a heartbeat (proves vehicle -> GCS traffic)."""
    print(f"Connecting to {connection_string} ...")
    master = mavutil.mavlink_connection(connection_string)
    hb = master.wait_heartbeat(timeout=timeout)
    if hb is None:
        raise TimeoutError(f"no heartbeat within {timeout}s - is SITL running?")
    print(f"  heartbeat OK: system {master.target_system}, "
          f"component {master.target_component}, "
          f"vehicle type {hb.type}, autopilot {hb.autopilot}")
    return master


def request_streams(master, rate_hz=4):
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, rate_hz, 1)


def read_telemetry(master, timeout=10):
    """Collect one sample each of position, attitude and battery."""
    wanted = {'GLOBAL_POSITION_INT': None, 'ATTITUDE': None, 'SYS_STATUS': None}
    deadline = time.time() + timeout
    while time.time() < deadline and None in wanted.values():
        msg = master.recv_match(type=list(wanted), blocking=True, timeout=2)
        if msg and wanted[msg.get_type()] is None:
            wanted[msg.get_type()] = msg

    missing = [k for k, v in wanted.items() if v is None]
    if missing:
        raise TimeoutError(f"telemetry not received within {timeout}s: {missing}")

    pos, att, sys_status = (wanted['GLOBAL_POSITION_INT'],
                            wanted['ATTITUDE'],
                            wanted['SYS_STATUS'])
    print("  telemetry OK:")
    print(f"    altitude : {pos.relative_alt / 1000:.2f} m relative "
          f"({pos.alt / 1000:.2f} m AMSL)")
    print(f"    attitude : roll {att.roll:+.3f}  pitch {att.pitch:+.3f}  "
          f"yaw {att.yaw:+.3f} rad")
    print(f"    battery  : {sys_status.voltage_battery / 1000:.2f} V, "
          f"{sys_status.current_battery / 100:.2f} A, "
          f"{sys_status.battery_remaining}% remaining")
    return wanted


def verify_two_way(master, timeout=10):
    """Prove GCS -> vehicle traffic with request/response round trips."""
    # Round trip 1: parameter read (vehicle must answer our specific request)
    master.mav.param_request_read_send(
        master.target_system, master.target_component, b'FRAME_CLASS', -1)
    param = None
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if msg and msg.param_id == 'FRAME_CLASS':
            param = msg
            break
    if param is None:
        raise TimeoutError("no PARAM_VALUE reply - GCS->vehicle link not confirmed")
    print(f"  param round trip OK: FRAME_CLASS = {param.param_value:.0f}")

    # Round trip 2: command + acknowledgement
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=timeout)
    if ack is None or ack.command != mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
        raise TimeoutError("no COMMAND_ACK - command channel not confirmed")
    result = mavutil.mavlink.enums['MAV_RESULT'][ack.result].name
    print(f"  command round trip OK: REQUEST_MESSAGE -> {result}")

    version = master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=5)
    if version:
        fw = version.flight_sw_version
        print(f"  firmware: {fw >> 24 & 0xff}.{fw >> 16 & 0xff}.{fw >> 8 & 0xff}")


def watch(master, seconds):
    """Stream live telemetry lines until the time is up."""
    print(f"\nStreaming telemetry for {seconds}s (Ctrl+C to stop) ...")
    alt = att = batt = None
    last_print = 0.0
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = master.recv_match(
            type=['GLOBAL_POSITION_INT', 'ATTITUDE', 'SYS_STATUS'],
            blocking=True, timeout=2)
        if msg is None:
            continue
        t = msg.get_type()
        if t == 'GLOBAL_POSITION_INT':
            alt = msg
        elif t == 'ATTITUDE':
            att = msg
        else:
            batt = msg
        if alt and att and batt and time.time() - last_print >= 1.0:
            print(f"  alt {alt.relative_alt / 1000:6.2f} m | "
                  f"roll {att.roll:+6.3f} pitch {att.pitch:+6.3f} "
                  f"yaw {att.yaw:+6.3f} | "
                  f"{batt.voltage_battery / 1000:5.2f} V "
                  f"{batt.battery_remaining:3d}%")
            last_print = time.time()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--connect', default='udpin:127.0.0.1:14550',
                        help='connection string (default: %(default)s)')
    parser.add_argument('--watch', type=float, default=0, metavar='SECONDS',
                        help='after the checks, stream telemetry for this long')
    args = parser.parse_args()

    try:
        master = connect(args.connect)
        request_streams(master)
        read_telemetry(master)
        verify_two_way(master)
        if args.watch:
            watch(master, args.watch)
    except TimeoutError as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 0

    print("\nAll checks passed: telemetry flowing and two-way link confirmed.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
