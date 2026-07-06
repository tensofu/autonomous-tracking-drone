#!/usr/bin/env python3
"""Command an ArduPilot SITL copter over MAVLink: modes, arming, takeoff,
position moves, hold and land.

CLI usage (each invocation connects, acts, then exits):
    python drone_control.py status                 # telemetry snapshot
    python drone_control.py mode GUIDED            # switch flight mode
    python drone_control.py arm                    # arm motors
    python drone_control.py disarm
    python drone_control.py takeoff 10             # guided takeoff to 10 m (arms if needed)
    python drone_control.py goto 20 5 10           # fly 20 m north, 5 m east, at 10 m alt
    python drone_control.py hold                   # hold current position (LOITER)
    python drone_control.py land
    python drone_control.py rtl                    # return to launch
    python drone_control.py demo                   # takeoff 10 -> square pattern -> land

Library usage:
    from drone_control import DroneController
    drone = DroneController('udpin:127.0.0.1:14550')
    drone.takeoff(10)
    drone.goto(north=20, east=0, alt=10)
    drone.land()

Connections: udpin:127.0.0.1:14550 (via MAVProxy, default) or
tcp:127.0.0.1:5762 (direct to SITL, works alongside MAVProxy).
"""

import argparse
import sys
import time

from pymavlink import mavutil


class CommandError(RuntimeError):
    """A command was rejected by the vehicle or timed out."""


class CommandAborted(CommandError):
    """A long-running command was interrupted via the abort callback."""


# overwrite progress lines in a terminal; emit normal lines when piped
PROGRESS_END = '\r' if sys.stdout.isatty() else '\n'


class DroneController:

    def __init__(self, connection_string='udpin:127.0.0.1:14550', timeout=20,
                 on_status=None, on_progress=None):
        """on_status/on_progress: optional callables taking one string, for
        embedding in a GUI. Defaults print to the terminal."""
        self._say = on_status or print
        self._progress = on_progress or (lambda s: print(s, end=PROGRESS_END))
        self._say(f"Connecting to {connection_string} ...")
        self.master = mavutil.mavlink_connection(connection_string)
        if self.master.wait_heartbeat(timeout=timeout) is None:
            raise CommandError(f"no heartbeat within {timeout}s - is SITL running?")
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
        self._say(f"  connected: system {self.master.target_system}, "
                  f"mode {self.mode()}, {'ARMED' if self.is_armed() else 'disarmed'}")

    @staticmethod
    def _check_abort(abort):
        if abort is not None and abort():
            raise CommandAborted("aborted")

    def drain(self):
        """Discard buffered messages so subsequent state reads are current.
        Call after the connection has sat idle - the stream keeps filling
        the socket, and stale HEARTBEAT/position messages would otherwise
        make is_armed()/altitude() report minutes-old state."""
        while self.master.recv_match(blocking=False) is not None:
            pass

    # ------------------------------------------------------------- state --

    def _heartbeat(self):
        """Next heartbeat from the vehicle itself. ArduPilot routes other
        GCS heartbeats (MAVProxy etc.) onto every link, and those never
        carry the armed bit - accepting one makes a flying vehicle look
        disarmed."""
        deadline = time.time() + 5
        while time.time() < deadline:
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True,
                                        timeout=2)
            if (hb is not None
                    and hb.get_srcSystem() == self.master.target_system
                    and hb.type != mavutil.mavlink.MAV_TYPE_GCS):
                return hb
        raise CommandError("heartbeat stream lost")

    def is_armed(self):
        return bool(self._heartbeat().base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def mode(self):
        return mavutil.mode_string_v10(self._heartbeat())

    def altitude(self):
        """Relative altitude in metres (drains first: always current)."""
        self.drain()
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT',
                                     blocking=True, timeout=5)
        if msg is None:
            raise CommandError("no position telemetry")
        return msg.relative_alt / 1000.0

    def is_flying(self):
        return self.is_armed() and self.altitude() > 0.3

    def _require_flying(self, action):
        if not self.is_flying():
            raise CommandError(f"cannot {action}: vehicle is not flying "
                               "(disarmed or on the ground) - run takeoff first")

    def status(self):
        pos = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        att = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=5)
        batt = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        print(f"  mode     : {self.mode()} ({'ARMED' if self.is_armed() else 'disarmed'})")
        if pos:
            print(f"  altitude : {pos.relative_alt / 1000:.2f} m relative")
            print(f"  position : lat {pos.lat / 1e7:.7f}  lon {pos.lon / 1e7:.7f}")
        if att:
            print(f"  attitude : roll {att.roll:+.3f}  pitch {att.pitch:+.3f}  "
                  f"yaw {att.yaw:+.3f} rad")
        if batt:
            print(f"  battery  : {batt.voltage_battery / 1000:.2f} V, "
                  f"{batt.battery_remaining}% remaining")

    # ---------------------------------------------------------- commands --

    def _command(self, command, *params, timeout=10):
        """Send COMMAND_LONG and raise unless the vehicle ACKs it."""
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            command, 0, *(list(params) + [0] * (7 - len(params))))
        deadline = time.time() + timeout
        while time.time() < deadline:
            ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
            if ack and ack.command == command:
                if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    result = mavutil.mavlink.enums['MAV_RESULT'][ack.result].name
                    raise CommandError(f"command {command} rejected: {result} "
                                       f"({self._last_statustext()})")
                return
        raise CommandError(f"no acknowledgement for command {command}")

    def _last_statustext(self):
        """Best-effort fetch of the vehicle's explanation for a rejection."""
        msg = self.master.recv_match(type='STATUSTEXT', blocking=True, timeout=2)
        return msg.text if msg else "no reason given"

    def set_mode(self, mode_name):
        mode_name = mode_name.upper()
        mapping = self.master.mode_mapping()
        if mode_name not in mapping:
            raise CommandError(f"unknown mode {mode_name}; "
                               f"choose from {sorted(mapping)}")
        self._command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                      mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                      mapping[mode_name])
        self._say(f"  mode -> {mode_name}")

    def arm(self, timeout=30, abort=None):
        """Arm and wait for confirmation. Retries while pre-arm checks settle."""
        deadline = time.time() + timeout
        while True:
            self._check_abort(abort)
            try:
                self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
                break
            except CommandError as exc:
                if time.time() > deadline:
                    raise CommandError(f"arming failed: {exc}") from exc
                self._say(f"  arming refused ({exc}); retrying ...")
                time.sleep(3)
        self.master.motors_armed_wait()
        self._say("  ARMED")

    def disarm(self):
        self._command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        self.master.motors_disarmed_wait()
        self._say("  disarmed")

    def takeoff(self, target_alt, timeout=60, abort=None):
        """Guided takeoff to target_alt metres; arms first if necessary."""
        self.set_mode('GUIDED')
        if not self.is_armed():
            self.arm(abort=abort)
        self._command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                      0, 0, 0, 0, 0, 0, target_alt)
        self._say(f"  taking off to {target_alt} m ...")
        self._wait_altitude(target_alt, timeout, abort=abort)

    def _wait_altitude(self, target_alt, timeout, fraction=0.95, abort=None):
        deadline = time.time() + timeout
        last_print = 0.0
        while time.time() < deadline:
            self._check_abort(abort)
            alt = self.altitude()
            if time.time() - last_print >= 1.0:
                self._progress(f"    alt {alt:5.2f} m")
                last_print = time.time()
            if alt >= target_alt * fraction:
                self._say(f"  reached {alt:.2f} m")
                return
            time.sleep(0.5)
        raise CommandError(f"did not reach {target_alt} m within {timeout}s")

    def goto(self, north, east, alt, speed=None, wait=True, timeout=120,
             abort=None):
        """Fly to a position given in metres north/east of HOME at `alt`
        metres above home. Vehicle must be flying in GUIDED mode."""
        self._require_flying("goto")
        self.set_mode('GUIDED')
        if speed:
            self._command(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 1, speed, -1)
        # local NED relative to EKF origin (home): down is negative altitude
        type_mask = 0b0000_1111_1111_1000  # only x/y/z position enabled
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask, north, east, -alt, 0, 0, 0, 0, 0, 0, 0, 0)
        self._say(f"  goto north {north:.0f} m, east {east:.0f} m, "
                  f"alt {alt:.0f} m ...")
        if not wait:
            return
        deadline = time.time() + timeout
        last_print = 0.0
        while time.time() < deadline:
            self._check_abort(abort)
            msg = self.master.recv_match(type='LOCAL_POSITION_NED',
                                         blocking=True, timeout=5)
            if msg is None:
                continue
            dist = ((msg.x - north) ** 2 + (msg.y - east) ** 2
                    + (msg.z + alt) ** 2) ** 0.5
            if time.time() - last_print >= 2.0:
                self._progress(f"    distance to target {dist:6.1f} m")
                last_print = time.time()
            if dist < 1.0:
                self._say("  target reached")
                return
        raise CommandError(f"target not reached within {timeout}s")

    def hold(self):
        """Hold the current position by re-targeting it in GUIDED mode.

        Deliberately not LOITER: pilot-controlled modes follow the RC
        throttle stick, and with no RC input (SITL default, or any pure
        GCS setup) the stick reads low and the copter descends."""
        self._require_flying("hold")
        msg = self.master.recv_match(type='LOCAL_POSITION_NED',
                                     blocking=True, timeout=5)
        if msg is None:
            raise CommandError("no local position telemetry")
        self.goto(msg.x, msg.y, -msg.z, wait=False)
        self._say(f"  holding at north {msg.x:.1f} m, east {msg.y:.1f} m, "
                  f"alt {-msg.z:.1f} m")

    def land(self, timeout=90):
        """Land at the current position and wait until motors disarm."""
        self.set_mode('LAND')
        self._say("  landing ...")
        deadline = time.time() + timeout
        last_print = 0.0
        while time.time() < deadline:
            if not self.is_armed():
                self._say("  landed and disarmed")
                return
            if time.time() - last_print >= 1.0:
                self._progress(f"    alt {self.altitude():5.2f} m")
                last_print = time.time()
            time.sleep(0.5)
        raise CommandError(f"still airborne after {timeout}s")

    def rtl(self):
        """Return to launch (flies home and lands automatically)."""
        self.set_mode('RTL')
        self._say("  returning to launch")

    def return_home(self, alt=None, abort=None):
        """Fly back to the origin (takeoff point) and hover there.
        Unlike RTL this does not land. Default altitude: keep the current
        one, but at least 5 m."""
        self._require_flying("return home")
        if alt is None:
            alt = max(self.altitude(), 5.0)
        self._say("  returning to origin ...")
        self.goto(0, 0, alt, abort=abort)


def demo(drone):
    """Takeoff, fly a 20 m square, land."""
    drone.takeoff(10)
    for north, east in ((20, 0), (20, 20), (0, 20), (0, 0)):
        drone.goto(north, east, 10)
    drone.land()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--connect', default='udpin:127.0.0.1:14550',
                        help='connection string (default: %(default)s)')
    sub = parser.add_subparsers(dest='action', required=True)

    sub.add_parser('status')
    sub.add_parser('arm')
    sub.add_parser('disarm')
    sub.add_parser('hold')
    sub.add_parser('land')
    sub.add_parser('rtl')
    sub.add_parser('home', help='fly back to origin and hover')
    sub.add_parser('demo')
    p = sub.add_parser('mode')
    p.add_argument('name', help='flight mode, e.g. GUIDED, LOITER, RTL')
    p = sub.add_parser('takeoff')
    p.add_argument('altitude', type=float, help='target altitude in metres')
    p = sub.add_parser('goto')
    p.add_argument('north', type=float, help='metres north of home')
    p.add_argument('east', type=float, help='metres east of home')
    p.add_argument('altitude', type=float, help='metres above home')
    p.add_argument('--speed', type=float, help='horizontal speed m/s')
    args = parser.parse_args()

    try:
        drone = DroneController(args.connect)
        if args.action == 'status':
            drone.status()
        elif args.action == 'mode':
            drone.set_mode(args.name)
        elif args.action == 'arm':
            drone.arm()
        elif args.action == 'disarm':
            drone.disarm()
        elif args.action == 'takeoff':
            drone.takeoff(args.altitude)
        elif args.action == 'goto':
            drone.goto(args.north, args.east, args.altitude, speed=args.speed)
        elif args.action == 'hold':
            drone.hold()
        elif args.action == 'land':
            drone.land()
        elif args.action == 'rtl':
            drone.rtl()
        elif args.action == 'home':
            drone.return_home()
        elif args.action == 'demo':
            demo(drone)
    except CommandError as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
