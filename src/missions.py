#!/usr/bin/env python3
"""Autonomy behaviors: follow a person and precision-land on the AprilTag
dock, both driven by the fixed nadir (straight-down) camera.

Each behavior is a control loop at ~5 Hz that reads the latest camera
detections (from vision.FrameSource.latest) and re-sends body-frame
velocity setpoints. `get_dets` is any callable returning
(detections_dict, age_seconds); `abort` is any callable returning True
to stop - both match what the GUI's worker provides.

No collision guard: the camera looks straight down (there is no gimbal -
see README "Custom drone body"), so there is no forward view to detect
obstacles in the flight path from. The previous guard used a horizon-line
heuristic that only makes sense for a forward/level camera; it does not
have a nadir-camera equivalent (a downward view sees the *tops* of nearby
obstacles, which alone doesn't tell you their height relative to the
drone). Real obstacle avoidance with this camera layout needs a separate
forward-facing sensor (a second camera, or a rangefinder) - flagged in
the README punch list, not solved here.

Sim vs real hardware: these loops only consume Detection objects, so on
the real drone the same code runs against the Raspberry Pi AI camera's
detections.
"""

import math
import time

from drone_control import CommandError, CommandAborted

LOOP_DT = 0.2                    # 5 Hz control loops
DET_MAX_AGE = 0.8                # detections older than this are "lost"


def _clamp(x, lim):
    return max(-lim, min(lim, x))


class _TargetSmoother:
    """Exponential smoothing on a position target. Each vision reading has
    pixel-level noise; feeding every raw noisy reading straight into
    send_position_target chases that noise (measured as ~1.7 attitude-rate
    reversals/sec while tracking - much faster than the real target's own
    motion), even after switching from velocity to position control fixed
    the larger overshoot problem. alpha=0.35 gives roughly a 0.4s time
    constant at 5 Hz - enough to average out single-frame jitter without
    meaningfully lagging real movement (which changes direction over
    multi-second timescales for a walking/circling target)."""

    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self.n = self.e = None

    def update(self, n, e):
        if self.n is None:
            self.n, self.e = n, e
        else:
            self.n += self.alpha * (n - self.n)
            self.e += self.alpha * (e - self.e)
        return self.n, self.e

    def reset(self):
        self.n = self.e = None


def follow_person(drone, get_dets, abort=None, alt=3.0, duration=None,
                  search_center=(2.6, 21.0)):
    """Hover directly above the person, tracking them as they move.

    A nadir camera only sees a small circle of ground directly below (at
    3 m altitude, roughly a 4.6 m radius) - nowhere near enough range to
    spot someone from across the arena. So this first flies to
    `search_center` (north, east metres - same convention as goto), an
    approximate starting area (sim default: the person's patrol zone; a
    real deployment would get this from some coarse cue - a GPS beacon,
    a wider preliminary scan, a handoff from another sensor).

    Once there, each frame re-estimates the person's NED position (same
    body-offset-to-NED conversion as dock_land) and streams it as a
    position target - not a velocity proportional to the raw camera
    angle. An earlier velocity-based version of this (and of dock_land)
    visibly wobbled: reacting to a noisy per-frame angle with a pure
    proportional controller and no damping overshoots and corrects
    repeatedly. Continuously retargeting a position - even one that
    moves each update - lets ArduPilot's own damped position controller
    do the smoothing, the same "follow me" pattern commercial drones use,
    rather than re-deriving PD damping by hand. If lost, it searches an
    outward spiral (velocity-driven, since there's no target position to
    aim at) - spinning in place would be useless here, since rotating a
    straight-down camera reveals no new ground, only translating does.
    Runs until aborted (or `duration` seconds).
    """
    if not drone.is_flying():
        drone.takeoff(alt, abort=abort)
    drone.set_mode('GUIDED')
    drone._say(f"  following: moving to search area {search_center}")
    drone.goto(search_center[0], search_center[1], alt, abort=abort)
    drone._say(f"  following: hovering above person, alt {alt} m")

    t0 = time.time()
    last_seen = time.time()
    search_angle = 0.0
    search_radius = 0.0
    smoother = _TargetSmoother()
    while duration is None or time.time() - t0 < duration:
        if abort is not None and abort():
            drone.send_body_velocity(0, 0, 0)
            raise CommandAborted("aborted")
        detections, age = get_dets()
        person = None
        if age < DET_MAX_AGE and detections.get('person'):
            person = detections['person'][0]

        if person:
            last_seen = time.time()
            search_radius = 0.0
            pos = drone.master.recv_match(type='LOCAL_POSITION_NED',
                                          blocking=True, timeout=2)
            att = drone.master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
            if pos is None or att is None:
                time.sleep(LOOP_DT)
                continue
            # image-up = body-forward, image-right = body-right (same
            # convention as dock_land's tag-centring, verified there)
            raw_n, raw_e = _body_offset_to_ned(
                pos, att.yaw, -person.elevation_rad * person.distance_m,
                person.bearing_rad * person.distance_m)
            target_n, target_e = smoother.update(raw_n, raw_e)
            drone.send_position_target(target_n, target_e, alt)
        elif time.time() - last_seen > 2.0:
            # outward spiral: slowly growing radius, constant angular step
            smoother.reset()   # don't blend a stale target once reacquired
            search_angle += 0.15
            search_radius = min(search_radius + 0.03, 8.0)
            vx = _clamp(search_radius * 0.15 * math.cos(search_angle), 1.0)
            vy = _clamp(search_radius * 0.15 * math.sin(search_angle), 1.0)
            drone.send_body_velocity(vx, vy, 0)
        else:
            drone.send_body_velocity(0, 0, 0)          # brief dropout: hold
        time.sleep(LOOP_DT)

    drone.send_body_velocity(0, 0, 0)
    if not drone.is_flying():
        raise CommandError("follow ended with the vehicle on the ground "
                           "(landed or crashed during the mission)")
    drone.hold()


def _body_offset_to_ned(pos, yaw, dx_body, dy_body):
    """Rotate a body-frame (forward, right) offset into a NED (north,
    east) target position, using the vehicle's current position and
    heading."""
    dn = dx_body * math.cos(yaw) - dy_body * math.sin(yaw)
    de = dx_body * math.sin(yaw) + dy_body * math.cos(yaw)
    return pos.x + dn, pos.y + de


def dock_land(drone, get_dets, abort=None, dock_ned=(-6.0, 6.0),
              approach_alt=6.0, timeout=180):
    """Fly to the dock area and precision-land on the AprilTag.

    The approach position comes from the mission plan (the dock is the
    drone's home base). The landing itself re-estimates the tag's
    position each frame and feeds it to ArduPilot's own position
    controller (not a hand-rolled velocity one) as an absolute NED
    target - since the dock doesn't move, "fly to where I currently
    think the tag is" converges smoothly the same way any other goto()
    in this project does, rather than continuously reacting to noisy
    per-frame camera angles the way a raw proportional velocity
    controller does (that was the old approach - it visibly wobbled).
    Altitude ratchets down only once well-centred, then hands over to
    LAND for the blind final metre.
    """
    if not drone.is_flying():
        raise CommandError("dock landing requires the vehicle to be flying")
    drone.set_mode('GUIDED')
    # climb vertically clear of obstacles FIRST, then translate: a diagonal
    # departure from low altitude near walls is how drones clip things
    msg = drone.master.recv_match(type='LOCAL_POSITION_NED',
                                  blocking=True, timeout=5)
    if msg is not None and -msg.z < approach_alt - 0.5:
        drone._say(f"  dock: climbing to {approach_alt} m before transit")
        drone.goto(msg.x, msg.y, approach_alt, abort=abort)
    drone._say("  dock: flying to approach position")
    drone.goto(dock_ned[0], dock_ned[1], approach_alt, abort=abort)

    lost_since = None
    last_lost_say = 0.0
    retries = 0
    target_alt = approach_alt
    smoother = _TargetSmoother()
    deadline = time.time() + timeout
    while time.time() < deadline:
        if abort is not None and abort():
            drone.send_body_velocity(0, 0, 0)
            raise CommandAborted("aborted")
        detections, age = get_dets()
        tag = None
        if age < DET_MAX_AGE and detections.get('apriltag'):
            tag = detections['apriltag'][0]

        if tag is None:
            if lost_since is None:
                lost_since = time.time()
            elif time.time() - lost_since > 2.5:
                if drone.altitude() > approach_alt + 3:
                    # climbed without reacquiring: go back to the approach
                    # point rather than climbing away forever
                    retries += 1
                    if retries > 3:
                        raise CommandError("dock: tag not found after "
                                           f"{retries} approaches")
                    drone._say(f"  dock: reapproaching (attempt {retries + 1})")
                    drone.goto(dock_ned[0], dock_ned[1], approach_alt,
                               abort=abort)
                    target_alt = approach_alt
                    smoother.reset()
                    lost_since = None
                else:
                    if time.time() - last_lost_say > 3.0:
                        drone._say(f"  dock: tag lost {time.time()-lost_since:.0f}s "
                                   f"(det age {age:.1f}s, "
                                   f"alt {drone.altitude():.1f} m) - climbing")
                        last_lost_say = time.time()
                    drone.send_body_velocity(0, 0, -0.4)
            else:
                drone.send_body_velocity(0, 0, 0)
            time.sleep(LOOP_DT)
            continue
        lost_since = None

        # camera points straight down, gimbal yaw 0: image-up = body-forward,
        # image-right = body-right (signs verified empirically via velocity
        # pulses - see the Jacobian probe in the project history)
        pos = drone.master.recv_match(type='LOCAL_POSITION_NED',
                                      blocking=True, timeout=2)
        att = drone.master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
        if pos is None or att is None:
            time.sleep(LOOP_DT)
            continue
        raw_n, raw_e = _body_offset_to_ned(
            pos, att.yaw, -tag.elevation_rad * tag.distance_m,
            tag.bearing_rad * tag.distance_m)
        target_n, target_e = smoother.update(raw_n, raw_e)
        centred = (abs(tag.bearing_rad) < 0.12 and abs(tag.elevation_rad) < 0.12)
        centred_tight = (abs(tag.bearing_rad) < 0.06
                         and abs(tag.elevation_rad) < 0.06)

        # the tag becomes undetectable closer than ~1.2 m (fills the frame),
        # so hand over to LAND at ~1.3 m while it is still tracked. Settle
        # at the fixed computed target first - residual drift carried into
        # the blind LAND phase is what drags the touchdown off-centre.
        if tag.distance_m < 1.3 and centred_tight:
            drone._say(f"  dock: {tag.distance_m:.1f} m over tag, centred - "
                       "settling")
            for _ in range(10):
                detections, age = get_dets()
                if age < DET_MAX_AGE and detections.get('apriltag'):
                    t = detections['apriltag'][0]
                    pos = drone.master.recv_match(type='LOCAL_POSITION_NED',
                                                  blocking=True, timeout=2)
                    att = drone.master.recv_match(type='ATTITUDE',
                                                  blocking=True, timeout=2)
                    if pos is not None and att is not None:
                        raw_n, raw_e = _body_offset_to_ned(
                            pos, att.yaw, -t.elevation_rad * t.distance_m,
                            t.bearing_rad * t.distance_m)
                        target_n, target_e = smoother.update(raw_n, raw_e)
                drone.send_position_target(target_n, target_e, target_alt)
                time.sleep(0.2)
            drone._say("  dock: final descent")
            drone.land()
            return

        # only ratchet the target altitude down once reasonably centred -
        # descending while still off-target just means descending off-axis
        if centred:
            step = 0.5 if tag.distance_m > 3.0 else 0.3
            target_alt = max(target_alt - step, 1.3)
        drone.send_position_target(target_n, target_e, target_alt)
        time.sleep(LOOP_DT)

    raise CommandError(f"dock landing did not complete within {timeout}s")
