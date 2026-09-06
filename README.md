# Autonomous Tracking Drone Project

A drone simulation and ground-control stack on macOS (Apple Silicon):

- **ArduPilot SITL** — the real ArduCopter flight controller firmware, compiled for the host and run as a software-in-the-loop simulation
- **Gazebo Harmonic** — 3D physics and visual simulation of a custom quadcopter (with a fixed nadir camera) flying on a runway
- **Custom ground station** — a Python (pygame) GUI with live telemetry, a moving map, preprogrammed flight paths, and the drone's camera feed decoded over RTP

```
├── src/                    ground-station + autonomy source
│   ├── drone_control.py    pymavlink flight-control library + CLI
│   ├── drone_gui.py        pygame ground station
│   ├── vision.py           camera pipeline + detectors (tag/person/obstacle)
│   ├── missions.py         autonomy: follow person, dock landing, collision guard
│   ├── telemetry_check.py  link/telemetry health check
│   └── camera_stream.sdp   SDP descriptor for the camera RTP stream
├── sim/                    project-owned Gazebo assets
│   ├── worlds/tracking_arena.sdf   dock + walls + obstacles + moving person
│   └── models/landing_dock/        platform with AprilTag 36h11 id 0
├── third_party/            dependencies (git submodules)
│   ├── ardupilot/          flight controller firmware + SITL tooling
│   └── ardupilot_gazebo/   Gazebo <-> ArduPilot bridge plugin + models/worlds
├── patches/                local patches applied to submodule working trees
├── requirements.txt        Python dependencies (venv)
└── venv/                   Python virtualenv (created during setup, not committed)
```

## Setup from scratch

### 1. Clone with submodules

```bash
git clone --recurse-submodules https://github.com/tensofu/autonomous-tracking-drone.git
cd autonomous-tracking-drone
```

All commands below are run from this directory unless stated otherwise.

### 2. System dependencies (Homebrew)

```bash
brew tap osrf/simulation
brew trust osrf/simulation          # Homebrew >= 6 requires trusting the tap
brew install gz-harmonic rapidjson opencv gstreamer qt@5 cmake ccache
```

### 3. Python environment

Requires Python 3.11+ (tested on 3.14).

```bash
python3 -m venv venv
./venv/bin/pip install -r requirements.txt
mkdir -p ~/.mavproxy                # works around a MAVProxy first-run quirk
```

### 4. Shell environment (`~/.zshrc`)

Add the following, with the first line adjusted to wherever you cloned the
repository:

```bash
export DRONE_SIM_HOME="$HOME/autonomous-tracking-drone"   # <-- your clone path

# gz-transport must be pinned to loopback: VPN interfaces (Tailscale etc.)
# otherwise break Gazebo server<->GUI comms (blank or stale GUI windows)
export GZ_IP=127.0.0.1
export GZ_SIM_SYSTEM_PLUGIN_PATH="$DRONE_SIM_HOME/third_party/ardupilot_gazebo/build"
export GZ_SIM_RESOURCE_PATH="$DRONE_SIM_HOME/third_party/ardupilot_gazebo/models:$DRONE_SIM_HOME/third_party/ardupilot_gazebo/worlds"
```

Then open a new terminal (or `source ~/.zshrc`).

> **Note:** if your shell config sets `CPLUS_INCLUDE_PATH` anywhere, make sure
> it never ends with a trailing colon — an empty entry means "current
> directory" as a *system* include path, which silently shadows generated
> headers and breaks the ArduPilot build (`AP_BUILD_ROOT` undeclared in
> `SIM_AIS.cpp`). Safe form:
>
> ```bash
> export CPLUS_INCLUDE_PATH="/opt/homebrew/opt/llvm/include${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"
> ```

### 5. Build ArduCopter SITL

```bash
source venv/bin/activate
cd third_party/ardupilot
./waf configure --board sitl
./waf copter
cd ../..
```

### 6. Build the Gazebo plugin

```bash
cd third_party/ardupilot_gazebo
git apply ../../patches/iris-customizations.patch   # recolor + custom body, see below
mkdir build && cd build
GZ_VERSION=harmonic cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_PREFIX_PATH=/opt/homebrew/opt/qt@5
make -j8
cd ../../..
```

(`qt@5` is keg-only, hence the explicit prefix path. OpenCV and GStreamer are
hard requirements of the plugin's CMakeLists despite looking optional.)

## Running the simulation

Four terminals (or background the first two), all with the environment from
setup step 4:

```bash
# 1. Gazebo server (physics; macOS cannot run server+GUI in one process)
gz sim -v4 -s -r tracking_arena.sdf      # or iris_runway.sdf for a bare runway

# 2. Gazebo GUI (3D view; start after the server)
gz sim -g

# 3. ArduPilot SITL + MAVProxy (from the repository root)
source venv/bin/activate
cd third_party/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map

# 4. Ground station GUI (from the repository root)
source venv/bin/activate
python src/drone_gui.py
```

Wait until the MAVProxy console stops printing `PreArm:` messages (the EKF
needs ~30-60 s to settle after boot), then fly from the GUI, or from MAVProxy:
`mode guided` → `arm throttle` → `takeoff 10`.

## The tools

### `src/drone_gui.py` — ground station

- Live telemetry, artificial horizon, compass, north-up moving map with a
  flight trail (CLEAR button resets it)
- Preprogrammed flight paths (square, triangle, circle, figure-eight, spiral,
  survey) — click to fly; HOLD / LAND / RTL / HOME interrupt a running path
- HOME flies back to the takeoff origin and hovers; RTL returns *and lands*
- Camera panel showing the drone's fixed nadir camera (see below); COPY
  button puts the log on the clipboard, full history in `gcs.log`
- Uses two MAVLink connections so the display never freezes: telemetry on
  `udpin:127.0.0.1:14550` (MAVProxy's output), commands on `tcp:127.0.0.1:5762`
  (SITL's spare serial port). Both reconnect automatically across sim restarts.

### `src/drone_control.py` — flight-control library + CLI

```bash
python src/drone_control.py status | arm | disarm | takeoff 10 | goto 20 5 10 \
    | hold | land | rtl | home | demo
```

Importable: `DroneController` exposes the same operations with ACK checking,
pre-flight guards, and abort callbacks for interruptible missions.

### `src/telemetry_check.py` — link health check

Connects, reads altitude/attitude/battery, and proves two-way communication
via parameter and command round trips. Exits nonzero on failure (CI-friendly).

## Custom drone body (flightstack.STL)

The sim's quadcopter body has been swapped from the stock iris frame to an
imported CAD part (`models/flightstack.STL`, copied into
`sim/models/flightstack/`). This is a **visual + collision replacement
only** - the rotor positions, motor mapping, mass, and inertia are all
untouched from the original iris, so the flight dynamics are exactly as
tested before. Patched into the `ardupilot_gazebo` submodule via
`patches/iris-customizations.patch` (also carries the earlier orange
recolor); apply it during setup step 6 above.

**v2 update:** re-exported Z-up with baked-in arms and propeller blades
(v1 had neither - see git history). Since the mesh's own bounding box was
skewed by an off-center underside bump, placement was derived by
*measuring the mesh's actual geometry*: clustering triangle centroids to
find the 4 propeller-blade hubs, whose mean gives the true rotational
center. The old separate spinning propeller visuals (generic iris
`iris_prop_ccw/cw.dae` meshes) were removed from the rotor links, since
the STL's own baked-in blades would otherwise double-render on top of
them - propellers no longer visually spin, though physics/thrust are
unaffected (the collision discs and joints are still there, just
invisible).

**v3 update:** the STL reverted to Y-up again (same fix applied: +90 deg
roll) and now includes its own baked-in landing-gear legs. The placeholder
cylinder legs (kept from the original iris frame in v2) are removed - the
STL's own 4 legs, confirmed via clustering (distinct clusters near the
mesh's bottom, symmetric about the same rotational center as the
propellers), now provide the actual ground contact. Body placement
convention changed accordingly: link origin is anchored so the legs' own
tips sit at exactly link Z=0 (rather than v2's rotational-center-height
origin), so world spawn height dropped from 0.195 to 0.01 m to match.
Rotor mount points (107.95mm / 143.55mm half-spans, 106.35mm elevation)
came directly from the user's own CAD measurements this time rather than
mesh-clustering estimates - independently cross-checked against this
session's clustering (agreed within ~1mm on the horizontal offsets).
Verified: settles at 0.0000 degrees tilt (better than v2), hover attitude
noise <0.1 degrees, camera view confirmed unobstructed by the legs.

**v3.1 update: real mass and inertia.** Mass is the user's measured 1062 g
(down from the iris placeholder's 1.5 kg). Rather than asking for a CAD
mass-properties export (every tool has different sign/unit conventions for
products of inertia - an easy source of silent errors), the full inertia
tensor is computed directly from the mesh, assuming uniform density
(the only reasonable option without per-component material/placement
data - a real battery or motor is much denser than the surrounding
enclosure, so this is an approximation). The computation - closed-form
tetrahedra-from-origin integration, standard for polyhedral mass
properties - was validated against an analytic solid box (volume, CoM,
and all three principal moments matched exactly) and checked for
translation-invariance before trusting it on the real mesh. Off-diagonal
terms came out three orders of magnitude smaller than the diagonal ones,
consistent with the body being nearly symmetric about the link's axes.
Verified: still settles at 0.0000 degrees, hover unaffected by the mass
change.

**What real hardware data would still improve:**
1. **True mass distribution** - the uniform-density assumption above is
   the main remaining gap; a battery or motors concentrate mass in ways
   a uniform shell can't capture. Only matters if precise dynamics
   (not just "flies stably like the real thing") become important.
2. **Fixed camera mount height** - currently an estimate (3 cm above the
   leg-tip/ground reference, chosen empirically to clear the legs without
   embedding in the mesh); should be set to wherever the camera actually
   mounts on the real stack.
3. **Spinning propellers** - if animated props matter more than exact
   visual fidelity to the real part, the old separate spinning-prop
   visuals could be restored (repositioned to the current 107.95/143.55mm
   mount points) instead of relying on the STL's static baked-in blades.

## Camera feed and vision

The drone carries a single **fixed nadir (straight-down) camera**, mounted
centered under the body on `iris_with_standoffs::base_link` - there is no
gimbal (one was tried and removed; see git history / `patches/` for that
version). It uses ardupilot_gazebo's `GstCameraPlugin`: when enabled (the
GUI does it automatically by publishing to the camera's `enable_streaming`
topic), Gazebo encodes H.264 and streams RTP to `udp://127.0.0.1:5600`.
OpenCV's bundled FFmpeg decodes it via `src/camera_stream.sdp`.

`src/vision.py` runs three detectors on every frame and draws the results
into the GUI's camera panel:

- **apriltag** - the dock fiducial (36h11 id 0), via OpenCV's aruco module
- **person** - the red-cylinder stand-in, via HSV color segmentation.
  Distance is estimated from apparent *width* (the diameter seen from
  directly above), not height - a nadir view never sees how tall something
  is.
- **obstacle** - hazard-yellow walls/objects, via color + geometry

The detectors return typed `Detection` objects (class, box, bearing,
elevation, distance estimate). **This is the hardware seam**: on the real
drone a Raspberry Pi AI Camera (IMX500) produces the same information from
onboard neural networks - swap the detector internals and the missions run
unchanged. `FrameSource` deliberately keeps only the newest frame: letting
decoded frames queue makes the autonomy chase seconds-old imagery.

## Autonomy missions (`src/missions.py`)

Run from the GUI (FOLLOW / DOCK buttons) or scripted. Both missions are
driven entirely by the fixed nadir camera - there's no gimbal to point.

**Position-targeted, not velocity-controlled.** Both missions re-estimate
the target's NED position every frame and feed it to ArduPilot's own
position controller as an absolute target (`DroneController.
send_position_target`), rather than commanding raw velocity proportional
to the instantaneous camera angle. An earlier velocity-based version of
both missions visibly wobbled - a classic un-damped P-controller symptom:
reacting to a noisy per-frame angle with no damping overshoots and
corrects repeatedly. Continuously retargeting a position (even one that
moves, for the follow mission) lets ArduPilot's already-tuned position
controller do the smoothing, the same "follow me" pattern commercial
drones use, instead of hand-deriving PD damping. On top of that, each
target position is exponentially smoothed (`_TargetSmoother`, alpha=0.2)
before being sent - individual vision readings have pixel-level noise,
and feeding every raw reading straight into the controller chases that
noise (measured directly: attitude standard deviation during tracking
dropped from ~4-5 degrees to ~2 degrees after adding the filter, with no
loss of tracking accuracy or touchdown precision).

- **follow_person** - a nadir camera only sees a small patch of ground
  directly below (roughly a 4.6 m radius at 3 m altitude) - nowhere near
  enough range to spot someone from across the arena. So this first flies
  to `search_center` (an approximate starting area; the sim default
  matches the person's patrol zone - a real deployment would get this from
  some coarse cue like a GPS beacon or a wider preliminary scan), then
  hovers directly above the person once detected. If lost, it searches an
  outward spiral - spinning in place is useless for a straight-down
  camera, since rotating reveals no new ground, only translating does.
  Verified: tracks within 2-3.5 m of the continuously-circling person,
  attitude noise ~2 degrees standard deviation.
- **dock_land** - climbs vertically, transits to the dock area, then homes
  onto the AprilTag (already looking straight down - no gimbal command
  needed). Descent altitude only ratchets down once well-centred, then
  settles at the final computed position (~1.3 m, where the tag becomes
  undetectable as it fills the frame) before handing over to LAND for the
  blind final metre. Verified over repeated runs: direct convergence with
  no back-and-forth correction, touchdowns consistently within 2 cm of
  tag centre.

**No collision guard.** An earlier version had one, using a horizon-line
heuristic (an obstacle whose box rises above where the drone's own altitude
projects in frame is in the flight path). That only works for a
forward/level camera. A nadir camera has no equivalent - it sees the *tops*
of nearby objects, which alone doesn't reveal their height relative to the
drone. Real obstacle avoidance with this camera layout needs a separate
forward-facing sensor (a second camera, or a rangefinder) - not solved
here.

## Troubleshooting

| Symptom | Cause / fix |
| --- | --- |
| Build fails: `AP_BUILD_ROOT` undeclared | `CPLUS_INCLUDE_PATH` ends with a colon — see the note in setup step 4 |
| Gazebo GUI window transparent/blank | `GZ_IP=127.0.0.1` missing (VPN interfaces break gz-transport); set it for **both** server and GUI |
| GUI shows drone parked while telemetry says flying (or vice versa) | Same `GZ_IP` issue: a half-connected GUI renders stale poses. Restart the GUI with the env set. Trust `gz topic -e -t /world/iris_runway/dynamic_pose/info -n 1` over the GUI |
| `PreArm: Motors: Check frame class and type` after a `-w` wipe | `--model JSON` bypasses frame default params in current ArduPilot. `param set FRAME_CLASS 1` + `reboot`, or wipe with `--add-param-file Tools/autotest/default_params/copter.parm --add-param-file Tools/autotest/default_params/gazebo-iris.parm` |
| pygame crashes: "SDL downgrade" | A global `DYLD_LIBRARY_PATH` (e.g. from a Vulkan SDK install) injects an old libSDL2; `drone_gui.py` strips it and re-execs itself automatically |
| Ground station says links lost forever | Another GUI instance holds the ports (one `udpin:14550` bind, one `tcp:5762` client). Kill duplicates |
| Copter descends in LOITER with no RC | Pilot modes follow the RC throttle, which reads low with no input. Use GUIDED (the GUI's HOLD does) |
| `sim_vehicle.py` behaves oddly | Bare `sim_vehicle.py` may resolve to a different ArduPilot checkout if one is on your PATH; always run `./Tools/autotest/sim_vehicle.py` from `third_party/ardupilot` |
| Drone flies to the wrong spot / mirrored position | `goto()` and mission position args are `(north, east, alt)` (NED); Gazebo's own world topics report `(x=east, y=north)`. Mixing these up sends the drone to the transposed position - easy to do when cross-checking against `gz topic` output |
| SITL prints "No JSON sensor message received" forever | A Gazebo world reset (`gz service .../control reset`) breaks the SITL lockstep. Restart the Gazebo server AND SITL together |
| `PreArm: Rangefinder 1: Not Detected` | Don't enable `RNGFND1_TYPE` with `--model JSON` - the JSON backend supplies no rangefinder data for this model. `param set RNGFND1_TYPE 0` |
| Dock landing misses / drifts | The camera loses the tag under ~1.2 m; the mission must settle centred before its blind final descent (already implemented in `dock_land`) |
