# DroneProject

A complete drone simulation and ground-control stack on macOS (Apple Silicon):

- **ArduPilot SITL** — the real ArduCopter flight controller firmware, compiled
  for the host and run as a software-in-the-loop simulation
- **Gazebo Harmonic** — 3D physics and visual simulation of an iris quadcopter
  (with gimbal camera) flying on a runway
- **Custom ground station** — a pygame GUI with live telemetry, a moving map,
  preprogrammed flight paths, and the drone's camera feed decoded over RTP

```
├── src/                    ground-station source
│   ├── drone_control.py    pymavlink flight-control library + CLI
│   ├── drone_gui.py        pygame ground station
│   ├── telemetry_check.py  link/telemetry health check
│   └── camera_stream.sdp   SDP descriptor for the camera RTP stream
├── third_party/            dependencies (git submodules)
│   ├── ardupilot/          flight controller firmware + SITL tooling
│   └── ardupilot_gazebo/   Gazebo <-> ArduPilot bridge plugin + models/worlds
├── patches/                local patches applied to submodule working trees
├── requirements.txt        Python dependencies (venv)
└── venv/                   Python 3.14 virtualenv (not committed)
```

## Setup from scratch

### 1. Clone with submodules

```bash
git clone --recurse-submodules <this-repo>
cd DroneProject
```

### 2. System dependencies (Homebrew)

```bash
brew tap osrf/simulation
brew trust osrf/simulation          # Homebrew >= 6 requires trusting the tap
brew install gz-harmonic rapidjson opencv gstreamer qt@5 cmake ccache
```

### 3. Python environment

```bash
python3 -m venv venv
./venv/bin/pip install -r requirements.txt
mkdir -p ~/.mavproxy                # first-run quirk of mavproxy --version
```

### 4. Shell environment (`~/.zshrc`)

```bash
# gz-transport must be pinned to loopback: VPN interfaces (Tailscale etc.)
# otherwise break Gazebo server<->GUI comms (blank or stale GUI windows)
export GZ_IP=127.0.0.1
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/CodingProjects/DroneProject/third_party/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$HOME/CodingProjects/DroneProject/third_party/ardupilot_gazebo/models:$HOME/CodingProjects/DroneProject/third_party/ardupilot_gazebo/worlds
```

If you set `CPLUS_INCLUDE_PATH` anywhere, make sure it never ends with a
trailing colon — an empty entry means "current directory" as a *system*
include path, which silently shadows generated headers and breaks the
ArduPilot build (`AP_BUILD_ROOT` undeclared in `SIM_AIS.cpp`). Safe form:

```bash
export CPLUS_INCLUDE_PATH="/opt/homebrew/opt/llvm/include${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"
```

### 5. Build ArduCopter SITL

```bash
cd third_party/ardupilot
../../venv/bin/python -m pip --version   # sanity: venv exists
source ../../venv/bin/activate
./waf configure --board sitl
./waf copter
```

### 6. Build the Gazebo plugin

```bash
cd third_party/ardupilot_gazebo
git apply ../../patches/iris-orange-visibility.patch   # optional: orange drone
mkdir build && cd build
GZ_VERSION=harmonic cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_PREFIX_PATH=/opt/homebrew/opt/qt@5
make -j8
```

(`qt@5` is keg-only, hence the explicit prefix path. OpenCV and GStreamer are
hard requirements of the plugin's CMakeLists despite looking optional.)

## Running the simulation

Four terminals (or background the first two):

```bash
# 1. Gazebo server (physics; macOS cannot run server+GUI in one process)
gz sim -v4 -s -r iris_runway.sdf

# 2. Gazebo GUI (3D view; start after the server)
gz sim -g

# 3. ArduPilot SITL + MAVProxy
cd third_party/ardupilot && source ../../venv/bin/activate
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map

# 4. Ground station GUI
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
- Camera panel showing the drone's gimbal camera (see below); COPY button
  puts the log on the clipboard, full history in `gcs.log`
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

## Camera feed

The `iris_with_gimbal` model carries a camera with ardupilot_gazebo's
`GstCameraPlugin`: when enabled (the GUI does it automatically by publishing
to the camera's `enable_streaming` topic), Gazebo encodes H.264 and streams
RTP to `udp://127.0.0.1:5600`. OpenCV's bundled FFmpeg decodes it via
`src/camera_stream.sdp`; frames land in the GUI's camera panel with a frame
counter. Hook detection code into `CameraFeed._pump()` in `drone_gui.py`.

## Troubleshooting (hard-won)

| Symptom | Cause / fix |
| --- | --- |
| Build fails: `AP_BUILD_ROOT` undeclared | `CPLUS_INCLUDE_PATH` ends with a colon — see setup step 4 |
| Gazebo GUI window transparent/blank | `GZ_IP=127.0.0.1` missing (VPN interfaces break gz-transport); set it for **both** server and GUI |
| GUI shows drone parked while telemetry says flying (or vice versa) | Same `GZ_IP` issue: a half-connected GUI renders stale poses. Restart the GUI with the env set. Trust `gz topic -e -t /world/iris_runway/dynamic_pose/info -n 1` over the GUI |
| `PreArm: Motors: Check frame class and type` after `-w` wipe | `--model JSON` bypasses frame default params in this ArduPilot version. `param set FRAME_CLASS 1` + `reboot`, or wipe with `--add-param-file Tools/autotest/default_params/copter.parm --add-param-file Tools/autotest/default_params/gazebo-iris.parm` |
| pygame crashes: "SDL downgrade" | A global `DYLD_LIBRARY_PATH` (e.g. Vulkan SDK) injects an old libSDL2; `drone_gui.py` strips it and re-execs itself automatically |
| Ground station says links lost forever | Another GUI instance holds the ports (one `udpin:14550` bind, one `tcp:5762` client). Kill duplicates |
| Copter descends in LOITER with no RC | Pilot modes follow the RC throttle, which reads low with no input. Use GUIDED (the GUI's HOLD does) |
| `sim_vehicle.py` behaves oddly | Bare `sim_vehicle.py` may resolve to another ArduPilot checkout on PATH; always run `./Tools/autotest/sim_vehicle.py` from `third_party/ardupilot` |
