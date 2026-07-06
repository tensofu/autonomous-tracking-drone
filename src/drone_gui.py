#!/usr/bin/env python3
"""Pygame ground station for the SITL copter.

    python drone_gui.py
    python drone_gui.py --telemetry udpin:127.0.0.1:14550 --command tcp:127.0.0.1:5762

Layout:
  - left   : live telemetry, attitude indicator, compass
  - centre : top-down moving map with flight trail, preprogrammed path list
  - right  : camera panel (reserved for a future feed), action buttons, log

Uses two MAVLink connections so the display never freezes while a command
runs: telemetry listens on MAVProxy's UDP output, commands go directly to
SITL's spare TCP port. Both work at the same time as the MAVProxy console.

Camera attachment later: call App.camera.set_frame(surface) with any pygame
Surface (e.g. decoded from a GStreamer/OpenCV image) and it will be scaled
into the reserved panel. Until then the panel shows a placeholder.
"""

import argparse
import math
import os
import queue
import subprocess
import sys
import threading
import time

# The user's shell exports DYLD_LIBRARY_PATH for the Vulkan SDK, which
# bundles an older libSDL2 that shadows pygame's own and crashes it on
# import ("Dynamic linking causes SDL downgrade"). Re-exec without it.
if os.environ.get("DYLD_LIBRARY_PATH"):
    env = {k: v for k, v in os.environ.items() if k != "DYLD_LIBRARY_PATH"}
    os.execve(sys.executable, [sys.executable] + sys.argv, env)

# camera feed: let OpenCV's FFmpeg accept the RTP-over-UDP stream described
# by the .sdp file, and silence its join-time decoder chatter
os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS",
                      "protocol_whitelist;file,rtp,udp")
os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "-8")
try:
    import cv2
except ImportError:
    cv2 = None

import pygame
from pymavlink import mavutil

from drone_control import DroneController, CommandError, CommandAborted

# ----------------------------------------------------------------- config --

SIZE = (1280, 800)
FPS = 30

BG = (18, 18, 18)
PANEL = (30, 30, 30)
PANEL_EDGE = (58, 58, 58)
TEXT = (228, 228, 228)
DIM = (148, 148, 148)
ACCENT = (190, 190, 190)          # primary buttons / highlights (neutral)
GOOD = (80, 200, 120)             # status colors stay semantic
WARN = (240, 180, 60)
BAD = (235, 90, 90)

# preprogrammed paths: (name, default altitude, [(north, east, alt), ...])


def _circle(radius, alt, points=12, laps=1):
    return [(radius * math.cos(2 * math.pi * i / points) - radius,
             radius * math.sin(2 * math.pi * i / points), alt)
            for i in range(1, points * laps + 1)]


def _spiral(radius, alt_from, alt_to, points=16):
    return [(radius * math.cos(2 * math.pi * i / points) - radius,
             radius * math.sin(2 * math.pi * i / points),
             alt_from + (alt_to - alt_from) * i / points)
            for i in range(1, points + 1)]


# gimbal camera on the iris_with_gimbal model in the iris_runway world
CAMERA_ENABLE_TOPIC = ("/world/iris_runway/model/iris_with_gimbal/model/"
                       "gimbal/link/pitch_link/sensor/camera/image/"
                       "enable_streaming")
SDP_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "camera_stream.sdp")
SDP_BODY = "c=IN IP4 127.0.0.1\nm=video 5600 RTP/AVP 96\na=rtpmap:96 H264/90000\n"

FLIGHT_PATHS = [
    ("Square 20 m", 10, [(20, 0, 10), (20, 20, 10), (0, 20, 10), (0, 0, 10)]),
    ("Triangle", 10, [(24, 0, 10), (12, 20, 10), (0, 0, 10)]),
    ("Circle r=15 m", 10, _circle(15, 10)),
    ("Figure eight", 12, _circle(10, 12) + _circle(-10, 12)[::-1]),
    ("Ascending spiral", 5, _spiral(12, 5, 18) + [(0, 0, 18)]),
    ("Survey lawnmower", 8, [(30, 0, 8), (30, 8, 8), (0, 8, 8), (0, 16, 8),
                             (30, 16, 8), (30, 24, 8), (0, 24, 8), (0, 0, 8)]),
]


# -------------------------------------------------------------- telemetry --

class Telemetry(threading.Thread):
    """Listens on its own MAVLink connection and keeps the latest state."""

    def __init__(self, connection_string, log):
        super().__init__(daemon=True)
        self.connection_string = connection_string
        self.log = log
        self.ok = False
        self.mode = '?'
        self.armed = False
        self.alt = 0.0
        self.lat = self.lon = 0.0
        self.heading = 0.0
        self.groundspeed = 0.0
        self.climb = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.voltage = 0.0
        self.battery_pct = -1
        self.ned = (0.0, 0.0, 0.0)
        self.trail = []            # [(north, east)] for the map
        self._lock = threading.Lock()
        self._warned = False

    def run(self):
        while True:                       # reconnect forever (SITL restarts)
            try:
                master = mavutil.mavlink_connection(self.connection_string)
                if master.wait_heartbeat(timeout=10) is None:
                    master.close()
                    continue
            except OSError as exc:
                if not self._warned:
                    self.log.put(f"telemetry can't connect: {exc} "
                                 "(another GUI instance holding the port?)")
                    self._warned = True
                time.sleep(3)
                continue
            self._warned = False
            master.mav.request_data_stream_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 8, 1)
            self.log.put("telemetry link up")
            self._listen(master)
            self.ok = False
            try:
                master.close()
            except OSError:
                pass
            self.log.put("telemetry link lost - reconnecting ...")

    def _listen(self, master):
        misses = 0
        while True:
            try:
                msg = master.recv_match(blocking=True, timeout=2)
            except Exception:
                return                     # socket died
            if msg is None:
                self.ok = False
                misses += 1
                if misses >= 5:
                    return                 # silent too long: reconnect
                continue
            misses = 0
            self.ok = True
            kind = msg.get_type()
            if kind == 'HEARTBEAT':
                if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                    continue          # MAVProxy's own heartbeat, not the drone
                self.mode = mavutil.mode_string_v10(msg)
                self.armed = bool(msg.base_mode
                                  & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif kind == 'GLOBAL_POSITION_INT':
                self.alt = msg.relative_alt / 1000.0
                self.lat, self.lon = msg.lat / 1e7, msg.lon / 1e7
                self.heading = msg.hdg / 100.0
                self.groundspeed = math.hypot(msg.vx, msg.vy) / 100.0
                self.climb = -msg.vz / 100.0
            elif kind == 'ATTITUDE':
                self.roll, self.pitch, self.yaw = msg.roll, msg.pitch, msg.yaw
            elif kind == 'SYS_STATUS':
                self.voltage = msg.voltage_battery / 1000.0
                self.battery_pct = msg.battery_remaining
            elif kind == 'LOCAL_POSITION_NED':
                self.ned = (msg.x, msg.y, -msg.z)
                with self._lock:
                    if (not self.trail
                            or math.hypot(msg.x - self.trail[-1][0],
                                          msg.y - self.trail[-1][1]) > 0.5):
                        self.trail.append((msg.x, msg.y))
                        del self.trail[:-2000]
            elif kind == 'STATUSTEXT':
                self.log.put(f"AP: {msg.text}")

    def trail_copy(self):
        with self._lock:
            return list(self.trail)

    def clear_trail(self):
        with self._lock:
            self.trail.clear()


# ---------------------------------------------------------------- worker --

class CommandWorker(threading.Thread):
    """Owns the command connection; executes one action at a time."""

    def __init__(self, connection_string, log):
        super().__init__(daemon=True)
        self.connection_string = connection_string
        self.log = log
        self.actions = queue.Queue()
        self.abort_event = threading.Event()
        self.busy = ""             # label of the running action, "" if idle
        self.progress = ""
        self.connected = False

    def submit(self, label, fn, interrupts=False):
        if interrupts:
            self.abort_event.set()
            while not self.actions.empty():      # drop queued-up actions
                try:
                    self.actions.get_nowait()
                except queue.Empty:
                    break
        self.actions.put((label, fn))

    def run(self):
        while True:                       # reconnect forever (SITL restarts)
            try:
                drone = DroneController(
                    self.connection_string,
                    on_status=self.log.put,
                    on_progress=lambda s: setattr(self, 'progress', s.strip()))
            except (CommandError, OSError) as exc:
                self.log.put(f"command link down ({exc}); retrying in 5 s")
                time.sleep(5)
                continue
            self.connected = True
            self._drop_queued()           # stale clicks from while offline
            try:
                self._serve(drone)
            except Exception as exc:      # socket died mid-flight
                self.log.put("command link lost "
                             f"({exc.__class__.__name__}) - reconnecting ...")
            self.connected = False
            self.busy, self.progress = "", ""

    def _drop_queued(self):
        while not self.actions.empty():
            try:
                self.actions.get_nowait()
            except queue.Empty:
                break

    def _serve(self, drone):
        while True:
            try:
                label, fn = self.actions.get(timeout=0.5)
            except queue.Empty:
                drone.drain()      # keep idle socket empty: reads stay current
                continue
            self.abort_event.clear()
            self.busy, self.progress = label, ""
            try:
                drone.drain()
                fn(drone, self.abort_event.is_set)
            except CommandAborted:
                self.log.put(f"{label}: aborted")
            except CommandError as exc:
                self.log.put(f"{label} FAILED: {exc}")
            finally:
                self.busy, self.progress = "", ""


class CameraFeed(threading.Thread):
    """Receives the Gazebo camera's H.264/RTP stream via OpenCV and hands
    decoded frames to the CameraPanel. Enables streaming on the Gazebo side
    itself, and reconnects if the stream (or the sim) goes away."""

    def __init__(self, panel, log):
        super().__init__(daemon=True)
        self.panel = panel
        self.log = log

    def _enable_streaming(self):
        env = {**os.environ, "GZ_IP": "127.0.0.1"}
        try:
            subprocess.run(["gz", "topic", "-t", CAMERA_ENABLE_TOPIC,
                            "-m", "gz.msgs.Boolean", "-p", "data: true"],
                           env=env, capture_output=True, timeout=10)
        except (OSError, subprocess.TimeoutExpired):
            pass                    # sim not up yet; the retry loop handles it

    def run(self):
        if cv2 is None:
            self.log.put("camera: opencv-python not installed - feed disabled")
            return
        if not os.path.exists(SDP_FILE):
            with open(SDP_FILE, "w") as f:
                f.write(SDP_BODY)
        announced = False
        while True:
            self._enable_streaming()
            cap = cv2.VideoCapture(SDP_FILE, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                cap.release()
                if not announced:
                    self.log.put("camera: waiting for stream ...")
                    announced = True
                time.sleep(3)
                continue
            announced = False
            self.log.put("camera: stream opened")
            self._pump(cap)
            cap.release()
            self.panel.set_frame(None)
            self.log.put("camera: stream lost - reconnecting ...")

    def _pump(self, cap):
        frames = 0
        fps, counted, window_start = 0.0, 0, time.time()
        while True:
            ok, frame = cap.read()
            if not ok:
                return
            frames += 1
            counted += 1
            now = time.time()
            if now - window_start >= 1.0:
                fps = counted / (now - window_start)
                counted, window_start = 0, now
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            surface = pygame.image.frombuffer(
                rgb.tobytes(), (frame.shape[1], frame.shape[0]), "RGB")
            self.panel.set_frame(
                surface.copy(),
                f"{frame.shape[1]}x{frame.shape[0]}  "
                f"{fps:4.1f} fps  frame {frames}")


def fly_path(name, takeoff_alt, waypoints):
    def action(drone, aborted):
        if not drone.is_flying():
            drone.takeoff(takeoff_alt, abort=aborted)
        for north, east, alt in waypoints:
            drone.goto(north, east, alt, abort=aborted)
        drone.hold()
    return action


# ------------------------------------------------------------ UI widgets --

class Button:
    def __init__(self, rect, label, on_click, color=ACCENT):
        self.rect = pygame.Rect(rect)
        self.label = label
        self.on_click = on_click
        self.color = color

    def draw(self, surf, font, enabled=True):
        color = self.color if enabled else PANEL_EDGE
        pygame.draw.rect(surf, color, self.rect, border_radius=6)
        text = font.render(self.label, True, (12, 14, 18) if enabled else DIM)
        surf.blit(text, text.get_rect(center=self.rect.center))

    def handle(self, pos):
        if self.rect.collidepoint(pos):
            self.on_click()
            return True
        return False


class CameraPanel:
    """Reserved display area. Later: call set_frame() with a pygame Surface
    built from your camera stream and it will be shown scaled to fit."""

    def __init__(self, rect):
        self.rect = pygame.Rect(rect)
        self._frame = None
        self._caption = ""
        self._lock = threading.Lock()

    def set_frame(self, surface, caption=""):
        with self._lock:
            self._frame = surface
            self._caption = caption

    def draw(self, surf, font, small):
        pygame.draw.rect(surf, (10, 11, 14), self.rect, border_radius=8)
        pygame.draw.rect(surf, PANEL_EDGE, self.rect, 1, border_radius=8)
        with self._lock:
            frame, caption = self._frame, self._caption
        if frame is not None:
            scaled = pygame.transform.smoothscale(frame, self.rect.size)
            surf.blit(scaled, self.rect)
            if caption:
                strip = pygame.Rect(self.rect.left, self.rect.bottom - 20,
                                    self.rect.width, 20)
                shade = pygame.Surface(strip.size, pygame.SRCALPHA)
                shade.fill((0, 0, 0, 150))
                surf.blit(shade, strip)
                surf.blit(small.render(caption, True, TEXT),
                          (strip.left + 8, strip.top + 4))
            pygame.draw.rect(surf, PANEL_EDGE, self.rect, 1, border_radius=8)
            return
        cx, cy = self.rect.center
        pygame.draw.line(surf, PANEL_EDGE, (cx - 12, cy), (cx + 12, cy))
        pygame.draw.line(surf, PANEL_EDGE, (cx, cy - 12), (cx, cy + 12))
        pygame.draw.circle(surf, PANEL_EDGE, (cx, cy), 24, 1)
        label = font.render("CAMERA FEED", True, DIM)
        sub = small.render("reserved - App.camera.set_frame(surface)", True, PANEL_EDGE)
        surf.blit(label, label.get_rect(center=(cx, cy - 48)))
        surf.blit(sub, sub.get_rect(center=(cx, cy + 48)))


# ------------------------------------------------------------------- app --

class App:

    def __init__(self, telemetry_conn, command_conn):
        pygame.init()
        pygame.display.set_caption("DroneProject Ground Station")
        self.screen = pygame.display.set_mode(SIZE)
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("menlo", 14)
        self.small = pygame.font.SysFont("menlo", 11)
        self.big = pygame.font.SysFont("menlo", 18, bold=True)

        self.log_lines = []
        self.log_queue = queue.Queue()
        self.telemetry = Telemetry(telemetry_conn, self.log_queue)
        self.worker = CommandWorker(command_conn, self.log_queue)
        self.telemetry.start()
        self.worker.start()

        self.camera = CameraPanel((790, 40, 470, 300))
        self.camera_feed = CameraFeed(self.camera, self.log_queue)
        self.camera_feed.start()
        self.buttons = self._make_buttons()
        self.path_rects = []       # filled in draw, used for click hits
        self.log_file = open("gcs.log", "a", buffering=1)
        self.copy_button = Button((1176, 466, 72, 22), "COPY",
                                  self._copy_log, (72, 72, 72))
        self.clear_button = Button((686, 46, 72, 22), "CLEAR",
                                   self.telemetry.clear_trail, (72, 72, 72))
        self.copy_flash = 0.0

    def _copy_log(self):
        text = "\n".join(self.log_lines)
        subprocess.run(["pbcopy"], input=text.encode())
        self.copy_flash = time.time()

    # ------------------------------------------------------------ actions --

    def _make_buttons(self):
        w = self.worker
        def simple(label, fn, interrupts=False):
            return lambda: w.submit(label, fn, interrupts)
        specs = [
            ("ARM",     simple("arm", lambda d, a: d.arm(abort=a)), ACCENT),
            ("TAKEOFF", simple("takeoff", lambda d, a: d.takeoff(10, abort=a)), ACCENT),
            ("HOME",    simple("return home",
                               lambda d, a: d.return_home(abort=a), True), ACCENT),
            ("HOLD",    simple("hold", lambda d, a: d.hold(), True), WARN),
            ("LAND",    simple("land", lambda d, a: d.land(), True), WARN),
            ("RTL",     simple("rtl", lambda d, a: d.rtl(), True), WARN),
            ("DISARM",  simple("disarm", lambda d, a: d.disarm(), True), BAD),
        ]
        buttons, x, y = [], 790, 360
        for i, (label, cb, color) in enumerate(specs):
            rect = (x + (i % 4) * 119, y + (i // 4) * 46, 111, 38)
            buttons.append(Button(rect, label, cb, color))
        return buttons

    def _run_path(self, index):
        name, alt, wps = FLIGHT_PATHS[index]
        self.worker.submit(name, fly_path(name, alt, wps))

    # ------------------------------------------------------------ drawing --

    def _panel(self, rect, title=None):
        pygame.draw.rect(self.screen, PANEL, rect, border_radius=8)
        pygame.draw.rect(self.screen, PANEL_EDGE, rect, 1, border_radius=8)
        if title:
            self.screen.blit(self.font.render(title, True, DIM),
                             (rect[0] + 12, rect[1] + 8))

    def _text(self, x, y, label, value, color=TEXT):
        self.screen.blit(self.font.render(label, True, DIM), (x, y))
        self.screen.blit(self.font.render(value, True, color), (x + 110, y))

    def _draw_header(self):
        t = self.telemetry
        armed_color = GOOD if t.armed else DIM
        status = f"{t.mode}  {'ARMED' if t.armed else 'disarmed'}"
        self.screen.blit(self.big.render(status, True, armed_color), (20, 10))
        tlm = ("tlm OK", GOOD) if t.ok else ("tlm LOST", BAD)
        cmd = ("cmd OK", GOOD) if self.worker.connected else ("cmd LOST", BAD)
        self.screen.blit(self.font.render(tlm[0], True, tlm[1]), (250, 14))
        self.screen.blit(self.font.render(cmd[0], True, cmd[1]), (325, 14))
        action = self.worker.busy or "idle"
        line = f"action: {action}   {self.worker.progress}"
        self.screen.blit(self.font.render(line, True, ACCENT), (400, 14))

    def _draw_telemetry(self):
        t = self.telemetry
        self._panel((20, 40, 300, 240), "TELEMETRY")
        x, y = 32, 66
        rows = [
            ("altitude", f"{t.alt:7.2f} m"),
            ("climb", f"{t.climb:+7.2f} m/s"),
            ("speed", f"{t.groundspeed:7.2f} m/s"),
            ("heading", f"{t.heading:7.1f} deg"),
            ("north/east", f"{t.ned[0]:+7.1f} / {t.ned[1]:+7.1f} m"),
            ("latitude", f"{t.lat:12.7f}"),
            ("longitude", f"{t.lon:12.7f}"),
            ("battery", f"{t.voltage:5.2f} V  {t.battery_pct}%",
             GOOD if t.battery_pct > 40 else
             WARN if t.battery_pct > 15 else BAD),
        ]
        for row in rows:
            self._text(x, y, row[0], row[1], row[2] if len(row) > 2 else TEXT)
            y += 24

    def _draw_attitude(self):
        t = self.telemetry
        self._panel((20, 292, 300, 200), "ATTITUDE / HEADING")
        # artificial horizon
        cx, cy, r = 100, 400, 66
        clip = self.screen.get_clip()
        pygame.draw.circle(self.screen, (96, 96, 100), (cx, cy), r)
        offset = t.pitch * 120            # px per rad of pitch
        points = []
        for dx in range(-r, r + 1, 4):
            dy = offset - dx * math.tan(-t.roll)
            points.append((cx + dx, cy + dy))
        ground = points + [(cx + r, cy + r * 2), (cx - r, cy + r * 2)]
        self.screen.set_clip(pygame.Rect(cx - r, cy - r, r * 2, r * 2))
        pygame.draw.polygon(self.screen, (52, 48, 44), ground)
        self.screen.set_clip(clip)
        pygame.draw.circle(self.screen, PANEL_EDGE, (cx, cy), r, 2)
        pygame.draw.line(self.screen, WARN, (cx - 26, cy), (cx - 8, cy), 3)
        pygame.draw.line(self.screen, WARN, (cx + 8, cy), (cx + 26, cy), 3)
        pygame.draw.circle(self.screen, WARN, (cx, cy), 3)
        # compass
        ccx, ccy = 240, 400
        pygame.draw.circle(self.screen, (20, 22, 28), (ccx, ccy), r)
        pygame.draw.circle(self.screen, PANEL_EDGE, (ccx, ccy), r, 2)
        hdg = math.radians(t.heading)
        for ang, name in ((0, 'N'), (90, 'E'), (180, 'S'), (270, 'W')):
            a = math.radians(ang) - hdg
            lx = ccx + (r - 14) * math.sin(a)
            ly = ccy - (r - 14) * math.cos(a)
            color = BAD if name == 'N' else DIM
            label = self.small.render(name, True, color)
            self.screen.blit(label, label.get_rect(center=(lx, ly)))
        pygame.draw.polygon(self.screen, ACCENT, [
            (ccx, ccy - 26), (ccx - 8, ccy + 14), (ccx + 8, ccy + 14)])

    def _draw_map(self):
        rect = pygame.Rect(340, 40, 430, 452)
        self._panel(rect, "MAP (north up, 10 m grid)")
        self.clear_button.draw(self.screen, self.small)
        area = rect.inflate(-24, -44)
        area.top = rect.top + 32
        clip = self.screen.get_clip()
        self.screen.set_clip(area)
        scale = 4.0                                   # px per metre
        cx, cy = area.centerx, area.centery
        def to_screen(north, east):
            return (cx + east * scale, cy - north * scale)
        grid = int(10 * scale)
        for gx in range(area.left + (cx - area.left) % grid, area.right, grid):
            pygame.draw.line(self.screen, (42, 42, 42),
                             (gx, area.top), (gx, area.bottom))
        for gy in range(area.top + (cy - area.top) % grid, area.bottom, grid):
            pygame.draw.line(self.screen, (42, 42, 42),
                             (area.left, gy), (area.right, gy))
        # home
        hx, hy = to_screen(0, 0)
        pygame.draw.line(self.screen, DIM, (hx - 7, hy), (hx + 7, hy))
        pygame.draw.line(self.screen, DIM, (hx, hy - 7), (hx, hy + 7))
        # trail
        trail = self.telemetry.trail_copy()
        if len(trail) > 1:
            pygame.draw.lines(self.screen, (172, 172, 172), False,
                              [to_screen(n, e) for n, e in trail], 2)
        # drone: triangle pointing along yaw
        n, e, _ = self.telemetry.ned
        dx, dy = to_screen(n, e)
        yaw = self.telemetry.yaw
        tip = (dx + 12 * math.sin(yaw), dy - 12 * math.cos(yaw))
        left = (dx + 7 * math.sin(yaw + 2.5), dy - 7 * math.cos(yaw + 2.5))
        right = (dx + 7 * math.sin(yaw - 2.5), dy - 7 * math.cos(yaw - 2.5))
        pygame.draw.polygon(self.screen, GOOD, [tip, left, right])
        self.screen.set_clip(clip)

    def _draw_paths(self):
        rect = pygame.Rect(340, 504, 430, 276)
        self._panel(rect, "FLIGHT PATHS  (click to fly)")
        self.path_rects = []
        y = rect.top + 34
        for i, (name, alt, wps) in enumerate(FLIGHT_PATHS):
            row = pygame.Rect(rect.left + 12, y, rect.width - 24, 32)
            hover = row.collidepoint(pygame.mouse.get_pos())
            running = self.worker.busy == name
            color = ACCENT if running else (50, 50, 50) if hover else (40, 40, 40)
            pygame.draw.rect(self.screen, color, row, border_radius=6)
            label = f"{name:<22s} {len(wps):>2d} wp   alt {alt} m"
            text_color = (12, 14, 18) if running else TEXT
            self.screen.blit(self.font.render(label, True, text_color),
                             (row.left + 10, row.top + 8))
            self.path_rects.append((row, i))
            y += 38

    def _draw_log(self):
        rect = pygame.Rect(790, 460, 470, 320)
        self._panel(rect, "LOG (full history in gcs.log)")
        copied = time.time() - self.copy_flash < 1.5
        self.copy_button.label = "COPIED" if copied else "COPY"
        self.copy_button.color = GOOD if copied else (60, 68, 88)
        self.copy_button.draw(self.screen, self.small)
        y = rect.bottom - 22
        for line in reversed(self.log_lines[-20:]):
            self.screen.blit(self.small.render(line[:64], True, DIM),
                             (rect.left + 12, y))
            y -= 15
            if y < rect.top + 28:
                break

    # --------------------------------------------------------------- loop --

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    for button in self.buttons:
                        button.handle(event.pos)
                    self.copy_button.handle(event.pos)
                    self.clear_button.handle(event.pos)
                    for row, index in self.path_rects:
                        if row.collidepoint(event.pos):
                            self._run_path(index)
            while not self.log_queue.empty():
                line = time.strftime("%H:%M:%S ") + self.log_queue.get()
                self.log_lines.append(line)
                self.log_file.write(line + "\n")
            self.screen.fill(BG)
            self._draw_header()
            self._draw_telemetry()
            self._draw_attitude()
            self._draw_map()
            self._draw_paths()
            self.camera.draw(self.screen, self.font, self.small)
            for button in self.buttons:
                button.draw(self.screen, self.font)
            self._draw_log()
            pygame.display.flip()
            self.clock.tick(FPS)
        pygame.quit()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--telemetry', default='udpin:127.0.0.1:14550',
                        help='telemetry connection (default: %(default)s)')
    parser.add_argument('--command', default='tcp:127.0.0.1:5762',
                        help='command connection (default: %(default)s)')
    args = parser.parse_args()
    App(args.telemetry, args.command).run()


if __name__ == '__main__':
    main()
