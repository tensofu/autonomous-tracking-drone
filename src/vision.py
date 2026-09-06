#!/usr/bin/env python3
"""Vision system for the tracking drone.

Consumes camera frames (BGR numpy arrays) and produces typed Detections:

  - 'apriltag'  : the landing dock fiducial (AprilTag 36h11, id 0)
  - 'person'    : the follow target (bright red cylinder in the sim world)
  - 'obstacle'  : hazard-yellow walls and objects to avoid

This module is the seam for the real hardware: on the actual drone a
Raspberry Pi AI Camera (IMX500) runs the detection networks and returns
much the same information (class, bounding box, confidence). Swap
`VisionSystem.process()` internals for the AI camera's outputs and the
behaviors in missions.py work unchanged. In the simulation the detectors
are classical CV: aruco for the tag, HSV color segmentation for the
person and obstacles.

FrameSource reads the Gazebo camera's H.264/RTP stream (via OpenCV's
FFmpeg) on a daemon thread, runs VisionSystem on every frame, and keeps
the latest annotated frame + detections for consumers (GUI panel,
mission control loops, headless tests).
"""

import math
import os
import subprocess
import threading
import time
from dataclasses import dataclass, field

os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS",
                      "protocol_whitelist;file,rtp,udp")
os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "-8")
import cv2
import numpy as np

# camera model (gimbal_small_3d camera sensor in the sim)
FRAME_W, FRAME_H = 640, 480
FOV_H = 2.0                                   # radians, horizontal
F_PX = FRAME_W / (2 * math.tan(FOV_H / 2))    # focal length in pixels

# physical sizes used for monocular distance estimates
TAG_SIZE_M = 0.96          # black tag square on the dock (1.2 m face * 512/640)
PERSON_WIDTH_M = 0.5      # apparent diameter from directly above (nadir camera)
WALL_HEIGHT_M = 2.5

CAMERA_ENABLE_TOPIC = ("/world/tracking_arena/model/iris_with_gimbal/model/"
                       "iris_with_standoffs/link/base_link/sensor/camera/"
                       "image/enable_streaming")
SDP_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "camera_stream.sdp")


@dataclass
class Detection:
    kind: str                  # 'apriltag' | 'person' | 'obstacle'
    center: tuple              # (u, v) pixels
    box: tuple                 # (x, y, w, h) pixels
    distance_m: float          # monocular estimate (rough!)
    bearing_rad: float         # horizontal angle from image centre (+right)
    elevation_rad: float       # vertical angle from image centre (+down)
    area_frac: float           # box area / frame area (proximity heuristic)
    extra: dict = field(default_factory=dict)


def _angles(u, v):
    return (math.atan2(u - FRAME_W / 2, F_PX),
            math.atan2(v - FRAME_H / 2, F_PX))


class VisionSystem:

    def __init__(self):
        self._aruco = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
            cv2.aruco.DetectorParameters())

    # ------------------------------------------------------------ detectors

    def _detect_tag(self, frame, gray):
        corners, ids, _ = self._aruco.detectMarkers(gray)
        out = []
        if ids is None:
            return out
        for quad, tag_id in zip(corners, ids.flatten()):
            pts = quad.reshape(4, 2)
            u, v = pts.mean(axis=0)
            side = float(np.mean([np.linalg.norm(pts[i] - pts[(i + 1) % 4])
                                  for i in range(4)]))
            x, y, w, h = cv2.boundingRect(pts.astype(np.int32))
            bearing, elevation = _angles(u, v)
            out.append(Detection(
                kind='apriltag', center=(u, v), box=(x, y, w, h),
                distance_m=F_PX * TAG_SIZE_M / max(side, 1.0),
                bearing_rad=bearing, elevation_rad=elevation,
                area_frac=w * h / (FRAME_W * FRAME_H),
                extra={'id': int(tag_id), 'corners': pts}))
        return out

    def _detect_person(self, frame, hsv):
        # bright red: two hue bands around 0/180
        mask = (cv2.inRange(hsv, (0, 130, 90), (8, 255, 255))
                | cv2.inRange(hsv, (172, 130, 90), (180, 255, 255)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        best = max(contours, key=cv2.contourArea, default=None)
        if best is None or cv2.contourArea(best) < 60:
            return []
        x, y, w, h = cv2.boundingRect(best)
        u, v = x + w / 2, y + h / 2
        bearing, elevation = _angles(u, v)
        # nadir camera: the person's circular footprint from above, not
        # their standing height, is what's in frame - use apparent width
        # against the known real diameter. If clipped by any frame edge
        # (not just top/bottom - a top-down blob can exit any side) the
        # estimate degrades.
        clipped = (x <= 2 or y <= 2
                  or x + w >= FRAME_W - 2 or y + h >= FRAME_H - 2)
        return [Detection(
            kind='person', center=(u, v), box=(x, y, w, h),
            distance_m=F_PX * PERSON_WIDTH_M / max(w, 1),
            bearing_rad=bearing, elevation_rad=elevation,
            area_frac=w * h / (FRAME_W * FRAME_H),
            extra={'clipped': clipped})]

    def _detect_obstacles(self, frame, hsv):
        # hazard yellow walls / objects
        mask = cv2.inRange(hsv, (20, 120, 120), (38, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                np.ones((7, 7), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in contours:
            if cv2.contourArea(c) < 400:
                continue
            x, y, w, h = cv2.boundingRect(c)
            u, v = x + w / 2, y + h / 2
            bearing, elevation = _angles(u, v)
            # monocular wall distance is unreliable (unknown visible height),
            # so estimate optimistically from height but ALSO report area
            # fraction; the collision guard treats a large area fraction as
            # "close" regardless of the distance estimate
            out.append(Detection(
                kind='obstacle', center=(u, v), box=(x, y, w, h),
                distance_m=F_PX * WALL_HEIGHT_M / max(h, 1),
                bearing_rad=bearing, elevation_rad=elevation,
                area_frac=w * h / (FRAME_W * FRAME_H)))
        return out

    # -------------------------------------------------------------- process

    def process(self, frame):
        """Returns (annotated_frame_bgr, {kind: [Detection, ...]})."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = {
            'apriltag': self._detect_tag(frame, gray),
            'person': self._detect_person(frame, hsv),
            'obstacle': self._detect_obstacles(frame, hsv),
        }
        return self._annotate(frame.copy(), detections), detections

    @staticmethod
    def _annotate(frame, detections):
        styles = {'apriltag': ((0, 255, 0), "TAG"),
                  'person': ((0, 0, 255), "PERSON"),
                  'obstacle': ((0, 200, 255), "OBST")}
        for kind, dets in detections.items():
            color, label = styles[kind]
            for d in dets:
                x, y, w, h = d.box
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                cv2.putText(frame, f"{label} {d.distance_m:.1f}m",
                            (x, max(y - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, color, 1, cv2.LINE_AA)
        cv2.drawMarker(frame, (FRAME_W // 2, FRAME_H // 2), (255, 255, 255),
                       cv2.MARKER_CROSS, 16, 1)
        return frame


class FrameSource(threading.Thread):
    """Decodes the camera stream, runs VisionSystem, keeps latest results.

    Consumers:
      latest()      -> (annotated_bgr | None, {kind: [Detection]}, age_s)
      set_overlay() -> optional extra text drawn on the annotated frame
    """

    def __init__(self, log=print, enable_topic=CAMERA_ENABLE_TOPIC):
        super().__init__(daemon=True)
        self.log = log
        self.enable_topic = enable_topic
        self.vision = VisionSystem()
        self.frames = 0
        self.fps = 0.0
        self._lock = threading.Lock()
        self._annotated = None
        self._detections = {}
        self._stamp = 0.0
        self._overlay = ""

    def latest(self):
        with self._lock:
            age = time.time() - self._stamp if self._stamp else float('inf')
            return self._annotated, dict(self._detections), age

    def set_overlay(self, text):
        self._overlay = text

    def _enable_streaming(self):
        env = {**os.environ, "GZ_IP": "127.0.0.1"}
        try:
            subprocess.run(["gz", "topic", "-t", self.enable_topic,
                            "-m", "gz.msgs.Boolean", "-p", "data: true"],
                           env=env, capture_output=True, timeout=10)
        except (OSError, subprocess.TimeoutExpired):
            pass

    def run(self):
        announced = False
        while True:
            self._enable_streaming()
            cap = cv2.VideoCapture(SDP_FILE, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                cap.release()
                if not announced:
                    self.log("camera: waiting for stream ...")
                    announced = True
                time.sleep(3)
                continue
            announced = False
            self.log("camera: stream opened")
            self._pump(cap)
            cap.release()
            with self._lock:
                self._annotated, self._detections = None, {}
            self.log("camera: stream lost - reconnecting ...")

    def _pump(self, cap):
        """Capture on a dedicated thread that keeps only the NEWEST frame;
        process here at whatever rate we manage. Processing slower than the
        stream must drop frames, never queue them - otherwise the decoder
        buffer grows and detections lag reality by tens of seconds, which
        sent the drone chasing minute-old imagery in testing."""
        newest = {}
        stop = threading.Event()

        def reader():
            while not stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    newest['dead'] = True
                    return
                newest['frame'] = frame
                newest['t'] = time.time()

        rt = threading.Thread(target=reader, daemon=True)
        rt.start()
        counted, window_start = 0, time.time()
        last_t = 0.0
        while not newest.get('dead'):
            frame = newest.get('frame')
            t = newest.get('t', 0.0)
            if frame is None or t == last_t:
                time.sleep(0.01)
                continue
            last_t = t
            self.frames += 1
            counted += 1
            now = time.time()
            if now - window_start >= 1.0:
                self.fps = counted / (now - window_start)
                counted, window_start = 0, now
            annotated, detections = self.vision.process(frame)
            if self._overlay:
                cv2.putText(annotated, self._overlay, (8, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255),
                            1, cv2.LINE_AA)
            with self._lock:
                self._annotated = annotated
                self._detections = detections
                self._stamp = t          # capture time, not processing time
        stop.set()
