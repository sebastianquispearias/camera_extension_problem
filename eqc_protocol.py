
"""
eqc_protocol.py
Exploration Quadcopter (E-QC) protocol:
- Patrols the area via predefined waypoints.
- Captures and filters PoI detections.
- Coordinates with V-QCs by sending ASSIGN messages.
"""

import json
import math
import logging
from typing import List

from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.communication import CommunicationCommand, CommunicationCommandType
from gradysim.protocol.messages.telemetry import Telemetry
from gradysim.protocol.plugin.mission_mobility import MissionMobilityPlugin, MissionMobilityConfiguration, LoopMission
from gradysim.simulator.extension.camera import CameraHardware, CameraConfiguration

from config import L, R_CAMERA, POIS, METRICS

class EQCProtocol(IProtocol):
    def initialize(self) -> None:
        self.id = self.provider.get_id()
        self.log = logging.getLogger(f"EQC-{self.id}")

        # Patrol mission waypoints
        waypoints = [
            (0,0,7.0),(L,0,7.0),(L,10,7.0),(0,10,7.0),
            (0,20,7.0),(L,20,7.0),(L,30,7.0),(0,30,7.0),
            (0,40,7.0),(L,40,7.0),(L,L,7.0),(0,L,7.0),
        ]
        cfg = MissionMobilityConfiguration(
            speed=10.0,
            loop_mission=LoopMission.RESTART,
            tolerance=1
        )
        self.mission = MissionMobilityPlugin(self, cfg)
        self.mission.start_mission(waypoints)

        # Onboard camera setup
        cam_cfg = CameraConfiguration(
            camera_reach=R_CAMERA,
            camera_theta=60.0,
            facing_elevation=180.0,
            facing_rotation=0.0
        )
        self.camera = CameraHardware(self, cam_cfg)

        # Internal state
        self.pending: List[dict] = []
        self.detect_ts: dict = {}
        self.vqc_states: dict = {}

        # Schedule first detection
        self.provider.schedule_timer("assign", self.provider.current_time() + 1)

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        if not self.mission.is_idle:
            wp = self.mission.current_waypoint
            self.log.info(f"EQC heading to {wp}")

    def handle_timer(self, timer: str) -> None:
        if timer == "assign":
            now = self.provider.current_time()
            detected = self.camera.take_picture()
            for poi in POIS:
                px, py = poi["coord"]
                for node in detected:
                    x, y, z = node["position"]
                    if abs(x-px) < 1e-3 and abs(y-py) < 1e-3 and z == 0.0:
                        label = poi["label"]
                        if label not in self.detect_ts:
                            self.detect_ts[label] = now
                            self.pending.append(poi)
                        break
            self.provider.schedule_timer("assign", now + 1)

    def handle_packet(self, message: str) -> None:
        msg = json.loads(message)
        t = msg.get("type")
        if t == "HELLO":
            vid = msg["v_id"]
            free = msg["huecos"]
            visited = msg["visited"]
            pos = tuple(msg["position"])
            self.pending = [p for p in self.pending if p["label"] not in visited]
            self.vqc_states[vid] = {"huecos": free, "pos": pos}
        elif t == "DELIVER":
            for pid in msg.get("pids", []):
                if pid not in METRICS["unique_ids"]:
                    METRICS["unique_ids"].add(pid)
                else:
                    METRICS["redundant"] += 1
        self.assign_to_vqcs()

    def assign_to_vqcs(self) -> None:
        for vid, st in self.vqc_states.items():
            free = st["huecos"]
            if free <= 0:
                continue
            scored = []
            for poi in self.pending:
                dist = max(1e-6, math.hypot(
                    st["pos"][0]-poi["coord"][0],
                    st["pos"][1]-poi["coord"][1]
                ))
                score = poi["urgency"] / dist
                scored.append((score, poi))
            scored.sort(key=lambda x: x[0], reverse=True)
            to_assign = [p for _, p in scored[:free]]
            if to_assign:
                payload = {
                    "type": "ASSIGN",
                    "v_id": vid,
                    "pois": [
                        {
                            "label": p["label"],
                            "coord": p["coord"],
                            "urgency": p["urgency"],
                            "ts": self.detect_ts[p["label"]]
                        }
                        for p in to_assign
                    ]
                }
                cmd = CommunicationCommand(
                    CommunicationCommandType.SEND,
                    json.dumps(payload),
                    vid
                )
                self.provider.send_communication_command(cmd)
