"""
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

from config import L, R_CAMERA, R_DETECT, POIS, METRICS

class EQCProtocol(IProtocol):
    def initialize(self) -> None:
        self.id = self.provider.get_id()
        self.log = logging.getLogger(f"EQC-{self.id}")

        # Definir waypoints de patrulla
        waypoints = [
            (0,0,7.0),(L,0,7.0),(L,10,7.0),(0,10,7.0),
            (0,20,7.0),(L,20,7.0),(L,30,7.0),(0,30,7.0),
            (0,40,7.0),(L,40,7.0),(L,L,7.0),(0,L,7.0),
        ]
        self.log.info(f"🛰️  EQC iniciando patrulla con waypoints: {waypoints}")
        cfg = MissionMobilityConfiguration(
            speed=10.0,
            loop_mission=LoopMission.RESTART,
            tolerance=1
        )
        self.mission = MissionMobilityPlugin(self, cfg)
        self.mission.start_mission(waypoints)

        # Configurar cámara
        cam_cfg = CameraConfiguration(
            camera_reach=R_CAMERA,
            camera_theta=60.0,
            facing_elevation=180.0,
            facing_rotation=0.0
        )
        self.camera = CameraHardware(self, cam_cfg)
        self.log.info(f"📷 Camera configured: reach={R_CAMERA}, theta={cam_cfg.camera_theta}")

        # Estados internos
        self.pending: List[dict] = []
        self.detect_ts: dict = {}
        self.vqc_states: dict = {}

        # Programar muestreo de detección y asignación cada 1s
        next_t = self.provider.current_time() + 1
        self.provider.schedule_timer("assign", next_t)
        self.log.debug(f"⏱️ Scheduled first 'assign' at t={next_t:.2f}")

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        self.log.debug(f"📡 Telemetry: pos={telemetry.current_position}, idle={self.mission.is_idle}")
        if not self.mission.is_idle:
            wp = self.mission.current_waypoint
            self.log.info(f"🛰️  EQC rumbo al waypoint en {wp}")

    def handle_timer(self, timer: str) -> None:
        if timer == "assign":
            now = self.provider.current_time()
            self.log.info(f"⚙️  EQC handle_timer('assign') @ t={now:.2f}")
            detected = self.camera.take_picture()
            self.log.info(f"⚙️  assign @ t={now:.2f}: {len(detected)} nodos detectados")

            # Log raw detections
            for node in detected:
                pos = node["position"]
                self.log.debug(f"   📸 Raw detection at {pos}")

            # Filtrar PoIs
            new_cnt = 0
            eps = 1e-3
            for poi in POIS:
                px, py = poi["coord"]
                for node in detected:
                    x, y, z = node["position"]
                    if abs(x-px)<eps and abs(y-py)<eps and abs(z-0.0)<eps:
                        label = poi["label"]
                        if label not in self.detect_ts:
                            self.detect_ts[label] = now
                            self.pending.append(poi)
                            new_cnt += 1
                            self.log.info(f"🔍 {label} detectado @ {poi['coord']} t={now:.2f}")
                        break

            self.log.debug(f"🗂️ pending size: {len(self.pending)} (+{new_cnt})")

            # Reprogramar
            next_t = now + 1
            self.provider.schedule_timer("assign", next_t)
            self.log.debug(f"⏱️ Rescheduled 'assign' at t={next_t:.2f}")

    def handle_packet(self, message: str) -> None:
        self.log.debug(f"📥 handle_packet: {message}")
        msg = json.loads(message)
        t = msg.get("type")

        if t == "HELLO":
            vid = msg["v_id"]
            free = msg["huecos"]
            visited = msg["visited"]
            pos = tuple(msg["position"])
            self.log.info(f"📩 HELLO from VQC-{vid}: free={free}, visited={visited}, pos={pos}")

            before = len(self.pending)
            self.pending = [p for p in self.pending if p["label"] not in visited]
            self.log.debug(f"🗑️ pending filtered: {before}→{len(self.pending)}")

            self.vqc_states[vid] = {"huecos": free, "pos": pos}

        elif t == "DELIVER":
            vid = msg["v_id"]
            pids = msg.get("pids", [])
            self.log.info(f"📥 DELIVER from VQC-{vid}: {pids}")
            for pid in pids:
                if pid not in METRICS["unique_ids"]:
                    METRICS["unique_ids"].add(pid)
                else:
                    METRICS["redundant"] += 1
            self.log.debug(f"🧮 Metrics: unique={len(METRICS['unique_ids'])}, redundant={METRICS['redundant']}")

        self.assign_to_vqcs()

    def assign_to_vqcs(self) -> None:
        self.log.debug(f"🔍 assign_to_vqcs: pending={len(self.pending)}, states={self.vqc_states}")
        for vid, st in self.vqc_states.items():
            free, pos = st["huecos"], st["pos"]
            self.log.debug(f"→ VQC-{vid} state: free={free}, pos={pos}")
            if free <= 0:
                self.log.debug(f"→ VQC-{vid} no free slots")
                continue

            scored = []
            for poi in self.pending:
                dist = max(1e-6, math.hypot(pos[0]-poi["coord"][0], pos[1]-poi["coord"][1]))
                score = poi["urgency"]/dist
                scored.append((score, poi))
                self.log.debug(f"    ⋅ {poi['label']} urg={poi['urgency']} dist={dist:.2f} score={score:.2f}")

            scored.sort(key=lambda x: x[0], reverse=True)
            to_assign = [p for _, p in scored[:free]]
            if not to_assign:
                self.log.debug(f"→ No PoIs for VQC-{vid}")
                continue

            payload = {
                "type":"ASSIGN","v_id":vid,
                "pois":[{"label":p["label"],"coord":p["coord"],"urgency":p["urgency"],"ts":self.detect_ts[p["label"]]} for p in to_assign]
            }
            self.log.debug(f"🚀 ASSIGN payload for VQC-{vid}: {payload}")

            cmd = CommunicationCommand(CommunicationCommandType.SEND, json.dumps(payload), vid)
            self.provider.send_communication_command(cmd)
            self.log.info(f"🚀 ASSIGN {len(to_assign)} to VQC-{vid}: {[p['label'] for p in to_assign]}")

    def finish(self) -> None:
        total = len(self.detect_ts)
        red = METRICS["redundant"]
        self.log.info(f"✅ EQC finished. Unique={total}, redundant={red}")
