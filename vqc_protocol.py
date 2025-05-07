
"""
vqc_protocol.py
Visiting Quadcopter (V-QC) protocol:
- Initial random roaming.
- Receives ASSIGN and visits PoIs.
- Locally detects PoI IDs and delivers them back.
"""

import json
import math
import logging
from typing import List, Tuple

from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.communication import CommunicationCommand, CommunicationCommandType
from gradysim.protocol.messages.telemetry import Telemetry
from gradysim.protocol.plugin.random_mobility import RandomMobilityPlugin, RandomMobilityConfig
from gradysim.protocol.plugin.mission_mobility import MissionMobilityPlugin, MissionMobilityConfiguration, LoopMission

from config import L, R_DETECT, M, POIS

class VQCProtocol(IProtocol):
    def initialize(self) -> None:
        self.id = self.provider.get_id()
        self.log = logging.getLogger(f"VQC-{self.id}")
        self.pos = (0.0, 0.0, 4.0)
        self.next2visit: List[Tuple[Tuple[float, float], int]] = []
        self.discovered: List[str] = []
        self.visited: List[str] = []
        self.delivering = False

        self.random = RandomMobilityPlugin(
            self,
            RandomMobilityConfig(x_range=(0, L), y_range=(0, L), z_range=(4.0, 4.0), tolerance=1)
        )
        self.mission = MissionMobilityPlugin(
            self,
            MissionMobilityConfiguration(speed=10, loop_mission=LoopMission.NO, tolerance=1)
        )

        self.random.initiate_random_trip()
        self.provider.schedule_timer("hello", self.provider.current_time() + 4)
        self.provider.schedule_timer("check_roam", self.provider.current_time() + 1)

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        self.pos = telemetry.current_position
        if not self.mission.is_idle:
            for coord3d, urg in list(self.next2visit):
                dx, dy, dz = (
                    self.pos[0]-coord3d[0],
                    self.pos[1]-coord3d[1],
                    self.pos[2]-coord3d[2]
                )
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                if dist <= R_DETECT:
                    pid = next(
                        p["id"] for p in POIS
                        if p["coord"] == (coord3d[0], coord3d[1]) and p["urgency"] == urg
                    )
                    if pid not in self.discovered and pid not in self.visited:
                        self.discovered.append(pid)
                    if not self.delivering and self.discovered:
                        self.delivering = True
                        self.provider.schedule_timer("deliver", self.provider.current_time() + 1)

    def handle_timer(self, timer: str) -> None:
        if timer == "hello":
            free = M - len(self.next2visit)
            msg = {
                "type": "HELLO",
                "v_id": self.id,
                "huecos": free,
                "visited": self.visited.copy(),
                "position": list(self.pos)
            }
            cmd = CommunicationCommand(CommunicationCommandType.BROADCAST, json.dumps(msg))
            self.provider.send_communication_command(cmd)
            self.provider.schedule_timer("hello", self.provider.current_time() + 1)

        elif timer == "deliver" and self.delivering:
            report = {"type": "DELIVER", "v_id": self.id, "pids": self.discovered.copy()}
            cmd = CommunicationCommand(CommunicationCommandType.BROADCAST, json.dumps(report))
            self.provider.send_communication_command(cmd)
            delivered = set(self.discovered)
            self.visited.extend(self.discovered)
            self.discovered.clear()
            self.next2visit = [
                (coord3d, urg) for coord3d, urg in self.next2visit
                if next(
                    p["id"] for p in POIS
                    if p["coord"] == (coord3d[0], coord3d[1]) and p["urgency"] == urg
                ) not in delivered
            ]
            if self.next2visit:
                waypoints = [coord for coord, _ in self.next2visit]
                self.mission.start_mission(waypoints)
            else:
                self.delivering = False
                self.provider.schedule_timer("check_roam", self.provider.current_time() + 0.1)

        elif timer == "check_roam":
            if self.mission.is_idle and not self.random._trip_ongoing:
                self.random.initiate_random_trip()
            self.provider.schedule_timer("check_roam", self.provider.current_time() + 1)

    def handle_packet(self, message: str) -> None:
        msg = json.loads(message)
        if msg.get("type") == "ASSIGN":
            if self.discovered:
                report = {"type": "DELIVER", "v_id": self.id, "pids": self.discovered.copy()}
                cmd = CommunicationCommand(CommunicationCommandType.BROADCAST, json.dumps(report))
                self.provider.send_communication_command(cmd)
                self.visited.extend(self.discovered)
            self.next2visit.clear()
            self.discovered.clear()
            self.delivering = False
            for p in msg["pois"]:
                x, y = p["coord"]
                self.next2visit.append(((x, y, 4.0), p["urgency"]))
            self.random.finish_random_trip()
            if not msg["pois"]:
                self.random.initiate_random_trip()
                return
            waypoints = [coord for coord, _ in self.next2visit]
            self.mission.start_mission(waypoints)

    def finish(self) -> None:
        self.log.info(f"VQC-{self.id} finished — remaining={self.next2visit}, visited={self.visited}")
