"""
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

        self.log.info(f"🛫 VQC-{self.id} initialized at {self.pos}")

        self.random = RandomMobilityPlugin(
            self, RandomMobilityConfig(x_range=(0,L), y_range=(0,L), z_range=(4.0,4.0), tolerance=1)
        )
        self.mission = MissionMobilityPlugin(
            self, MissionMobilityConfiguration(speed=10, loop_mission=LoopMission.NO, tolerance=1)
        )

        self.random.initiate_random_trip()
        self.log.info("✅ Random roaming started")
        t0 = self.provider.current_time()
        self.provider.schedule_timer("hello", t0+4)
        self.provider.schedule_timer("check_roam", t0+1)

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        old = self.pos
        self.pos = telemetry.current_position
        self.log.debug(f"📡 Telemetry: from {old} to {self.pos}")

        if not self.mission.is_idle:
            for coord3d, urg in list(self.next2visit):
                dx, dy, dz = self.pos[0]-coord3d[0], self.pos[1]-coord3d[1], self.pos[2]-coord3d[2]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                self.log.debug(f"    Dist to {coord3d}: {dist:.2f} (tol={R_DETECT})")
                if dist <= R_DETECT:
                    pid = next(p["id"] for p in POIS if p["coord"]==(coord3d[0],coord3d[1]) and p["urgency"]==urg)
                    if pid not in self.discovered and pid not in self.visited:
                        self.discovered.append(pid)
                        self.log.info(f"🔍 Local detect: {pid}")
                    if not self.delivering and self.discovered:
                        self.delivering = True
                        self.log.info(f"📦 Starting deliver phase with {self.discovered}")
                        self.provider.schedule_timer("deliver", self.provider.current_time()+1)

    def handle_timer(self, timer: str) -> None:
        if timer == "hello":
            free = M - len(self.next2visit)
            msg = {"type":"HELLO","v_id":self.id,"huecos":free,"visited":self.visited.copy(),"position":list(self.pos)}
            self.log.debug(f"📤 HELLO payload: {msg}")
            cmd = CommunicationCommand(CommunicationCommandType.BROADCAST, json.dumps(msg))
            self.provider.send_communication_command(cmd)
            self.log.info(f"📤 HELLO sent: free={free}, visited={self.visited}")
            self.provider.schedule_timer("hello", self.provider.current_time()+1)

        elif timer == "deliver" and self.delivering:
            report = {"type":"DELIVER","v_id":self.id,"pids":self.discovered.copy()}
            self.log.debug(f"📣 DELIVER payload: {report}")
            cmd = CommunicationCommand(CommunicationCommandType.BROADCAST, json.dumps(report))
            self.provider.send_communication_command(cmd)
            self.log.info(f"📣 DELIVER sent: {self.discovered}")
            delivered = set(self.discovered)
            self.visited.extend(self.discovered)
            self.discovered.clear()
            self.log.debug(f"🗑️ Discovered cleared; visited={self.visited}")

            before = len(self.next2visit)
            self.next2visit = [(c,u) for (c,u) in self.next2visit
                               if next(p["id"] for p in POIS if p["coord"]==(c[0],c[1]) and p["urgency"]==u)
                               not in delivered]
            self.log.debug(f"🗑️ next2visit filtered: {before}→{len(self.next2visit)}")

            if self.next2visit:
                waypoints = [c for (c,_) in self.next2visit]
                self.log.info(f"🗺️ Restarting mission to {waypoints}")
                self.mission.start_mission(waypoints)
            else:
                self.delivering = False
                self.log.warning("⚠️ No PoIs left → will resume roaming")
                self.provider.schedule_timer("check_roam", self.provider.current_time()+0.1)

        elif timer == "check_roam":
            self.log.debug(f"🔥 check_roam: idle={self.mission.is_idle}, roaming={self.random._trip_ongoing}")
            if self.mission.is_idle and not self.random._trip_ongoing:
                self.log.info("✅ check_roam: resuming random roaming")
                self.random.initiate_random_trip()
            self.provider.schedule_timer("check_roam", self.provider.current_time()+1)

    def handle_packet(self, message: str) -> None:
        self.log.debug(f"📥 handle_packet ASSIGN: {message}")
        msg = json.loads(message)
        if msg.get("type") == "ASSIGN":
            self.log.info(f"📥 ASSIGN received: {msg['pois']}")
            if self.discovered:
                report = {"type":"DELIVER","v_id":self.id,"pids":self.discovered.copy()}
                cmd = CommunicationCommand(CommunicationCommandType.BROADCAST, json.dumps(report))
                self.provider.send_communication_command(cmd)
                self.log.info(f"📣 Flushed deliver: {self.discovered}")
                self.visited.extend(self.discovered)

            self.next2visit.clear()
            self.discovered.clear()
            self.delivering = False
            self.log.debug("🔄 Cleared next2visit & discovered")

            for p in msg["pois"]:
                x,y = p["coord"]
                self.next2visit.append(((x,y,4.0), p["urgency"]))
            self.log.debug(f"📋 Loaded next2visit: {self.next2visit}")

            self.random.finish_random_trip()
            if not msg["pois"]:
                self.log.warning("⚠️ ASSIGN empty → resuming roaming")
                self.random.initiate_random_trip()
                return

            waypoints = [c for (c,_) in self.next2visit]
            self.log.info(f"🛑 Stopping roam, starting mission to {waypoints}")
            self.mission.start_mission(waypoints)

    def finish(self) -> None:
        self.log.info(f"🏁 VQC-{self.id} finished — next2visit={self.next2visit}, visited={self.visited}")
