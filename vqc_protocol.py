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
                        if len(self.discovered) <M:
                            self.discovered.append(pid)
                            self.log.info(f"🔍 Local detect: {pid}")
                        else:
                            self.log.warning(f"Buffer discovered lleno")

    def handle_timer(self, timer: str) -> None:
        if timer == "hello":
            if self.discovered:
                report = {
                    "type": "DELIVER",
                    "v_id": self.id,
                    "pids": self.discovered.copy()
                }
                cmd = CommunicationCommand(
                    CommunicationCommandType.BROADCAST,
                    json.dumps(report)
                )
                self.provider.send_communication_command(cmd)
                self.log.info(f"📣 DELIVER automático en HELLO: entregados {self.discovered}")
                # Marcar como visitados y vaciar buffer
                #self.visited.extend(self.discovered)
                #self.discovered.clear()            
            free = M - len(self.next2visit)
            msg = {"type":"HELLO","v_id":self.id,"huecos":free,"visited":self.visited.copy(),"position":list(self.pos)}
            self.log.debug(f"📤 HELLO payload: {msg}")
            cmd = CommunicationCommand(CommunicationCommandType.BROADCAST, json.dumps(msg))
            self.provider.send_communication_command(cmd)
            self.log.info(f"📤 HELLO sent: free={free}, visited={self.visited}")
            self.provider.schedule_timer("hello", self.provider.current_time()+1)

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

            antiguos = list(self.next2visit)

            self.next2visit.clear()
            # 3) Cargar primero las nuevas tareas que envía el EQC
            nuevos_ids = set()
            for p in msg["pois"]:
                x, y = p["coord"]
                urg = p["urgency"]
                coord3d = (x, y, 4.0)
                self.next2visit.append((coord3d, urg))
                nuevos_ids.add(p["label"])      # usa "label" o "id" según tu POIS
            # 4) Volver a añadir las antiguas que no estén ya en los nuevos,
            #    hasta completar la capacidad M
            for coord3d, urg in antiguos:
                # obtener el label según las coordenadas
                label = next(poi["label"]
                            for poi in POIS
                            if poi["coord"] == (coord3d[0], coord3d[1]))
                if label not in nuevos_ids and len(self.next2visit) < M:
                    self.next2visit.append((coord3d, urg))

            # 5) Si tras el merge no queda nada, reanudar roaming
            if not self.next2visit:
                self.log.warning("⚠️ No quedan PoIs tras merge → reanudando roaming")
                self.random.initiate_random_trip()
                return
                
            # 6) Arrancar la misión guiada con la lista combinada
            waypoints = [coord for (coord, _) in self.next2visit]
            self.log.info(f"🗺️ Waypoints combinados: {waypoints}")
            self.random.finish_random_trip()
            self.mission.start_mission(waypoints)

        elif msg.get("type") == "DELIVER_ACK":
            acked = msg.get("pids", [])
            self.log.info(f"📥 DELIVER_ACK recibido: {acked}")
            # Solo eliminar de discovered los PoIs que realmente llegaron
            for pid in acked:
                if pid in self.discovered:
                    self.discovered.remove(pid)
                    self.visited.append(pid)
            self.log.debug(f"🗂️ discovered tras ACK: {self.discovered}, visited: {self.visited}")
       
    def finish(self) -> None:
        self.log.info(f"🏁 VQC-{self.id} finished — next2visit={self.next2visit}, visited={self.visited}")
