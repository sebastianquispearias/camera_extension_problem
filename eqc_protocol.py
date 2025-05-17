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
from config import MAX_ASSIGN_PER_ENCOUNTER

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
        self.log.info(f"Current handlers: {self.log.handlers!r}")
        # ——— Inicializar el último waypoint para el logging condicional ———
        self._last_wp = None

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

        # ---------- Métricas ----------
        self.start_time        = self.provider.current_time()
        self.assign_count      = 0                # total ASSIGNs enviadas
        self.assign_success    = 0                # PoIs de ASSIGN que efectivamente se entregaron
        self.assign_times      = {}               # mapa poi_label → t_assign
        self.latencies         = []               # lista de (label, latency)
        self.coverage_timeline = []               # lista de (elapsed_time, unique_count)
        self.redundant_delivers = 0
        METRICS["unique_ids"]  = set()
        METRICS["redundant"]   = 0


        self.cam_raw_count     = 0   # cada nodo detectado por take_picture()
        self.cam_poi_matches   = 0   # cuántos de esos nodes eran PoIs
        # Flags de ejecución
        self._executed = {
            "handle_timer.assign": False,
            "handle_packet.HELLO": False,
            "handle_packet.DELIVER": False,
        }
        # --------------------------------
    
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
        self.log.info(f"✅ First ‘assign’ timer scheduled for t={next_t:.2f}s")
        self.log.debug(f"⏱️ Scheduled first 'assign' at t={next_t:.2f}")

    def handle_telemetry(self, telemetry: Telemetry) -> None: # lo que hace es imprimir posicion y a que waypoint se dirige
        self.log.debug(f"📡 Telemetry: pos={telemetry.current_position}, idle={self.mission.is_idle}")
        if not self.mission.is_idle:
            wp = self.mission.current_waypoint
            self.log.debug(f"🛰️ EQC moving towards waypoint {wp}")
            # INFO sólo cuando cambiamos de waypoint
            if wp != self._last_wp:
                self.log.info(f"🛰️ EQC rumbo al waypoint en {wp}")
                self._last_wp = wp

    def handle_timer(self, timer: str) -> None: # lo que hace es actualizar self.pending con las coordenadas detectadas
        self.log.debug(f"handle_timer invoked with timer='{timer}'")
        if timer == "assign":
            self._executed["handle_timer.assign"] = True
            now = self.provider.current_time()
            self.log.info("*"*40 + f" t={now:.2f}s " + "*"*40)
            ########
            self.log.info(f"⚙️  EQC handle_timer('assign') @ t={now:.2f}")
            detected = self.camera.take_picture()
            # Métrica raw
            self.cam_raw_count += len(detected)
            self.log.info(f"⚙️  assign @ t={now:.2f}: {len(detected)} nodos detectados")

            # Log raw detections
            for node in detected:
                pos = node["position"]
                self.log.debug(f"   📸 Raw detection at {pos}")

            # Filtrar PoIs
            new_cnt = 0
            eps = 0.2
            for poi in POIS:
                px, py = poi["coord"]
                for node in detected:
                    self.log.debug(f"   Raw node: {node!r}")
                    x, y, z = node["position"]
                    if abs(x-px)<eps and abs(y-py)<eps and abs(z-0.0)<eps:
                        # Métrica de match válido
                        label = poi["label"]
                        if label not in self.detect_ts:
                            self.cam_poi_matches += 1
                            self.detect_ts[label] = now
                            self.pending.append(poi)
                            new_cnt += 1
                            self.log.info(f"🔍 {label} detectado @ {poi['coord']} t={now:.2f}")
                        break
            #for node in detected:
            #    self.log.debug(f"   Raw node: {node!r}")
            #    pid = node.get("id")
            #    if pid and pid not in self.detect_ts:
            #        # es un PoI nuevo
            #        self.cam_poi_matches += 1
            #        self.detect_ts[pid] = now
            #        # busco el objeto completo en POIS
            #        poi = next(p for p in POIS if p.get("label")==pid or p.get("id")==pid)
            #        self.pending.append(poi)
            #        new_cnt += 1
            #        self.log.info(f"🔍 {pid} detectado @ {poi['coord']} t={now:.2f}")

            self.log.debug(f"🗂️ pending size /relacionado con new_cnt: {len(self.pending)} (+{new_cnt})")

            # Reprogramar
            next_t = now + 1
            self.provider.schedule_timer("assign", next_t)
            self.log.debug(f"⏱️ Rescheduled 'assign' at t={next_t:.2f}")

    def handle_packet(self, message: str) -> None: #se activa con HELLO o deliver, actualiza vqc states, pendindg      y en deliver
        self.log.debug(f"📥 [RAW] handle_packet recibido: {message}")
        msg = json.loads(message)
        t = msg.get("type")
        vid = msg["v_id"]
        # Si aún no tenemos estado de este VQC y el mensaje no es HELLO, lo ignoramos
        if t != "HELLO" and vid not in self.vqc_states:
            self.log.warning(f"Ignorando {t} de VQC-{vid} (no hay HELLO previo)")
            return
        if t == "HELLO":
            self._executed["handle_packet.HELLO"] = True
            free = msg["huecos"]
            pos = tuple(msg["position"])
            self.log.info(f"📩 HELLO from VQC-{vid}: free={free}, pos={pos}")
            #before = len(self.pending)
            #self.pending = [p for p in self.pending if p["label"] not in visited]
            #self.log.debug(f"🗑️ pending filtered: {before}→{len(self.pending)}")
            self.vqc_states[vid] = {"huecos": free, "pos": pos}
            if free <= 0:
                    self.log.debug(f"→ VQC-{vid} buffer FULL tras assign")

            # CHANGED: ahora respondemos HELLO_ACK
            ack = {"type": "HELLO_ACK", "v_id": vid}
            cmd_ack = CommunicationCommand(
                CommunicationCommandType.SEND,
                json.dumps(ack),
                vid
            )
            self.provider.send_communication_command(cmd_ack)
            self.log.info(f"📣 EQC envió HELLO_ACK a VQC-{vid}")

        elif t == "DELIVER":
            self._executed["handle_packet.DELIVER"] = True
            now = self.provider.current_time()
            vid = msg["v_id"]
            delivered = msg.get("pids", [])  
            self.log.info(f"📥 DELIVER from VQC-{vid}: {delivered}")
            # Métricas y filtrado
            for entry in delivered:
                label = entry.get("label")
                poi_id = entry.get("id")
                if label is None or poi_id is None:
                    self.log.warning(f"DELIVER malformed: {entry!r}")
                    continue
                # 1) “Pop” usando el mismo label que guardamos en assign_times
                t0 = self.assign_times.pop(label, None)
                if t0 is not None:
                    latency = now - t0
                    self.latencies.append((label, latency))
                    self.assign_success += 1
                elif label not in METRICS["unique_ids"]:
                    METRICS["unique_ids"].add(label)
                    self.log.debug(f"ℹ️ First auto‐deliver for {label}")
                else:
                    self.redundant_delivers += 1
                    METRICS["redundant"] += 1
                    self.log.debug(f"⚠️ Redundant DELIVER for {label}")

                # 2) Métrica de cobertura
                elapsed = now - self.start_time
                self.coverage_timeline.append((elapsed, len(METRICS["unique_ids"])))

            self.log.debug(f"🧮 Metrics: unique={len(METRICS['unique_ids'])}, redundant={METRICS['redundant']}")
            delivered_labels = [e["label"] for e in delivered]
            self.log.debug(f"DELIVER recibido de VQC-{vid}: {delivered_labels}")
            self.pending =[
                p for p in self.pending
                if p["label"] not in delivered_labels
            ]
            # 2) Enviar ACK de entrega SOLO a ese V-QC
            ack_payload = {
                "type": "DELIVER_ACK",
                "v_id": vid,
                "pids": [entry["id"] for entry in delivered]
            }
            cmd_ack = CommunicationCommand(
                CommunicationCommandType.SEND,
                json.dumps(ack_payload),
                vid
            )
            self.provider.send_communication_command(cmd_ack)
            self.log.info(f"📣 Enviado DELIVER_ACK a VQC-{vid}: {ack_payload['pids']}")



            self.assign_to_vqcs()

    def assign_to_vqcs(self) -> None:
        now = self.provider.current_time()
        self.log.debug(f"🔍 assign_to_vqcs: pending={len(self.pending)}, states={self.vqc_states}")
        for vid, st in self.vqc_states.items():
            
            free, pos = st["huecos"], st["pos"]
            self.log.debug(f"→ VQC-{vid} state: free={free}, pos={pos}")
            if free <= 0:
                self.log.debug(f"→ VQC-{vid} no free slots")
                continue
##########unicon dos conjuntos, passa informacoes novas
###no sobrecargar buffer, no dar mas tareas. heuristaica de blanceamiento de carga
            scored = []
            for poi in self.pending:
                dist = max(1e-6, math.hypot(pos[0]-poi["coord"][0], pos[1]-poi["coord"][1]))
                score = poi["urgency"]/dist
                scored.append((score, poi))
                self.log.debug(f"    ⋅ {poi['label']} urg={poi['urgency']} dist={dist:.2f} score={score:.2f}")

            scored.sort(key=lambda x: x[0], reverse=True)
            limit = min(free, MAX_ASSIGN_PER_ENCOUNTER)

            to_assign = [p for _, p in scored[:limit]]
            if not to_assign:
                self.log.debug(f"→ No PoIs for VQC-{vid}")
                continue

            # 3) MARCAR “IN-FLIGHT”: quitar inmediatamente de pending
            for p in to_assign:
                self.assign_times[p["label"]] = now
                self.pending.remove(p)
            self.assign_count += len(to_assign)

            payload = {
                "type":"ASSIGN","v_id":vid,
                "pois":[{"label":p["label"],"coord":p["coord"],"urgency":p["urgency"],"ts":self.detect_ts[p["label"]]} for p in to_assign]
            }
            self.log.debug(f"🚀 ASSIGN payload for VQC-{vid}: {payload}")

            cmd = CommunicationCommand(CommunicationCommandType.SEND, json.dumps(payload), vid)
            self.provider.send_communication_command(cmd)
            self.log.info(f"🚀 ASSIGN {len(to_assign)} to VQC-{vid}: {[p['label'] for p in to_assign]}")
                # ——> ACTUALIZAR huecos
            self.vqc_states[vid]["huecos"] -= len(to_assign)
            self.log.debug(f"→ Después de ASSIGN, VQC-{vid} tiene {self.vqc_states[vid]['huecos']} huecos")


    def finish(self) -> None:
        # calcular latencia promedio ignorando ceros
        valid_latencies = [l for _, l in self.latencies if l > 0]
        avg_latency = (
            sum(valid_latencies) / len(valid_latencies)
            if valid_latencies else float("nan")
        )

        self.log.info(f"✔️ assign_success    = {self.assign_success}")
        self.log.info(f"ℹ️ redundant_delivers = {self.redundant_delivers}")
        self.log.info(f"⏱️ avg_latency       = {avg_latency:.3f}s")
        # ... resto del finish ...

        total_time = self.provider.current_time() - self.start_time
        unique = len(METRICS["unique_ids"])
        redundant = METRICS["redundant"]
        success = self.assign_success
        assigns = self.assign_count
        avg_latency = sum(l for _, l in self.latencies) / len(self.latencies) if self.latencies else float('nan')
        discovery_rate = unique / total_time if total_time>0 else float('nan')
        success_rate   = success / assigns if assigns>0 else float('nan')

        self.log.info(f"✅ EQC finished. Unique={unique}, redundant={redundant}")
        self.log.info(f"   Assigns sent={assigns}, successful delivers={success} (rate={success_rate:.2f})")
        self.log.info(f"   Avg. latency={avg_latency:.2f}s, discovery rate={discovery_rate:.2f} PoIs/s")

        self.log.info(f"📷 Cámara hizo {self.cam_raw_count} detecciones totales, "
                      f"{self.cam_poi_matches} coincidencias con PoIs")
        # Métodos invocados
        never_called = [k for k,v in self._executed.items() if not v]
        if never_called:
            self.log.warning(f"⚠️ Métodos nunca ejecutados: {never_called}")