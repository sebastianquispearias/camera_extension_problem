#!/usr/bin/env python3
"""
run_simulation.py
Main script to set up and run the simulation:
- Configures communication, timer, mobility, and visualization handlers.
- Initializes E-QC, V-QCs, and PoI nodes.
- Starts the simulation.

Ahora parametrizado para:
  --num-pois    : número de PoIs
  --area        : tamaño (L×L) del área de simulación
  --num-uavs    : número de UAVs visitadores (VQCs)
  --buffer      : tamaño del buffer M en cada VQC
"""

import logging
import random
import argparse
import config                # importamos config para sobreescribir valores
from gradysim.simulator.handler.communication import CommunicationHandler, CommunicationMedium
from gradysim.simulator.handler.timer import TimerHandler
from gradysim.simulator.handler.mobility import MobilityHandler, MobilityConfiguration
from gradysim.simulator.handler.visualization import VisualizationHandler
from gradysim.simulator.simulation import SimulationBuilder, SimulationConfiguration

from poi_protocol import POIProtocol
from eqc_protocol import EQCProtocol
from vqc_protocol import VQCProtocol

def parse_args():
    p = argparse.ArgumentParser(description="Simulación UAV swarms con parámetros variables")
    p.add_argument("--num-pois",  type=int,   default=len(config.POIS),
                   help="Cantidad de PoIs (default=valor en config.POIS)")
    p.add_argument("--area",      type=float, default=getattr(config, "L", 100.0),
                   help="Tamaño L del área cuadrada (default=config.L)")
    p.add_argument("--num-uavs",  type=int,   default=getattr(config, "NUM_VQCS", 3),
                   help="Número de VQCs (default=config.NUM_VQCS)")
    p.add_argument("--buffer",    type=int,   default=getattr(config, "M", 5),
                   help="Tamaño del buffer en VQC (M) (default=config.M)")
    return p.parse_args()

if __name__ == "__main__":
    args = parse_args()

    # Sobrescribo en config para que tus protocolos lean los nuevos valores
    config.L         = args.area
    config.NUM_VQCS  = args.num_uavs
    config.M         = args.buffer

    # Genero dinamicamente la lista de PoIs
    pois = []
    for i in range(args.num_pois):
        x = random.uniform(0, config.L)
        y = random.uniform(0, config.L)
        pois.append({"coord": (x, y)})

    # === setup de logging (UNCHANGED) ===
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    fmt = logging.Formatter("%(asctime)s %(name)-12s %(levelname)-8s %(message)s")
    ch = logging.StreamHandler(); ch.setFormatter(fmt); root.addHandler(ch)
    fh = logging.FileHandler("sim.log","w","utf-8"); fh.setFormatter(fmt); root.addHandler(fh)

    root.info(f"✅ Simulation start — duration={config.DURATION}s, VQCs={config.NUM_VQCS}, area={config.L}×{config.L}, buffer={config.M}, pois={len(pois)}")

    # === construcción de la simulación (UNCHANGED salvo los valores) ===
    sim_cfg = SimulationConfiguration(duration=config.DURATION, debug=True, real_time=False)
    builder = SimulationBuilder(sim_cfg)

    builder.add_node(EQCProtocol, (0.0, 0.0, 7.0))
    root.info("➕ Added EQCProtocol at (0,0,7)")

    for i in range(config.NUM_VQCS):
        pos = (random.uniform(0, config.L), random.uniform(0, config.L), 4.0)
        builder.add_node(VQCProtocol, pos)
        root.info(f"➕ Added VQCProtocol #{i+1} at {pos}")

    for poi in pois:
        builder.add_node(POIProtocol, (poi["coord"][0], poi["coord"][1], 0.0))
    root.info(f"➕ Added {len(pois)} POIProtocol nodes")

    medium = CommunicationMedium(transmission_range=config.R_COMM)
    builder.add_handler(CommunicationHandler(medium))
    builder.add_handler(TimerHandler())
    builder.add_handler(MobilityHandler(MobilityConfiguration(default_speed=8.0)))
    builder.add_handler(VisualizationHandler())
    root.info("🔧 Handlers added")

    # === arrancar ===
    sim = builder.build()
    root.info("▶️ Starting simulation")
    sim.start_simulation()
    root.info("🏁 Simulation complete")
