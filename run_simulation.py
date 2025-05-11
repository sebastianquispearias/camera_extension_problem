"""
run_simulation.py
Main script to set up and run the simulation:
- Configures communication, timer, mobility, and visualization handlers.
- Initializes E-QC, V-QCs, and PoI nodes.
- Starts the simulation.
"""

import logging
import random

from gradysim.simulator.handler.communication import CommunicationHandler, CommunicationMedium
from gradysim.simulator.handler.timer import TimerHandler
from gradysim.simulator.handler.mobility import MobilityHandler, MobilityConfiguration
from gradysim.simulator.handler.visualization import VisualizationHandler
from gradysim.simulator.simulation import SimulationBuilder, SimulationConfiguration

from config import L, R_COMM, DURATION, NUM_VQCS, POIS
from poi_protocol import POIProtocol
from eqc_protocol import EQCProtocol
from vqc_protocol import VQCProtocol

if __name__ == "__main__":
    # Logger setup
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    fmt = logging.Formatter("%(asctime)s %(name)-12s %(levelname)-8s %(message)s")
    ch = logging.StreamHandler(); ch.setFormatter(fmt); root.addHandler(ch)
    fh = logging.FileHandler("sim.log","w","utf-8"); fh.setFormatter(fmt); root.addHandler(fh)

    root.info(f"✅ Simulation start — duration={DURATION}s, VQCs={NUM_VQCS}, area={L}×{L}")

    sim_cfg = SimulationConfiguration(duration=DURATION, debug=False, real_time=False)
    builder = SimulationBuilder(sim_cfg)

    builder.add_node(EQCProtocol, (0.0, 0.0, 7.0))
    root.info("➕ Added EQCProtocol at (0,0,7)")

    for i in range(NUM_VQCS):
        pos = (random.uniform(0,L), random.uniform(0,L), 4.0)
        builder.add_node(VQCProtocol, pos)
        root.info(f"➕ Added VQCProtocol #{i+1} at {pos}")

    for poi in POIS:
        builder.add_node(POIProtocol, (poi["coord"][0], poi["coord"][1], 0.0))
    root.info(f"➕ Added {len(POIS)} POIProtocol nodes")

    medium = CommunicationMedium(transmission_range=R_COMM)
    builder.add_handler(CommunicationHandler(medium))
    builder.add_handler(TimerHandler())
    builder.add_handler(MobilityHandler(MobilityConfiguration(default_speed=5.0)))
    builder.add_handler(VisualizationHandler())
    root.info("🔧 Handlers added")

    sim = builder.build()
    root.info("▶️ Starting simulation")
    sim.start_simulation()
    root.info("🏁 Simulation complete")
