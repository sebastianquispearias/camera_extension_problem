
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
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    fmt = logging.Formatter("%(asctime)s %(name)-10s %(levelname)-7s %(message)s")
    ch = logging.StreamHandler(); ch.setFormatter(fmt); root.addHandler(ch)
    fh = logging.FileHandler("sim.log","w","utf-8"); fh.setFormatter(fmt); root.addHandler(fh)

    sim_cfg = SimulationConfiguration(duration=DURATION, debug=True, real_time=True)
    builder = SimulationBuilder(sim_cfg)

    # Exploration quadcopter
    builder.add_node(EQCProtocol, (0.0, 0.0, 7.0))
    # Visiting quadcopters
    for _ in range(NUM_VQCS):
        x, y = random.uniform(0, L), random.uniform(0, L)
        builder.add_node(VQCProtocol, (x, y, 4.0))
    # Static PoI nodes
    for poi in POIS:
        x, y = poi["coord"]
        builder.add_node(POIProtocol, (x, y, 0.0))

    # Simulation handlers
    medium = CommunicationMedium(transmission_range=R_COMM)
    builder.add_handler(CommunicationHandler(medium))
    builder.add_handler(TimerHandler())
    builder.add_handler(MobilityHandler(MobilityConfiguration(default_speed=10.0)))
    builder.add_handler(VisualizationHandler())

    sim = builder.build()
    sim.start_simulation()
