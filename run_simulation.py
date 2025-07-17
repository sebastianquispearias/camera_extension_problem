"""
run_simulation.py
Main script to set up and run the simulation:
- Configures communication, timer, mobility, and visualization handlers.
- Initializes E-QC, V-QCs, and PoI nodes.
- Starts the simulation.
"""
import logging
import random
import argparse                                       
import config     

from gradysim.simulator.handler.communication import CommunicationHandler, CommunicationMedium
from gradysim.simulator.handler.timer import TimerHandler
from gradysim.simulator.handler.mobility import MobilityHandler, MobilityConfiguration
from gradysim.simulator.handler.visualization import VisualizationHandler
from gradysim.simulator.simulation import SimulationBuilder, SimulationConfiguration

from poi_protocol import POIProtocol
from eqc_protocol import EQCProtocol
from vqc_protocol import VQCProtocol
from config import EQC_INIT_POS

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Ejecuta simulaciones con parámetros variables")
    parser.add_argument('--num_pois',      type=int,required=True,   choices=[50,100,200], help='Cantidad de PoIs a usar')
    parser.add_argument('--num_vqcs',      type=int,required=True,   choices=[5,10,20],     help='Número de V-QCs')
    parser.add_argument('--buffer_size',   type=int,required=True,   choices=[3,5,10],    help='Tamaño máximo de buffer M')
    parser.add_argument('--speed',         type=float,required=True, choices=[5.0,10.0],  help='Velocidad de vuelo (m/s)')
    parser.add_argument('--camera_reach',  type=float,required=True, choices=[10.0,15.0,20.0], help='Alcance oblicuo de la cámara')
    parser.add_argument('--seed',          type=int,required=True,help='Semilla para generar PoIs y posiciones iniciales')
 

    args = parser.parse_args()
    random.seed(args.seed)  
    config.POIS = config.get_pois(seed=args.seed, n=args.num_pois)
    config.NUM_VQCS   = args.num_vqcs    
    config.M          = args.buffer_size 
    config.R_CAMERA   = args.camera_reach
    mobility_speed    = args.speed      


    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    fmt = logging.Formatter("%(asctime)s %(name)-12s %(levelname)-8s %(message)s")
    ch = logging.StreamHandler(); ch.setFormatter(fmt); root.addHandler(ch)
    fh = logging.FileHandler("sim.log","w","utf-8"); fh.setFormatter(fmt); root.addHandler(fh)

    root.info(
        f"✅ Simulation start — seed={args.seed}, num_pois={len(config.POIS)}, "
        f"duration={config.DURATION}s, VQCs={config.NUM_VQCS}, area={config.L}×{config.L}, "
        f"speed={mobility_speed} m/s, camera_reach={config.R_CAMERA}"
    )

 #####################——— Construcción de la simulación ———
    sim_cfg = SimulationConfiguration(duration=config.DURATION, debug=False, real_time=True)
    builder = SimulationBuilder(sim_cfg)

    builder.add_node(EQCProtocol, EQC_INIT_POS)
    root.info("➕ Added EQCProtocol at (0,0,7)")
  # Añadimos VQCs con posiciones reproducibles
    for i in range(config.NUM_VQCS):
        pos = (random.uniform(0,config.L), random.uniform(0,config.L), 4.0)
        builder.add_node(VQCProtocol, pos)
        root.info(f"➕ Added VQCProtocol #{i+1} at {pos}")
# Añadimos PoIs
    for poi in config.POIS:
        builder.add_node(POIProtocol, (poi["coord"][0], poi["coord"][1], 0.0))
    root.info(f"➕ Added {len(config.POIS)} POIProtocol nodes")
 # ——— Handler
    medium = CommunicationMedium(transmission_range=config.R_COMM)
    builder.add_handler(CommunicationHandler(medium))
    builder.add_handler(TimerHandler())
    builder.add_handler(MobilityHandler(MobilityConfiguration(default_speed=mobility_speed))) 
    builder.add_handler(VisualizationHandler())
    root.info("🔧 Handlers added")
 # ——— Ejecución ———
    sim = builder.build()
    root.info("▶️ Starting simulation")
    sim.start_simulation()
    root.info("🏁 Simulation complete")
