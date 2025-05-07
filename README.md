# GrADyS UAV Swarm Simulation

This repository implements a coordinated exploration scenario using a heterogeneous swarm of quadcopters (E-QC and V-QCs) to discover and catalog Points of Interest (PoIs) within a square mission area. The goal is to maximize unique PoI identifications while minimizing redundant visits.

## Mission Overview

- **Area**: A square of size L×L contains multiple unmapped PoIs, each with a unique ID, (x,y) coordinates and an urgency level (1 = low, 3 = high).  
- **Exploration Quadcopter (E-QC)**  
  - Patrols predefined waypoints in a continuous loop.  
  - Uses an onboard camera to detect PoIs within its field of view.  
  - Logs detection timestamps and prioritizes PoIs by urgency and proximity.  
  - Sends `ASSIGN` messages to Visitor QCs.  
- **Visiting Quadcopters (V-QCs)**  
  - Roam randomly until they receive an `ASSIGN` payload (up to M PoIs).  
  - Fly to each assigned PoI, read its ID, and report back via `DELIVER` messages.  
  - Maintain a small buffer of discovered-but-not-yet-delivered IDs.  
- **Static PoI Nodes**  
  - Represent PoIs in the simulation; do not move or communicate.  

## Repository Structure
- **config.py**  
  Contains global parameters: area size, camera/detection ranges, buffer limits, simulation duration and PoI definitions.  
- **poi_protocol.py**  
  Defines the `POIProtocol` class (static PoI node stub).  
- **eqc_protocol.py**  
  Implements `EQCProtocol`: area patrol, onboard camera handling, PoI filtering, and ASSIGN message coordination.  
- **vqc_protocol.py**  
  Implements `VQCProtocol`: random roaming, ASSIGN reception, PoI visitation, local detection, and DELIVER reporting.  
- **run_simulation.py**  
  Main script that sets up simulation handlers (communication, timer, mobility, visualization), initializes all nodes, and starts the run.  

#Configuration
Adjust constants in config.py to experiment with:

L: Side length of the mission area.

R_CAMERA / R_DETECT: Sensor ranges for E-QC and V-QC.

M: Maximum PoIs per V-QC assignment buffer.

DURATION: Total simulation time in seconds.

POIS: Add, remove or modify PoI entries (ID, label, coords, urgency).

How It Works
E-QC loops through waypoints; every second it takes a “picture” and filters for new PoIs within camera range.

Detected PoIs are queued with their detection timestamp.

V-QCs periodically broadcast HELLO messages advertising free buffer slots.

E-QC uses a simple urgency/proximity heuristic (urgency ÷ distance) to assign PoIs.

V-QCs receive ASSIGN, interrupt random roaming, fly to each target, detect locally, and send DELIVER messages within communication range.

E-QC aggregates delivered IDs into unique and redundant counts.

