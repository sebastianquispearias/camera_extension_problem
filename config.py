
"""
config.py
Simulation parameters and PoI definitions:
- Area size, camera and detection ranges.
- Buffer limits and simulation duration.
- List of PoIs with ID, label, coordinates, and urgency.
- Global metrics structures.
"""

# Dimensions and ranges
L = 50.0                   # area size
R_CAMERA = 15              # camera reach
R_DETECT = 5.0             # V-QC detection range
R_COMM = 10.0              # communication range

# Buffer and duration
M = 5                      # maximum PoI buffer per V-QC
DURATION = 60              # simulation duration (seconds)
NUM_VQCS = 5               # number of V-QCs

# PoI definitions
POIS = [
    {"id": "ABC-0001", "label": "POI-1",  "coord": (5.0,  5.0),  "urgency": 3},
    {"id": "ABC-0002", "label": "POI-2",  "coord": (10.0, 40.0), "urgency": 2},
    {"id": "ABC-0003", "label": "POI-3",  "coord": (15.0, 25.0), "urgency": 1},
    {"id": "ABC-0004", "label": "POI-4",  "coord": (20.0, 10.0), "urgency": 3},
    {"id": "ABC-0005", "label": "POI-5",  "coord": (25.0, 35.0), "urgency": 2},
    {"id": "ABC-0006", "label": "POI-6",  "coord": (30.0, 15.0), "urgency": 1},
    {"id": "ABC-0007", "label": "POI-7",  "coord": (35.0, 45.0), "urgency": 3},
    {"id": "ABC-0008", "label": "POI-8",  "coord": (40.0, 20.0), "urgency": 2},
    {"id": "ABC-0009", "label": "POI-9",  "coord": (45.0, 30.0), "urgency": 1},
    {"id": "ABC-0010","label": "POI-10","coord": (12.5,12.5), "urgency": 2},
    {"id": "ABC-0011","label": "POI-11","coord": (6.0, 22.5), "urgency": 1},
    {"id": "ABC-0012","label": "POI-12","coord": (2.5, 16.0), "urgency": 2},
    {"id": "ABC-0013","label": "POI-13","coord": (23.5,27.5), "urgency": 3},
    {"id": "ABC-0014","label": "POI-14","coord": (16.5, 5.0), "urgency": 1},
    {"id": "ABC-0015","label": "POI-15","coord": (11.0,38.5), "urgency": 2},
    {"id": "ABC-0016","label": "POI-16","coord": (30.5, 6.0), "urgency": 3},
    {"id": "ABC-0017","label": "POI-17","coord": (22.0,19.0), "urgency": 1},
    {"id": "ABC-0018","label": "POI-18","coord": (9.0,44.0),  "urgency": 2},
    {"id": "ABC-0019","label": "POI-19","coord": (36.0,33.0), "urgency": 3},
    {"id": "ABC-0020","label": "POI-20","coord": (14.0, 7.5), "urgency": 1},
    {"id": "ABC-0021","label": "POI-21","coord": (27.5,25.0),"urgency": 2},
    {"id": "ABC-0022","label": "POI-22","coord": (4.0,33.0),  "urgency": 3},
    {"id": "ABC-0023","label": "POI-23","coord": (47.5,10.0), "urgency": 2},
    {"id": "ABC-0024","label": "POI-24","coord": (20.0,40.0), "urgency": 1},
    {"id": "ABC-0025","label": "POI-25","coord": (35.0,15.0), "urgency": 3},
]

# Global metrics
METRICS = {"unique_ids": set(), "redundant": 0}
