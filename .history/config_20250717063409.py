git
"""
Simulation parameters and PoI definitions:
- Area size, camera and detection ranges.
- Buffer limits and simulation duration.
- List of PoIs with ID, label, coordinates, and urgency.
- Global metrics structures.
"""
import random
from typing import List, Dict, Tuple

#falta funcion para ver distancias entre eqc y vqc
# Dimensions and ranges
L = 50.0                 
R_CAMERA = 15            
R_DETECT = 8.0            
R_COMM = 10.0              
POIS: List[Dict] = []   

# Buffer and duration
M = 5                    
DURATION = 35          
NUM_VQCS = 5              
MAX_ASSIGN_PER_ENCOUNTER = 3
EQC_SPEED = 10.0               
VQC_SPEED = 25.0    


# Métricas globales
METRICS = {
    "unique_ids": set(),
    "redundant":  0
}
MAX_POIS = 100

URGENCY_WEIGHTS = {
    1: 0.2,   # low
    2: 0.5,   # medium
    3: 1.0    # high
}

# Posición inicial del EQC (w₀)
EQC_INIT_POS: Tuple[float,float,float] = (0.0, 0.0, 7.0)

# Waypoints de la patrulla del EQC
EQC_WAYPOINTS: List[Tuple[float,float,float]] = [
    EQC_INIT_POS,
    (L,    0.0, 7.0),
    (L,   10.0, 7.0),
    (0.0, 10.0, 7.0),
    (0.0, 20.0, 7.0),
    (L,   20.0, 7.0),
    (L,   30.0, 7.0),
    (0.0, 30.0, 7.0),
    (0.0, 40.0, 7.0),
    (L,   40.0, 7.0),
    (L,   L,    7.0),
    (0.0,  L,   7.0),
]

def get_pois(seed: int, n: int) -> List[Dict]:
    """
    Genera hasta MAX_POIS PoIs de forma reproducible con la misma seed,
    y devuelve los primeros n (nested sampling).
    Cada PoI tiene 'coord' (x,y) y 'urgency' (1–3).
    """
    rng = random.Random(seed)
    base: List[Dict] = []
    # 1) Creamos un pool de MAX_POIS: coords aleatorias + urgencia aleatoria
    for i in range(MAX_POIS):
        x = rng.uniform(0, L)
        y = rng.uniform(0, L)
        urg = rng.randint(1, 3)
        base.append({
            "id":      f"{seed:03d}-{i:03d}",
            "label":   f"POI-{i+1}",
            "coord":   (x, y),
            "urgency": urg
        })
    # 2) Tomamos los primeros n
    return base[:n]
