
"""
Simulation parameters and PoI definitions:
- Area size, camera and detection ranges.
- Buffer limits and simulation duration.
- List of PoIs with ID, label, coordinates, and urgency.
- Global metrics structures.
"""
import random
from typing import List, Dict, Tuple

# Dimensions and ranges
L = 50.0                 
R_CAMERA = 15            
R_DETECT = 8.0            
R_COMM = 7.0              
POIS: List[Dict] = []   

# Buffer and duration
M = 5                    
DURATION = 100          
NUM_VQCS = 5              
MAX_ASSIGN_PER_ENCOUNTER = 3
EQC_SPEED = 5.0               
VQC_SPEED = 5.0    


# Métricas globales
METRICS = {
    "unique_ids": set(),
    "redundant":  0
}
MAX_POIS = 100

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
