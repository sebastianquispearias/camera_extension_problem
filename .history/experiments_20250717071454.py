
import subprocess
import re
import csv
import itertools

seeds = list(range(100, 110))  
num_pois_list     = [50, 100, 200]      # densidades de PoIs
num_vqcs_list     = [5, 10, 20]         # número de V-QCs
buffer_sizes_list = [3, 5, 10]          # tamaño de buffer M
speeds_list       = [5.0]               # velocidad de vuelo (m/s)
camera_reaches    = [10.0, 15.0, 20.0]  # alcance oblicuo de la cámara


with open('experiment_results.csv', 'w', newline='', encoding='utf-8') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow([
        'seed','num_pois','num_vqcs','buffer_size',
        'speed','camera_reach',
        'assign_success','redundant_delivers',
        'avg_latency','discovery_rate',
        'global_score','cam_matches','assigns_sent','assign_rate'  # <<< CAMBIO
    ])

    for seed in seeds:              
        for pois, vqcs, buf, spd, reach in itertools.product(
            num_pois_list, num_vqcs_list, buffer_sizes_list, speeds_list, camera_reaches
        ):
            cmd = (
                f"python run_simulation.py"
                f" --seed {seed}"                   
                f" --num_pois {pois}"
                f" --num_vqcs {vqcs}"
                f" --buffer_size {buf}"
                f" --speed {spd}"
                f" --camera_reach {reach}"
            )
            # … resto idéntico …
            print(f"\n🏃 Ejecutando: {cmd}")
            proc = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            log = proc.stdout + proc.stderr

            # Extraer métricas con regex
            a_s = re.search(r'assign_success\s*=\s*(\d+)', log)
            r_d = re.search(r'redundant_delivers\s*=\s*(\d+)', log)
            a_l = re.search(r'avg_latency\s*=\s*([\d\.]+)s', log)
            d_r = re.search(r'discovery rate=\s*([\d\.]+)\s*PoIs/s', log)

            gms = re.search(r'Global mission score\s*=\s*([\d\.]+)', log)
            global_score = float(gms.group(1)) if gms else ''

            # Camera matches
            cam = re.search(
                r'Cámara hizo\s*\d+\s*detecciones.*?,\s*(\d+)\s*coincidencias',
                log
            )
            cam_matches = int(cam.group(1)) if cam else ''

            # Assigns sent & assign rate (successful delivers ya ≡ assign_success)
            ar = re.search(
                r'Assigns sent=(\d+), successful delivers=(\d+) \(rate=([\d\.]+)\)',
                log
            )
            assigns_sent = int(ar.group(1)) if ar else ''
            assign_rate  = float(ar.group(3)) if ar else ''
            
            writer.writerow([
                seed, pois, vqcs, buf, spd, reach,
                int(a_s.group(1)) if a_s else '',
                int(r_d.group(1)) if r_d else '',
                float(a_l.group(1)) if a_l else '',
                float(d_r.group(1)) if d_r else '',
                global_score,    # <<< CAMBIO
                cam_matches,     # <<< CAMBIO
                assigns_sent,    # <<< CAMBIO
                assign_rate      # <<< CAMBIO
            ])

            print(
            print(
                f"→ seed={seed}, Pois={pois}, VQCs={vqcs}, M={buf}, "
                f"speed={spd}, reach={reach} → "
                f"assign_success={a_s.group(1) if a_s else '?'}  "
                f"redundant_delivers={r_d.group(1) if r_d else '?'}  "
                f"global_score={global_score}  "  # <<< OPCIONAL
                f"assign_rate={assign_rate}"       # <<< OPCIONAL
            )