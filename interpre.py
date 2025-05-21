import re
import os

# Cambia aquí si el nombre real difiere
log_file = r'C:\Users\User\Desktop\gradysUAV\camera_extension_problem\sim.log'

# Verifico que exista realmente
if not os.path.isfile(log_file):
    raise FileNotFoundError(f"No he encontrado el log en: {log_file}\nArchivos en carpeta:\n" +
                            "\n".join(os.listdir(os.path.dirname(log_file))))

# Cargo todas las líneas
with open(log_file, 'r', encoding='utf-8') as f:
    lines = f.readlines()

def print_section(title, entries):
    print(f"\n=== {title} ===")
    for e in entries:
        print(e.strip())

# 1) assign_success y eventos ASSIGN
for l in lines:
    if 'assign_success' in l:
        print_section("assign_success", [l])
        break

assign_events = []
for l in lines:
    if 'ASSIGN' in l:
        m = re.search(r'(\d{2}:\d{2}:\d{2},\d{3}).*ASSIGN\s+\d+\s+to\s+(\S+):\s*(\[[^\]]+\])', l)
        if m:
            assign_events.append(f"{m.group(1)} → {m.group(2)}: {m.group(3)}")
print_section("Eventos ASSIGN", assign_events)

# 2) pending size y new_cnt
pendings = []
for l in lines:
    if 'pending size' in l:
        m = re.search(r'(\d{2}:\d{2}:\d{2},\d{3}).*pending size\s*=\s*(\d+)\s*\+new_cnt\s*=\s*([\+\-]?\d+)', l)
        if m:
            pendings.append(f"{m.group(1)} → size={m.group(2)}, new_cnt={m.group(3)}")
print_section("pending size", pendings)

# 3) redundant_delivers y eventos DELIVER enviado
redundant = [l for l in lines if 'redundant_delivers' in l]
print_section("redundant_delivers", redundant)

deliver_events = [l for l in lines if '📤 DELIVER enviado' in l]
print_section("Eventos DELIVER enviado", deliver_events)

# 4) avg_latency
latency = [l for l in lines if 'avg_latency' in l or 'Avg. latency' in l]
print_section("avg_latency y latencies", latency)

# 5) Resumen final EQC (Unique / redundant)
summary = [l for l in lines if 'EQC finished' in l or 'unique=' in l.lower() or 'redundant=' in l.lower()]
print_section("Resumen final EQC", summary)

# 6) Detecciones de Cámara
camera = [l for l in lines if 'Cámara' in l or 'detecciones' in l]
print_section("Detecciones de Cámara", camera)

# 7) Discovery rate
disc = [l for l in lines if 'discovery rate' in l]
print_section("Discovery rate", disc)

# 8) Assigns sent y successful delivers
assigns = [l for l in lines if 'Assigns sent' in l or 'successful delivers' in l]
print_section("Assigns sent y successful delivers", assigns)
