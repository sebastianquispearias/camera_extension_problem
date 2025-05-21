# analysis.py

import pandas as pd
import matplotlib.pyplot as plt

# Carga de resultados
df = pd.read_csv('experiment_results.csv')

# Gráfico: assign_success vs num_pois por buffer_size
plt.figure()
for buf in sorted(df['buffer_size'].unique()):
    sub = df[df['buffer_size'] == buf]
    plt.plot(sub['num_pois'], sub['assign_success'], marker='o', label=f'buffer={buf}')
plt.xlabel('Número de PoIs')
plt.ylabel('assign_success')
plt.title('assign_success vs Número de PoIs')
plt.legend()
plt.grid(True)
plt.show()

# --- Puedes crear otros gráficos iguales para:
#    - redundant_delivers
#    - avg_latency
#    - discovery_rate
# Cambia el campo en plt.plot(...) y en ylabel() según la métrica.
