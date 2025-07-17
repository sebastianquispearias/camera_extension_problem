#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
analisis_experimentos.py

Carga un CSV de resultados de simulación ('experiment_results.csv'),
calcula medias y desviaciones estándar, genera tablas LaTeX y
gráficos PNG para incluir luego en Overleaf.
"""

import pandas as pd
import matplotlib.pyplot as plt
import os

# --- Configuraciones iniciales ---
CSV_PATH = "experiment_results1semilla.csv"   # pon aquí la ruta a tu archivo CSV
OUTPUT_DIR = "resultados_analisis"    # carpeta donde saldrán tablas e imágenes

# Asegúrate de crear la carpeta de salida
os.makedirs(OUTPUT_DIR, exist_ok=True)

# --- Carga de datos ---
df = pd.read_csv(CSV_PATH)

# --- 1. Estadísticas globales ---
metrics = ["assign_success", "avg_latency", "discovery_rate"]
global_stats = df[metrics].agg(['mean', 'std']).transpose().rename(
    columns={'mean':'Media','std':'DesvEst'})

# Guardar tabla global en LaTeX
global_tex = global_stats.to_latex(
    os.path.join(OUTPUT_DIR, "global_stats.tex"),
    float_format="%.2f",
    caption="Estadísticas globales de las métricas (Greedy)",
    label="tab:global-stats"
)

# --- 2. Por-factor: medias, desviaciones y gráficos ---
factors = ["buffer_size", "num_vqcs", "num_pois", "camera_reach"]

for factor in factors:
    # Agrupa y calcula
    grp = df.groupby(factor)["assign_success"].agg(['mean','std']).reset_index()
    grp = grp.rename(columns={'mean':'Media','std':'DesvEst'})

    # 2.1 Exportar tabla LaTeX
    tex_path = os.path.join(OUTPUT_DIR, f"{factor}_stats.tex")
    grp.to_latex(
        tex_path,
        index=False,
        float_format="%.2f",
        caption=f"Media y desviación estándar de assign\\_success por {factor}",
        label=f"tab:{factor}-stats"
    )

    # 2.2 Generar gráfico de barras con error bars
    plt.figure(figsize=(6,4))
    plt.bar(grp[factor].astype(str), grp["Media"], yerr=grp["DesvEst"], capsize=5)
    plt.xlabel(factor.replace("_"," ").title())
    plt.ylabel("Media assign_success")
    plt.title(f"Efecto de {factor.replace('_',' ')} en assign_success")
    plt.tight_layout()

    # Guardar imagen
    img_path = os.path.join(OUTPUT_DIR, f"{factor}_assign_success.png")
    plt.savefig(img_path, dpi=300)
    plt.close()

print(f"Listo. Tablas y gráficos en la carpeta: {OUTPUT_DIR}")
