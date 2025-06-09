import pandas as pd

# 1) Leer los tres CSV originales
df_greedy = pd.read_csv('experiment_results_1greedy.csv')
df_rr     = pd.read_csv('experiment_results2round_robin.csv')
df_lb     = pd.read_csv('experiment_results3load_balancing.csv')

# 2) Filtrar al “slice” que tú quieras comparar.
#    Aquí tomamos: buffer_size=5, speed=5.0, camera_reach=15.0
cond_g = (
    (df_greedy['buffer_size']  == 5) &
    (df_greedy['speed']        == 5.0) &
    (df_greedy['camera_reach'] == 15.0)
)
cond_r = (
    (df_rr['buffer_size']      == 5) &
    (df_rr['speed']            == 5.0) &
    (df_rr['camera_reach']     == 15.0)
)
cond_l = (
    (df_lb['buffer_size']      == 5) &
    (df_lb['speed']            == 5.0) &
    (df_lb['camera_reach']     == 15.0)
)

df_g = df_greedy[cond_g]
df_r = df_rr[cond_r]
df_l = df_lb[cond_l]

# 3) Para cada política, agrupar por num_pois y calcular mean/std de assign_success
sum_g = df_g.groupby('num_pois')['assign_success'].agg(['mean','std']).reset_index()
sum_r = df_r.groupby('num_pois')['assign_success'].agg(['mean','std']).reset_index()
sum_l = df_l.groupby('num_pois')['assign_success'].agg(['mean','std']).reset_index()

# 4) Renombrar columnas para distinguir en el CSV final
sum_g = sum_g.rename(columns={'mean':'greedy_mean', 'std':'greedy_std'})
sum_r = sum_r.rename(columns={'mean':'rr_mean',     'std':'rr_std'})
sum_l = sum_l.rename(columns={'mean':'lb_mean',     'std':'lb_std'})

# 5) Hacer un “merge” secuencial de los tres DataFrames sobre la clave num_pois
df01 = pd.merge(sum_g, sum_r, on='num_pois', how='outer')
df_summary = pd.merge(df01, sum_l, on='num_pois', how='outer')

# 6) Guardar el CSV de resumen
df_summary.to_csv('summary_assign_success.csv', index=False, float_format='%.2f')

print("► CSV de resumen guardado en summary_assign_success.csv:")
print(df_summary)
