import pandas as pd
import matplotlib.pyplot as plt
import os
import glob

# Ruta base para los archivos CSV
csv_dir = os.path.expanduser('~/pf_logs')

# Cargar datos de referencia de las balizas
beacon_ref_path = os.path.join(csv_dir, 'beacon_positions.csv')
beacon_refs = pd.read_csv(beacon_ref_path, index_col='beacon_id')

# Leer los tiempos de inicio EKF sin comentarios, CSV limpio
ekf_start_path = os.path.join(csv_dir, 'ekf_start_times.csv')
ekf_start_df = pd.read_csv(ekf_start_path)
ekf_start_dict = dict(zip(ekf_start_df['beacon_id'], ekf_start_df['ekf_start_time']))

# Buscar y ordenar todos los archivos pf_means_*.csv
csv_files = sorted(glob.glob(os.path.join(csv_dir, 'pf_means_*.csv')))

dfs = []
beacon_ids = []
for file in csv_files:
    # Ignorar líneas comentadas si las hay
    df = pd.read_csv(file, comment='#')
    dfs.append(df)
    beacon_id = int(os.path.splitext(os.path.basename(file))[0].split('_')[-1])
    beacon_ids.append(beacon_id)

# Cargar CSV con distribución de partículas (ignorando comentarios)
particle_dist_path = os.path.join(csv_dir, 'pf_particles.csv')
particle_df = pd.read_csv(particle_dist_path, comment='#')

# Número de subplots = cantidad de balizas + 1 extra para partículas
num_plots = len(dfs) + 1

fig, axs = plt.subplots(num_plots, 1, figsize=(10, 4 * num_plots), sharex=True)
if num_plots == 1:
    axs = [axs]

# Graficar para cada baliza
for i, (df, beacon_id) in enumerate(zip(dfs, beacon_ids)):
    timestamps = pd.to_numeric(df['timestamp']).to_numpy()
    x = df['x_mean'].to_numpy()
    y = df['y_mean'].to_numpy()
    z = df['z_mean'].to_numpy()

    axs[i].plot(timestamps, x, label='X estimado', color='C0')
    axs[i].plot(timestamps, y, label='Y estimado', color='C1')
    axs[i].plot(timestamps, z, label='Z estimado', color='C2')

    # Referencias reales de la baliza
    ref = beacon_refs.loc[beacon_id]
    axs[i].hlines(ref['x'], timestamps[0], timestamps[-1], colors='C0', linestyles='dashed', label='X referencia')
    axs[i].hlines(ref['y'], timestamps[0], timestamps[-1], colors='C1', linestyles='dashed', label='Y referencia')
    axs[i].hlines(ref['z'], timestamps[0], timestamps[-1], colors='C2', linestyles='dashed', label='Z referencia')

    # Línea vertical EKF start para esta baliza
    ekf_start = ekf_start_dict.get(beacon_id, None)
    if ekf_start is not None and timestamps[0] <= ekf_start <= timestamps[-1]:
        axs[i].axvline(x=ekf_start, color='purple', linestyle='-.', label='EKF start')

    axs[i].set_ylabel('Coordenada estimada (m)')
    axs[i].set_title(f'Convergencia Filtro Partículas - Beacon {beacon_id}')
    axs[i].legend(loc='upper right')
    axs[i].grid(True)

# Graficar distribución de partículas en el subplot extra
timestamps = pd.to_numeric(particle_df['timestamp']).to_numpy()
for col in particle_df.columns:
    if col != 'timestamp':
        axs[-1].plot(timestamps, particle_df[col].to_numpy(), label=col)

axs[-1].set_title('Distribución de partículas asignadas por baliza')
axs[-1].set_ylabel('Cantidad de partículas')
axs[-1].set_xlabel('Tiempo (s)')
axs[-1].legend(loc='upper right')
axs[-1].grid(True)

plt.tight_layout()
plt.show()
