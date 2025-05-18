import pandas as pd
import matplotlib.pyplot as plt
import os
import glob

# Cargar los datos de referencia de las balizas
beacon_ref_path = os.path.expanduser('~/pf_logs/beacon_positions.csv')
beacon_refs = pd.read_csv(beacon_ref_path, index_col='beacon_id')

# Buscar todos los archivos pf_means_*.csv
csv_dir = os.path.expanduser('~/pf_logs')
csv_files = sorted(glob.glob(os.path.join(csv_dir, 'pf_means_*.csv')))

# Cargar los dataframes y extraer beacon_id de cada archivo
dfs = []
beacon_ids = []
for file in csv_files:
    df = pd.read_csv(file)
    dfs.append(df)
    beacon_id = int(os.path.splitext(os.path.basename(file))[0].split('_')[-1])
    beacon_ids.append(beacon_id)

# Cargar CSV con la distribución de partículas para el subplot extra
particle_dist_path = os.path.expanduser('~/pf_logs/pf_particles.csv')
particle_df = pd.read_csv(particle_dist_path)

# Crear subplots dinámicamente, uno más de los dfs que tienes
num_plots = len(dfs) + 1  # +1 subplot extra

fig, axs = plt.subplots(num_plots, 1, figsize=(10, 4 * num_plots), sharex=True)

if num_plots == 1:
    axs = [axs]

# Graficar los filtros de partículas para cada baliza
for i, (df, beacon_id) in enumerate(zip(dfs, beacon_ids)):
    timestamps = df['timestamp'].to_numpy()
    x = df['x_mean'].to_numpy()
    y = df['y_mean'].to_numpy()
    z = df['z_mean'].to_numpy()
    axs[i].plot(timestamps, x, label='X')
    axs[i].plot(timestamps, y, label='Y')
    axs[i].plot(timestamps, z, label='Z')
    # Dibujar la referencia real de la baliza
    ref = beacon_refs.loc[beacon_id]
    axs[i].hlines(ref['x'], timestamps[0], timestamps[-1], colors='C0', linestyles='dashed', label='X ref')
    axs[i].hlines(ref['y'], timestamps[0], timestamps[-1], colors='C1', linestyles='dashed', label='Y ref')
    axs[i].hlines(ref['z'], timestamps[0], timestamps[-1], colors='C2', linestyles='dashed', label='Z ref')
    axs[i].set_ylabel('Coordenada estimada')
    axs[i].set_title(f'Convergencia Filtro Partículas - Beacon {beacon_id}')
    axs[i].legend()
    axs[i].grid(True)

# Graficar distribución de partículas en el subplot extra
timestamps = particle_df['timestamp'].to_numpy()
for col in particle_df.columns:
    if col != 'timestamp':
        axs[-1].plot(timestamps, particle_df[col].to_numpy(), label=col)

axs[-1].set_title('Distribución de partículas asignadas por baliza')
axs[-1].set_ylabel('Cantidad de partículas')
axs[-1].set_xlabel('Tiempo (s)')
axs[-1].legend()
axs[-1].grid(True)

plt.tight_layout()
plt.show()
