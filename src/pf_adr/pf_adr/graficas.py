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
    # Extraer el id del nombre del archivo
    beacon_id = int(os.path.splitext(os.path.basename(file))[0].split('_')[-1])
    beacon_ids.append(beacon_id)

# Crear subplots dinámicamente
fig, axs = plt.subplots(len(dfs), 1, figsize=(10, 4 * len(dfs)), sharex=True)

if len(dfs) == 1:
    axs = [axs]  # Para el caso de solo un subplot

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

axs[-1].set_xlabel('Tiempo (s)')
plt.tight_layout()
plt.show()