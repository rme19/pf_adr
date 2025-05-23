import pandas as pd
import matplotlib.pyplot as plt
import os
import glob

# ------------------------------
# RUTAS Y CARGA DE ARCHIVOS
# ------------------------------
log_dir = os.path.expanduser('~/pf_logs')
beacon_ref_path = os.path.join(log_dir, 'beacon_positions.csv')
csv_files = sorted(glob.glob(os.path.join(log_dir, 'pf_means_*.csv')))

# Datos de referencia
beacon_refs = pd.read_csv(beacon_ref_path, index_col='beacon_id')



# Cargar estimaciones de cada baliza
dfs = []
beacon_ids = []
for file in csv_files:
    df = pd.read_csv(file)
    beacon_id = int(os.path.splitext(os.path.basename(file))[0].split('_')[-1])
    dfs.append(df)
    beacon_ids.append(beacon_id)

# ------------------------------
# FIGURAS INDIVIDUALES POR BALIZA
# ------------------------------
for df, beacon_id in zip(dfs, beacon_ids):
    fig, axs = plt.subplots(4, 1, figsize=(10, 12))  # Quita sharex=True
    timestamps = df['timestamp'].to_numpy()
    x = df['x_mean'].to_numpy()
    y = df['y_mean'].to_numpy()
    z = df['z_mean'].to_numpy()

    ref = beacon_refs.loc[beacon_id]
    x_ref, y_ref, z_ref = ref['x'], ref['y'], ref['z']

    # Posición estimada vs referencia (todo el tiempo)
    axs[0].plot(timestamps, x, label='X', color='r')
    axs[0].plot(timestamps, y, label='Y', color='g')
    axs[0].plot(timestamps, z, label='Z', color='b')
    axs[0].hlines(x_ref, timestamps[0], timestamps[-1], colors='r', linestyles='dashed', label='X ref')
    axs[0].hlines(y_ref, timestamps[0], timestamps[-1], colors='g', linestyles='dashed', label='Y ref')
    axs[0].hlines(z_ref, timestamps[0], timestamps[-1], colors='b', linestyles='dashed', label='Z ref')
    axs[0].set_ylabel('Posición')
    axs[0].set_title(f'Beacon {beacon_id} - Posición estimada vs referencia')
    axs[0].set_xlabel('Tiempo (s)')
    axs[0].legend()
    axs[0].grid(True)

    # --- Filtrar últimos 10 segundos ---
    last_time = timestamps[-1]
    mask = timestamps >= (last_time - 10)
    t10 = timestamps[mask]
    x10 = x[mask]
    y10 = y[mask]
    z10 = z[mask]

    # Error en X (últimos 10s)
    axs[1].set_title(f'Beacon {beacon_id} - Error en X')
    axs[1].plot(t10, x_ref - x10, label='Error X', color='r')
    axs[1].set_ylabel('Error X')
    axs[1].set_xlabel('Tiempo (s)')
    axs[1].grid(True)

    # Error en Y (últimos 10s)
    axs[2].set_title(f'Beacon {beacon_id} - Error en Y')
    axs[2].plot(t10, y_ref - y10, label='Error Y', color='g')
    axs[2].set_ylabel('Error Y')
    axs[2].set_xlabel('Tiempo (s)')
    
    axs[2].grid(True)

    # Error en Z (últimos 10s)
    axs[3].set_title(f'Beacon {beacon_id} - Error en Z')
    axs[3].plot(t10, z_ref - z10, label='Error Z', color='b')
    axs[3].set_ylabel('Error Z')
    axs[3].set_xlabel('Tiempo (s)')
    axs[3].grid(True)

    plt.tight_layout()
    plt.show()

# ------------------------------
# FIGURA GENERAL: TODAS LAS BALIZAS
# ------------------------------
# Crear subplots dinámicamente, uno más de los dfs que tienes
num_plots = len(dfs)  # +1 subplot extra

fig, axs = plt.subplots(num_plots, 1, figsize=(10, 4 * num_plots), sharex=True)

if num_plots == 1:
    axs = [axs]

# Graficar los filtros de partículas para cada baliza
for i, (df, beacon_id) in enumerate(zip(dfs, beacon_ids)):
    timestamps = df['timestamp'].to_numpy()
    x = df['x_mean'].to_numpy()
    y = df['y_mean'].to_numpy()
    z = df['z_mean'].to_numpy()


    axs[i].plot(timestamps, x, label='X',color='r') 
    axs[i].plot(timestamps, y, label='Y',color='g')
    axs[i].plot(timestamps, z, label='Z',color='b')
    # Dibujar la referencia real de la baliza
    ref = beacon_refs.loc[beacon_id]


    axs[i].hlines(ref['x'], timestamps[0], timestamps[-1], colors='r', linestyles='dashed', label='X ref')
    axs[i].hlines(ref['y'], timestamps[0], timestamps[-1], colors='g', linestyles='dashed', label='Y ref')
    axs[i].hlines(ref['z'], timestamps[0], timestamps[-1], colors='b', linestyles='dashed', label='Z ref')
    axs[i].set_ylabel('Coordenada estimada')
    axs[i].set_title(f'Beacon {beacon_id} - Posición estimada vs referencia')
    axs[i].legend()
    axs[i].grid(True)

    
plt.tight_layout()
plt.show()
