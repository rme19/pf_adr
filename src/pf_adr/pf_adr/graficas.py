import pandas as pd
import matplotlib.pyplot as plt

# Cargar los datos
df0 = pd.read_csv('~/ros2_ws/src/pf_adr/pf_logs/pf_means_0.csv')
df1 = pd.read_csv('~/ros2_ws/src/pf_adr/pf_logs/pf_means_1.csv')
df2 = pd.read_csv('~/ros2_ws/src/pf_adr/pf_logs/pf_means_2.csv')

# Crear subplots
fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

for i, (df, beacon_id) in enumerate(zip([df0, df1, df2], [0, 1, 2])):
    timestamps = df['timestamp'].to_numpy()
    x = df['x_mean'].to_numpy()
    y = df['y_mean'].to_numpy()
    z = df['z_mean'].to_numpy()
    axs[i].plot(timestamps, x, label='X')
    axs[i].plot(timestamps, y, label='Y')
    axs[i].plot(timestamps, z, label='Z')
    axs[i].set_ylabel('Coordenada estimada')
    axs[i].set_title(f'Convergencia Filtro Partículas - Beacon {beacon_id}')
    axs[i].legend()
    axs[i].grid(True)

axs[-1].set_xlabel('Tiempo (s)')
plt.tight_layout()
plt.show()