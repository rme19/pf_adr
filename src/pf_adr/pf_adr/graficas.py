import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('~/ros2_ws/src/pf_adr/pf_logs/pf_means_20250516_194221.csv')  # Ajusta al nombre correcto
# Graficar
# Convertir explícitamente las columnas a arrays unidimensionales
timestamps = df['timestamp'].to_numpy()
x = df['x_mean'].to_numpy()
y = df['y_mean'].to_numpy()
z = df['z_mean'].to_numpy()

# Graficar
plt.plot(timestamps, x, label='X')
plt.plot(timestamps, y, label='Y')
plt.plot(timestamps, z, label='Z')
plt.xlabel('Tiempo (s)')
plt.ylabel('Coordenada estimada')
plt.title('Convergencia del filtro de partículas')
plt.legend()
plt.grid(True)
plt.show()
