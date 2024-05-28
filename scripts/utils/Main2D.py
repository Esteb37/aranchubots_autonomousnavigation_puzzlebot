import numpy as np
from KF2D import KF2D
import matplotlib.pyplot as plt

# Variables del sistema S = [x; v] z = x + r
real_x = 0
real_y = 0

meas_x = 0
meas_y = 0

prev_meas_x = 0
prev_meas_y = 0

meas_variance = 0.05
accel_variance = 0.05

# Parámetros de simulacion
DT = 0.1
steps = 1000
meas_step = 5

# Vectores de resultados
x_real = []
v_real = []
x_meas = []
v_est = []

mean_kf = []
cov_kf = []

# Filtro de Kalman
kf = KF2D(0, 0.5, accel_variance)

# Simulación
for step in range(steps):
	real_x = np.sin(1 * DT * step)
	real_v = np.cos(1 * DT * step)

	# Preddicción
	kf.predict(DT)

	if (step != 0) and (step % meas_step == 0):
		meas_x = real_x + np.random.randn() * meas_variance
		est_v = (meas_x - prev_meas) / (meas_step * DT)

		# Corrección
		kf.correc(meas_x, meas_variance)
		prev_meas = meas_x

	# Guardar resultados
	x_real.append(real_x)
	v_real.append(real_v)
	x_meas.append(meas_x)
	v_est.append(est_v)
	mean_kf.append(kf.mu)
	cov_kf.append(kf.sigma)

# Grafcar resultados de posición
plt.subplot(2, 1, 1)
plt.plot(x_real, 'b')
plt.plot(x_meas, 'g')
plt.plot([mean[0] for mean in mean_kf], 'r')

# Grafcar resultados de velocidad
plt.subplot(2, 1, 2)
plt.plot(v_real, 'b')
plt.plot(v_est, 'g')
plt.plot([mean[1] for mean in mean_kf], 'r')

plt.show()