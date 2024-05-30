from KF import KalmanFilter
import numpy as np

Qk = np.array([[0.5, 0.01, 0.01],
               [0.01, 0.5, 0.01],
               [0.01, 0.01, 0.2]])

Rk = np.array([[0.1, 0.0],
			   [0.0, 0.02]])

z = np.array([[4.87, 0.8],
     [4.72, 0.72],
     [4.69, 0.65]])

mu = [0, 0, 0]

m = [3, 4]

dt = 0.1

u = [1, 1]

KF = KalmanFilter(dt, mu)

for i in range(len(z)):
	KF.predict(u, Qk)
	mu, sigma = KF.correct(m, z[i], Rk)
	print(mu)
	print(sigma)
	print("-------------------")