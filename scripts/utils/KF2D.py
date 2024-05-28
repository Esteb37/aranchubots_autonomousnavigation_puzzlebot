import numpy as np

class KF2D:

	Q = np.array([[0.1, 0, 0, 0], [0, 0.05, 0, 0],[0, 0, 0.1, 0], [0, 0, 0, 0.05]])
	C = np.array([[1, 0]])

	# Funcion de inicializacion del fltro de kalman
	def __init__(self, init_x, init_v, accel_variance):
		self.mu = np.zeros(2)
		self.mu[0] = init_x
		self.mu[1] = init_v
		self.sigma = np.eye(2)
		self.variance = accel_variance

	# Funcion de prediccion
	def predict(self, dt):
		A = np.array([[1, dt], [0, 1]])
		B = np.array([0, dt])

		self.mu = A.dot(self.mu) + B * self.variance


		self.sigma = A.dot(self.sigma).dot(A.T) + self.Q


	# Funcion de correccion
	def correc(self, meas_x, meas_variance):
		K = self.sigma.dot(self.C.T).dot(np.linalg.inv(self.C.dot(self.sigma).dot(self.C.T) + meas_variance))
		self.mu = self.mu + K.dot(meas_x - self.C.dot(self.mu))
		self.sigma = self.sigma - K.dot(self.C).dot(self.sigma)

	@property
	def mean(self):
		return self.mu

	@property
	def cov(self):
		return self.sigma