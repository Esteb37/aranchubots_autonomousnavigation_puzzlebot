#!/usr/bin/env python
import numpy as np

# u 		= control inputs		[v, w]
# q 		= motion noise
# s 		= motion model 			mu_prev + q + [[dt*cos(theta_prev)*v]
# 												  [dt*sin(theta_prev)*v]
# 												  [dt*w]]
# m_i 		= landmark position		[m_i_x, m_i_y]
# z_i 		= landmark measurement 	[z_i_rho, z_i_alpha]
# z_i_rho	= landmark distance		sqrt((m_i_x - s_x)^2 + (m_i_y - s_y)^2) + r_i_rho
# z_i_alpha	= landmark bearking		atan2(m_i_y - s_y, m_i_x - s_x) - s_theta + r_i_alpha
# r_i 		= measurement noise
# H 		= movement jacobian		[[1	0	-dt*V*sin(theta_prev)],
# 								  	 [0	1	dt*V*cos(theta_prev)],
# 								  	 [0	0	1]]
# dx		= m_i_x - s_x
# dy		= m_i_y - s_y
# p 		= dx^2 + dy^2
# G 		= observation jacobian	[[-dx/sqrt(p),	-dy/sqrt(p), 0],
# 		  	   						[dy/p, 			-dx/p, 		-1]]



class KalmanFilter():

	def __init__(self, dt, mu):
		self.dt = dt
		self.mu = mu
		self.sigma = np.zeros((3,3))
		self.I = np.identity(3)

	def h(self, prev_mu, u):
		v, w = u
		x, y, theta = prev_mu
		return np.array([x + self.dt*np.cos(theta)*v, y + self.dt*np.sin(theta)*v, theta + self.dt*w])

	def H(self, s, u):
		v = u[0]
		theta = s[2]
		return np.array([[1, 0, -self.dt*v*np.sin(theta)], [0, 1, self.dt*v*np.cos(theta)], [0, 0, 1]])

	def g(self, s, m_i):
		m_i_x, m_i_y = m_i
		s_x, s_y, s_theta = s
		dx = m_i_x - s_x
		dy = m_i_y - s_y
		p = dx*dx + dy*dy
		z_i_rho = np.sqrt(p)
		z_i_alpha = np.arctan2(dy, dx) - s_theta
		z_i_alpha = np.arctan2(np.sin(z_i_alpha), np.cos(z_i_alpha))
		return np.array([z_i_rho, z_i_alpha])

	def G(self, s, m_i):
		m_i_x, m_i_y = m_i
		s_x, s_y, _ = s
		dx = m_i_x - s_x
		dy = m_i_y - s_y
		p = dx*dx + dy*dy
		return np.array([[-dx/np.sqrt(p), -dy/np.sqrt(p), 0], [dy/p, -dx/p, -1]])

	def q(self, Q):
		return np.random.normal(0, Q)

	def r(self, R):
		return np.random.normal(0, R)

	def predict(self,u, Q):
		self.mu_pred = self.h(self.mu, u)
		H = self.H(self.mu, u)
		self.sigma_pred = H.dot(self.sigma).dot(H.T) + Q

	def step(self):
		self.mu = self.mu_pred
		self.sigma = self.sigma_pred
		return self.mu, self.sigma

	def correct(self, m, z, R):
		z_pred = self.g(self.mu_pred, m)
		G = self.G(self.mu_pred, m)
		Z = G.dot(self.sigma_pred).dot(G.T) + R
		K = self.sigma_pred.dot(G.T).dot(np.linalg.inv(Z))
		self.mu = self.mu_pred + K.dot(z - z_pred)
		self.sigma = (self.I - K.dot(G)).dot(self.sigma_pred)
		return self.mu, self.sigma