""" Written by Brian Hou for CSE571: Probabilistic Robotics (Winter 2019)
"""

import numpy as np

from utils import minimized_angle

class ExtendedKalmanFilter:
    def __init__(self, mean, cov, alphas, beta):
        self.alphas = alphas
        self.beta = beta

        self._init_mean = mean
        self._init_cov = cov
        self.reset()

    def reset(self):
        self.mu = self._init_mean
        self.sigma = self._init_cov

    def update(self, env, u, z, marker_id): 
        """Update the state estimate after taking an action and receiving a landmark
        observation.

        u: action
        z: landmark observation
        marker_id: landmark ID
        """
        # YOUR IMPLEMENTATION HERE
        # Prediction step
        mu_t_bar = env.forward(self.mu, u)
        Gt = env.G(self.mu, u)
        Vt = env.V(self.mu, u)
        Mt = env.noise_from_motion(u, self.alphas)
        sigma_t_bar = Gt.dot(self.sigma).dot(Gt.T) + Vt.dot(Mt).dot(Vt.T)

        # Correction step
        Ht = env.H(mu_t_bar, marker_id)
        Qt = self.beta
        St = Ht.dot(sigma_t_bar).dot(Ht.T) + Qt
        Kt = sigma_t_bar.dot(Ht.T).dot(np.linalg.inv(St))

        self.mu = mu_t_bar + Kt.dot(minimized_angle(z - env.observe(mu_t_bar, marker_id)))
        # self.mu[2] = minimized_angle(self.mu[2])
        self.sigma = (np.eye(3) - Kt.dot(Ht)).dot(sigma_t_bar)

        return self.mu, self.sigma
