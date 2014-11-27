#!/usr/bin/env python
import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt

class DataSet(object):
	"""docstring for DataSet"""
	def __init__(self, size=60):
		self.size = size
		self.out = []
		self.obs = []

	#data - np.array(shape(1,4))
	def add_out(self, data):
		if len(self.out) + len(self.obs) == self.size:
			if len(self.obs) != 0:
				self.obs = self.obs[1:]
			else:
				self.out = self.out[1:]
		self.out.append(data)

	def add_obs(self, data):
		if len(self.out) + len(self.obs) == self.size:
			if len(self.out) != 0:
				self.out = self.out[1:]
			else:
				self.obs = self.obs[1:]
		self.obs.append(data)

if __name__ == '__main__':
	a = np.array([[10., 10., 10., 10.], [-3.16, -3.16, 3.16, 3.16]])
	b = np.array([1., -1.])
	u = np.linalg.lstsq(a, b)
	print u[0].tolist()



