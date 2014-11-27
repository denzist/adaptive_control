#!/usr/bin/env python
import rospy
import roslib
import std_msgs.msg
import tf
import message_filters

from tf import transformations

import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt

# Messages
from adaptive_controller.msg import Data
from adaptive_controller.msg import ControlMatrix
from control_msgs.msg import ControlVector

#threading
from multiprocessing import Process, Event
import time


roslib.load_manifest('adaptive_controller')

class DataSet(object):
	"""docstring for DataSet"""
	def __init__(self, size=60):
		self.size = size
		self.out = []
		self.obs = []
		self.__new = 0

	def is_fully_updated(self):
		if self.get_curr_size() == 0:
			return False
		if self.__new == 0:
			return True
		else:
			return False

	#data - np.array(shape(1,5))
	def add_out(self, data):
		if len(self.out) + len(self.obs) == self.size:
			if len(self.obs) != 0:
				self.obs = self.obs[1:]
			else:
				self.out = self.out[1:]
		self.out.append(data)
		self.__new = (self.__new + 1) % self.size

	def add_obs(self, data):
		if len(self.out) + len(self.obs) == self.size:
			if len(self.out) != 0:
				self.out = self.out[1:]
			else:
				self.obs = self.obs[1:]
		self.obs.append(data)
		self.__new = (self.__new + 1) % self.size

	def get_curr_size(self):
		return len(self.out) + len(self.obs)

	def get_f(self):
		return np.array([len(self.obs), len(self.out)])

	#swap and delete obs
	def unite(self):
		self.obs = self.obs + self.out
		self.out = []



class Learning(object):
	def __init__(self, data_topic='/data', model_topic='/model', alpha=0.05, size=100):
		self.data_topic = data_topic
		self.model_topic = model_topic
		#init model 0 - lin, 1 - ang
		self.data_set = [DataSet(size), DataSet(size)]
		self.size = size
		#self.model = np.transpose(np.array([[1., 1., 1., 1.], [-1., -1., 1., 1.]])/4.)
		self.model = np.transpose(np.array([[1./ 40., 1./ 40., 1./ 40., 1./ 40.] , [-1./(4.*3.16), -1./(4.*3.16), 1./(4.*3.16), 1./(4.*3.16)]]))
		rospy.loginfo(self.model)
		#initial distribution
		sigma = 0.01
		self.s = [sigma, sigma]
		self.p = stats.norm(0.0, 1.0).cdf(3.0) - stats.norm(0.0, 1.0).cdf(-3.0)
		self.f = np.array([self.p, 1 - self.p]) * size 
		#statistical significance
		self.a = alpha

	#data = [est.v.x(z), u]
	#submodel = model[i]
	def get_error(self, data, submodel):
		return data[0] - np.dot(np.array(data[1:]), submodel)

	#e - scalar error
	def is_out(self, e, s):
		if e < 3.*s and e > -3.*s:
			return False
		else:
			return True

	def get_data_type_id(self, data_type):
		if data_type == 'ang':
			return 1
		if data_type == 'lin':
			return 0

	def add(self, data_type, data):
		i = self.get_data_type_id(data_type)
		submodel = self.model[:, i]
		s = self.s[i]
		is_out = self.is_out(self.get_error(data, submodel),s)
		if is_out == True:
			self.data_set[i].add_out(data)
		else:
			self.data_set[i].add_obs(data)

	def get_size(self, data_type):
		i = self.get_data_type_id(data_type)
		return self.data_set[i].get_curr_size()

	def is_bad(self, data_type):
		i = self.get_data_type_id(data_type)
		rospy.loginfo(data_type)
		rospy.loginfo(self.data_set[i].get_f())
		p_value = stats.chisquare(self.data_set[i].get_f(), f_exp=self.f)[1]
		if(p_value < self.a):
			return True
		else:
			return False

	def is_fully_updated(self, data_type):
		i = self.get_data_type_id(data_type)
		return self.data_set[i].is_fully_updated()

	def update_data(self, data_type):
		i = self.get_data_type_id(data_type)
		self.data_set[i].unite()

	#TODO
	def update_model(self, data_type):
		i = self.get_data_type_id(data_type)
		a = np.array(self.data_set[i].obs)[:,1:]
		b = np.array(self.data_set[i].obs)[:,0]
		submodel = np.linalg.lstsq(a, b)
		residuals = (b - np.dot(a, submodel[0]))**2
		sum_residuals = np.sum(residuals)
		R_squared = 1 - sum_residuals / np.sum((b - b.mean())**2)
		#self.model[:, i] = submodel[0]
		#self.s[i] = np.sqrt(sum_residuals / self.get_size(data_type))
		return [submodel[0], sum_residuals, R_squared]

	def train(self):
		self.publisher = rospy.Publisher(self.model_topic, ControlMatrix, queue_size=10)
		self.subscriber = rospy.Subscriber(self.data_topic, Data, self.update)
		rospy.spin()

	def update(self, data):
		update_flag = False
		#if v.lin.x != 0
		if data.cmd_vel.twist.linear.x != 0.0:
			v = np.sqrt(data.est_vel.twist.linear.x**2 + data.est_vel.twist.linear.y**2)
			if data.cmd_vel.twist.linear.x >= 0:
				v = v
			else:
				v= -v
			d = [v] + list(data.control.control)
			self.add('lin', d)

			if self.is_fully_updated('lin'):
				if(self.is_bad('lin') == True):
					update_flag = True
					self.update_data('lin')
					submodel = self.update_model('lin')
					rospy.loginfo('lin')
					rospy.loginfo(submodel)
		#if v.ang.z = 0
		if data.cmd_vel.twist.angular.z != 0.0:
			d = [-data.est_vel.twist.angular.z] + list(data.control.control)
			self.add('ang', d)

			if self.is_fully_updated('ang'):
				if(self.is_bad('ang') == True):
					update_flag = True
					self.update_data('ang')
					submodel = self.update_model('ang')
					rospy.loginfo('ang')
					rospy.loginfo(submodel)
		if update_flag == True:
			rospy.loginfo(self.model)
			msg = ControlMatrix()
			msg.header.stamp = rospy.get_rostime()
			msg.row = self.model.shape[1]
			msg.col = self.model.shape[0]
			msg.control = self.model.reshape(msg.row*msg.col).tolist()
			self.publisher.publish(msg)


if __name__ == '__main__':
	rospy.init_node('dasd')
	data_topic = '/data'
	if rospy.has_param('~data'):
		data_topic = rospy.get_param('~data')
	model_topic = '/model'
	if rospy.has_param('~model'):
		model_topic = rospy.get_param('~model')
	size = 30
	if rospy.has_param('~size'):
		size = rospy.get_param('~size')
	a = 0.05
	if rospy.has_param('~alpha'):
		a = rospy.get_param('~alpha')
	learner = Learning(data_topic, model_topic, a, size);
	try:
		learner.train()
	except rospy.ROSInterruptException:
		pass