#!/usr/bin/env python
import rospy
import roslib
import std_msgs.msg
import numpy as np
import tf
import message_filters

from tf import transformations

# Messages
from adaptive_controller.msg import DataSet
from adaptive_controller.msg import ControlMatrix

#threading
from multiprocessing import Process, Event
import time


roslib.load_manifest('adaptive_controller')

class Learning(object):
	def __init__(self, node_name='enhancing_matrix_estimator',
	 data_set_topic='/training_set', enhancing_matrix_topic='/fsdfenhancing_matrix'):
		self.node_name = node_name
		self.data_set_topic = data_set_topic
		self.enhancing_matrix_topic = enhancing_matrix_topic
		self.data_set = None
		self.control_matrix = None
		self.lin_control_submatrix = np.identity(3)
		self.ang_control_submatrix = np.identity(3)
		self.ang_period = 0
		self.lin_period = 0
		self.error = 0.1

	def train(self):
		rospy.init_node(self.node_name)
		if rospy.has_param('~training_set'):
			self.data_set_topic = rospy.get_param('~training_set')
		if rospy.has_param('~enhancing_matrix'):
			self.enhancing_matrix_topic = rospy.get_param('~enhancing_matrix')
		if rospy.has_param('~error'):
			self.error = rospy.get_param('~error')
		self.publisher = rospy.Publisher(self.enhancing_matrix_topic, ControlMatrix, queue_size=10)
		self.subscriber = rospy.Subscriber(self.data_set_topic, DataSet, self.update)
		rospy.spin()

	def update(self, data_set):
		is_bad_data = 0

		if data_set.ang_col != 0:
			self.ang_period += 1
			ang_control_submatrix = np.identity(3)
			if self.ang_period%30 == 0:
				cmd_ang = np.array(data_set.cmd_ang).reshape(data_set.ang_col, data_set.ang_row)
				est_ang = np.array(data_set.est_ang).reshape(data_set.ang_col, data_set.ang_row)
				if self.is_bad(cmd_ang, est_ang):
					ang_control_submatrix = self.esimate(est_ang, cmd_ang)
					self.ang_control_submatrix = np.dot(self.ang_control_submatrix, ang_control_submatrix)
					is_bad_data += 1

		if data_set.lin_col != 0:
			self.lin_period += 1
			lin_control_submatrix = np.identity(3)
			if self.lin_period%30 == 0:
				cmd_lin = np.array(data_set.cmd_lin).reshape(data_set.lin_col, data_set.lin_row)
				est_lin = np.array(data_set.est_lin).reshape(data_set.lin_col, data_set.lin_row)
				if self.is_bad(cmd_lin, est_lin):
					lin_control_submatrix = self.esimate(est_lin, cmd_lin)
					self.lin_control_submatrix = np.dot(self.lin_control_submatrix, lin_control_submatrix)
					is_bad_data += 1		

		self.control_matrix = np.array([[self.lin_control_submatrix[0][0], 0.0], [0.0, self.ang_control_submatrix[2][2]]])
		msg = ControlMatrix()
		msg.header.stamp = rospy.get_rostime()
		msg.row = self.control_matrix.shape[1]
		msg.col = self.control_matrix.shape[0]
		msg.control = self.control_matrix.reshape(msg.row*msg.col).tolist()
		if is_bad_data > 0:
			self.publisher.publish(msg)

	def getEstVelArray(self, data_set):
		return  np.array(data_set.est).reshape(data_set.est_col, data_set.est_row)

	def is_bad(self, a, b):
		if np.linalg.norm(a-b) > np.sqrt(a.shape[0])*self.error:
			return True
		return False


	def getDesVelArray(self, data_set):
		return np.array(data_set.des).reshape(data_set.des_col, data_set.des_row) 

	def esimate(self, vel, control): #np.array
		control_matrix = np.linalg.lstsq(vel, control, rcond=0.1)
		return control_matrix[0]

if __name__ == '__main__':
	learner = Learning();
	try:
		learner.train()
	except rospy.ROSInterruptException:
		pass