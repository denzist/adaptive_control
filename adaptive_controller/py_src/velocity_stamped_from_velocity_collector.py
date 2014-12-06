#!/usr/bin/env python
import rospy
import roslib
import std_msgs.msg
import numpy
import tf
import message_filters

from tf import transformations

# Messages
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

roslib.load_manifest('adaptive_controller')

class VelocityStampedFromVelocityCollector(object):
	def __init__(self, update_period=0.3, node_name='velocity_stamped_from_velocity_collector', vel_topic='/vel', vel_stamped_topic='/vel_stamped'):
		self.node_name = node_name
		self.update_period = update_period
		self.vel_topic = vel_topic
		self.vel_stamped_topic = vel_stamped_topic
		self.vel_stamped = None

	def get_velocity(self, vel):
		self.vel_stamped = TwistStamped()
		self.vel_stamped.header.stamp = rospy.get_rostime()
		self.vel_stamped.twist = vel


	def collect(self):
		rospy.init_node(self.node_name, log_level=rospy.INFO)
		if rospy.has_param('~vel_stamped'):
			self.vel_stamped_topic = rospy.get_param('~vel_stamped')
		if rospy.has_param('~vel'):
			self.vel_topic = rospy.get_param('~vel')
		rate = 30.0
		if rospy.has_param('~rate'):
			rate = rospy.get_param('~rate')
		vel_sub = rospy.Subscriber(self.vel_topic, Twist, self.get_velocity)
		velocity_publisher = rospy.Publisher(self.vel_stamped_topic, TwistStamped, queue_size=10)
		r = rospy.Rate(rate)
		while not rospy.is_shutdown():
			if(self.vel_stamped != None):
				velocity_publisher.publish(self.vel_stamped)
				r.sleep()

if __name__ == '__main__':
	collector = VelocityStampedFromVelocityCollector();
	try:
		collector.collect()
	except rospy.ROSInterruptException:
		pass