#!/usr/bin/env python
import rospy
import roslib
import std_msgs.msg
import numpy
import tf
import message_filters

from tf import transformations

# Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

roslib.load_manifest('adaptive_controller')

class VelocityFromOdometryCollector(object):
	def __init__(self, update_period=0.3, node_name='velocity_from_odom_collector', odom_topic='/odom', velocity_topic='/est_vel_stamped'):
		self.node_name = node_name
		self.update_period = update_period
		self.odom_topic = odom_topic
		self.velocity_topic = velocity_topic
		self.pose = None
		self.velocity_publisher = None

	def get_velocity(self, pose):
		velocity = TwistStamped()
		velocity.header.stamp = pose.header.stamp
		velocity.header.frame_id = pose.header.frame_id
		velocity.twist = pose.twist.twist
		return velocity

	def collect(self):
		rospy.init_node(self.node_name, log_level=rospy.INFO)
		if rospy.has_param('~odom'):
			self.odom_topic = rospy.get_param('~odom')
		if rospy.has_param('~vel'):
			self.velocity_topic = rospy.get_param('~vel')
		odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.update)
		self.velocity_publisher = rospy.Publisher(self.velocity_topic, TwistStamped, queue_size=10)
		rospy.spin()

	def update(self, pose):
		velocity = self.get_velocity(pose)
		self.velocity_publisher.publish(velocity)


if __name__ == '__main__':
	collector = VelocityFromOdometryCollector();
	try:
		collector.collect()
	except rospy.ROSInterruptException:
		pass