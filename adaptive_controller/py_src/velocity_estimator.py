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
from geometry_msgs.msg import PoseWithCovarianceStamped as Pose
from geometry_msgs.msg import TwistStamped

roslib.load_manifest('adaptive_controller')

class VelocityEstimator(object):
	def __init__(self, update_period=0.3, node_name='velocity_estimator', pose_topic='/poseupdate', velocity_topic='/est_vel_stamped'):
		self.node_name = node_name
		self.update_period = update_period
		self.pose_topic = pose_topic
		self.velocity_topic = velocity_topic
		self.pose = None
		self.velocity_publisher = None

	def get_velocity(self, prev_pose, cur_pose):
		velocity = TwistStamped()
		velocity.header.stamp = prev_pose.header.stamp
		velocity.header.frame_id = prev_pose.header.frame_id
		delta = (prev_pose.header.stamp - cur_pose.header.stamp).to_sec()
		velocity.twist.linear.x = (cur_pose.pose.pose.position.x - prev_pose.pose.pose.position.x) / delta
		velocity.twist.linear.y = (cur_pose.pose.pose.position.y - prev_pose.pose.pose.position.y) / delta
		velocity.twist.linear.z = (cur_pose.pose.pose.position.z - prev_pose.pose.pose.position.z) / delta
		#get angular velocity
		prev_pose_quat = numpy.array([prev_pose.pose.pose.orientation.x,
	 	prev_pose.pose.pose.orientation.y,
	 	prev_pose.pose.pose.orientation.z,
	 	prev_pose.pose.pose.orientation.w],dtype=numpy.float64)
		cur_pose_quat = numpy.array([cur_pose.pose.pose.orientation.x,
	 	cur_pose.pose.pose.orientation.y,
	 	cur_pose.pose.pose.orientation.z,
	 	cur_pose.pose.pose.orientation.w], dtype=numpy.float64)
		resudal_quat = cur_pose_quat - prev_pose_quat
		der_quat = numpy.float64(2.0) * resudal_quat / numpy.float64(delta)
		vel_quat = transformations.quaternion_multiply(transformations.quaternion_inverse(prev_pose_quat), der_quat)
		velocity.twist.angular.x, velocity.twist.angular.y, velocity.twist.angular.z = vel_quat.tolist()[0:3]
		return velocity

	def estimate(self):
		rospy.init_node(self.node_name, log_level=rospy.INFO)
		if rospy.has_param('~pose'):
			self.pose_topic = rospy.get_param('~pose')
		if rospy.has_param('~est_vel_stamped'):
			self.velocity_topic = rospy.get_param('~est_vel_stamped')
		vel_sub = rospy.Subscriber(self.pose_topic, Pose, self.update)
		self.velocity_publisher = rospy.Publisher(self.velocity_topic, TwistStamped, queue_size=10)
		rospy.spin()

	def update(self, pose):
		if not self.pose is None:
			velocity = self.get_velocity(self.pose, pose)
			self.velocity_publisher.publish(velocity)
		self.pose = pose


if __name__ == '__main__':
	estimator = VelocityEstimator();
	try:
		estimator.estimate()
	except rospy.ROSInterruptException:
		pass