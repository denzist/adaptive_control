#!/usr/bin/env python
import rospy
import roslib
import tf
import numpy as np

# Messages
from geometry_msgs.msg import Twist
from adaptive_controller.msg import ControlMatrix
from control_msgs.msg import ControlVector

roslib.load_manifest('adaptive_controller')

class Controller(object):
  def __init__(self, node_name='robot_controller', cmd_topic='/gazebo/cmd_vel', model_topic='/model', control_topic='/gazebo/control_vector'):
    self.model_topic = model_topic
    self.cmd_topic = cmd_topic
    self.control_topic = control_topic
    self.node_name = node_name
    #self.model = np.array([[1./ 40., 1./ 40., 1./ 40., 1./ 40.] ,  [-1./(4.*3.16), -1./(4.*3.16), 1./(4.*3.16), 1./(4.*3.16)]])
    self.model = np.array([[1./ 4., 1./ 4., 1./ 4., 1./ 4.] ,  [-1./(4.), -1./(4.), 1./(4.), 1./(4.)]])

  def control(self):
    rospy.init_node(self.node_name)
    if rospy.has_param('~cmd_vel'):
        self.cmd_topic = rospy.get_param('~cmd_vel')
    if rospy.has_param('~control'):
        self.control_topic = rospy.get_param('~control')
    if rospy.has_param('~model'):
        self.model_topic = rospy.get_param('~model')

    self.pub = rospy.Publisher(self.control_topic, ControlVector, queue_size=10)
    self.vel_sub = rospy.Subscriber(self.cmd_topic, Twist, self.update)
    self.model_sub = rospy.Subscriber(self.model_topic, ControlMatrix, self.update_model)
    while not rospy.is_shutdown():
      rospy.spin()

  def update_model(self, model):
    self.model = np.array(model.control).reshape((model.col, model.row))
    rospy.loginfo(self.model)

  def update(self, v):
    vel =  self.vel_to_list(v)
    b = [vel[0], vel[5]]
    a = self.model
    u = np.linalg.lstsq(a, b)[0].tolist()
    msg = ControlVector()
    msg.header.stamp = rospy.get_rostime()
    msg.length = len(u)
    msg.control = u
    self.pub.publish(msg)

  def vel_to_list(self, vel):
    return [vel.linear.x, vel.linear.y, vel.linear.z,
    vel.angular.x, vel.angular.y, vel.angular.z]

if __name__ == '__main__':
	controller = Controller();
	try:
		controller.control()
	except rospy.ROSInterruptException:
		pass
