#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

TOPIC = 'cmd_vel'

def joy_callback(joy_msg):
	global publisher
	global vel_msg
	global LINEAR_SPEED_MAX
	global ANGULAR_SPEED_MAX
	if (joy_msg.buttons[5]):
		vel_msg.linear.x = joy_msg.axes[1] * LINEAR_SPEED_MAX
		vel_msg.linear.y = joy_msg.axes[0] * LINEAR_SPEED_MAX
		vel_msg.angular.z = joy_msg.axes[3] * ANGULAR_SPEED_MAX
	else:
		vel_msg.linear.x = 0.0
		vel_msg.linear.y = 0.0
		vel_msg.angular.z = 0.0
	publisher.publish(vel_msg)


def teleoperator():
	global publisher
	global vel_msg
	global LINEAR_SPEED_MAX
	global ANGULAR_SPEED_MAX
	rospy.init_node('robot_teleop_joy')
	LINEAR_SPEED_MAX = rospy.get_param("linear_max", 1.0)
	ANGULAR_SPEED_MAX = rospy.get_param("angular_max", 1.0)
	print ('Listening to /joy')
	publisher = rospy.Publisher(TOPIC, Twist, queue_size=1)
	subscriber = rospy.Subscriber('joy', Joy, joy_callback, queue_size=1)
	vel_msg = Twist()
	vel_msg.linear.x = 0.0
	vel_msg.linear.y = 0.0
	vel_msg.angular.z = 0.0
	rospy.spin()

if __name__ == '__main__':
    try:
        teleoperator()
    except rospy.ROSInterruptException:
    	pass