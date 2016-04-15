#!/usr/bin/env python

import rospy
#for getkey
import sys, select, termios, tty
from geometry_msgs.msg import Twist

TOPIC = 'cmd_vel'
LINEAR_SPEED = 1.0
ANGULAR_SPEED = 1.0

##From ROS's original teleop_twist_keyboard
def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def teleoperator():
	rospy.init_node('robot_teleop')
	print ('Use wasd to move, qezc for diagonals, jkl to set angular speed. Press p to exit, any other key to brake')
	publisher = rospy.Publisher(TOPIC, Twist, queue_size=1)
	msg = Twist()
	msg.linear.x = 0.0
	msg.linear.y = 0.0
	msg.angular.z = 0.0
	while(1):
		msg.linear.x = 0.0
		msg.linear.y = 0.0
		key = getKey()
		if key == 'p':
			termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
			msg.angular.z = 0.0
			publisher.publish(msg)
			break
		if key in {'q', 'w', 'e'}:
			msg.linear.x = LINEAR_SPEED
		elif key in {'z', 's', 'c'}:
			msg.linear.x = -(LINEAR_SPEED)
		else:
			msg.linear.x = 0.0

		if key in {'q', 'a', 'z'}:
			msg.linear.y = LINEAR_SPEED
		elif key in {'e', 'd', 'c'}:
			msg.linear.y = -(LINEAR_SPEED)
		else:
			msg.linear.y = 0.0

		if key == 'j':
			msg.angular.z = ANGULAR_SPEED
			print('Angular speed set to ' + str(ANGULAR_SPEED))
		elif key == 'k':
			msg.angular.z = 0.0
			print('Angular speed set to 0')
		elif key ==  'l':
			msg.angular.z = -(ANGULAR_SPEED)
			print('Angular speed set to -' + str(ANGULAR_SPEED))
		publisher.publish(msg)



if __name__ == '__main__':
    try:
    	settings = termios.tcgetattr(sys.stdin)
        teleoperator()
    except rospy.ROSInterruptException:
    	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)