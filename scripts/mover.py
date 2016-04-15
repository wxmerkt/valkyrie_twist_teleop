#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import numpy

LEFT = 0
RIGHT = 1

global steps_pub
global rate

##minimum speed that would be considered a valid movement command (in any direction)
MIN_SPEED = 0.001

from ihmc_msgs.msg import FootstepDataListMessage
from ihmc_msgs.msg import FootstepDataMessage
from ihmc_msgs.msg import FootstepStatusMessage
from geometry_msgs.msg import Twist

##do not increment subscriber queue size, queueing messages is undesirable
def cmd_vel_callback(msg):
    global seq
    global steps_pub
    global SPEED_LIMIT
    global ANGULAR_SPEED_LIMIT
    global MOVING_TIME
    footstepArrayMessage = FootstepDataListMessage()
    footstepArrayMessage.swing_time = MOVING_TIME
    footstepArrayMessage.transfer_time = MOVING_TIME
    footstepArrayMessage.unique_id = seq
    seq += 1
    linX = msg.linear.x
    linY = msg.linear.y
    angZ = msg.angular.z
    ##enforce a (REALLY) low speed limit for now
    if (linX < -SPEED_LIMIT):
        linX = -SPEED_LIMIT
    elif (linX > SPEED_LIMIT):
        linX = SPEED_LIMIT
    if (linY < -SPEED_LIMIT):
        linY = -SPEED_LIMIT
    elif (linY > SPEED_LIMIT):
        linY = SPEED_LIMIT
    ##do the same for angular speed
    if (angZ < -ANGULAR_SPEED_LIMIT):
        angZ = -ANGULAR_SPEED_LIMIT
    elif (angZ > ANGULAR_SPEED_LIMIT):
        angZ = ANGULAR_SPEED_LIMIT
    if (abs(linX) >= MIN_SPEED or abs(linY) >= MIN_SPEED or abs(angZ) >= MIN_SPEED): #filter out messages that would cause no movement in any direction
        if (linY < 0): ##if moving to the right begin with right foot
            footstepArrayMessage.footstep_data_list.append(createFootStepOffset(RIGHT, [linX * (MOVING_TIME * 3), linY * (MOVING_TIME * 3), 0.0], angZ * (MOVING_TIME * 3)))
        footstepArrayMessage.footstep_data_list.append(createFootStepOffset(LEFT, [linX * (MOVING_TIME * 3), linY * (MOVING_TIME * 3), 0.0], angZ * (MOVING_TIME * 3)))
        if (linY >= 0): ## if moving to the left or straight, end with right foot
            footstepArrayMessage.footstep_data_list.append(createFootStepOffset(RIGHT, [linX * (MOVING_TIME * 3), linY * (MOVING_TIME * 3), 0.0], angZ * (MOVING_TIME * 3)))
        steps_pub.publish(footstepArrayMessage)
        waitForFootsteps(len(footstepArrayMessage.footstep_data_list))



def createFootStepInPlace(stepSide):
    global tfBuffer
    footstep = FootstepDataMessage()
    footstep.robot_side = stepSide

    if stepSide == LEFT:
        foot_frame = 'leftFoot'
    else:
        foot_frame = 'rightFoot'

    footWorld = tfBuffer.lookup_transform('world', foot_frame, rospy.Time(), rospy.Duration(1.0))
    footstep.orientation = footWorld.transform.rotation
    footstep.location = footWorld.transform.translation

    return footstep

# Creates footstep offset from the current foot position. The offset is in foot frame. turning is the angle footsteps should
# turn in respect with their previous position (in radians)
def createFootStepOffset(stepSide, offset, turning):
    footstep = createFootStepInPlace(stepSide)

    if (turning < -0.01 or turning > 0.01): # do not turn for very small amounts, may be joystick noise
        (r, p, y) = tf.transformations.euler_from_quaternion([footstep.orientation.x, footstep.orientation.y, footstep.orientation.z, footstep.orientation.w])
        y += turning
        rotated_quat = tf.transformations.quaternion_from_euler(r,p,y)
        footstep.orientation.x = rotated_quat[0]
        footstep.orientation.y = rotated_quat[1]
        footstep.orientation.z = rotated_quat[2]
        footstep.orientation.w = rotated_quat[3]

    # transform the offset to world frame
    quat = footstep.orientation
    rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    transformedOffset = numpy.dot(rot[0:3, 0:3], offset)

    footstep.location.x += transformedOffset[0]
    footstep.location.y += transformedOffset[1]
    footstep.location.z += transformedOffset[2]

    return footstep
    
## have to wait this way because ids do not seem to be implemented yet, should be changed when ids are implemented
def waitForFootsteps(numberOfSteps):
    global stepCounter
    global rate
    stepCounter = 0
    while stepCounter < numberOfSteps:
        rate.sleep()
    print 'finished set of steps'

def footstep_status_callback(msg):
    global stepCounter
    if msg.status == 1:
        stepCounter += 1

##sends a footstep, moving the left leg to set the leg aperture (distance between center of both feet, in meters)
def set_initial_leg_aperture(aperture):
    global steps_pub
    global tfBuffer
    global seq
    global MOVING_TIME
    if aperture > 0.0:
        footstepArrayMessage = FootstepDataListMessage()
        footstepArrayMessage.swing_time = MOVING_TIME
        footstepArrayMessage.transfer_time = MOVING_TIME
        footstepArrayMessage.unique_id = seq
        seq += 1
        rightFoot = createFootStepInPlace(RIGHT)
        footstepArrayMessage.footstep_data_list.append(createFootStepOffset(LEFT, [0.0, rightFoot.location.y + aperture, 0.0], 0.0))
        steps_pub.publish(footstepArrayMessage)
        waitForFootsteps(len(footstepArrayMessage.footstep_data_list))

def robot_mover():
    global rate
    global steps_pub
    global seq
    global tfBuffer
    global SPEED_LIMIT
    global ANGULAR_SPEED_LIMIT
    global MOVING_TIME
    global INITIAL_LEG_APERTURE
    seq = 1
    rospy.init_node('robot_mover', anonymous=True)
    SPEED_LIMIT = rospy.get_param('~linear_speed_limit', 0.03)
    ANGULAR_SPEED_LIMIT = rospy.get_param('~angular_speed_limit', 0.045)
    MOVING_TIME = rospy.get_param('~moving_time', 2.0)
    INITIAL_LEG_APERTURE = rospy.get_param('~initial_leg_aperture', 0.0)
    ##print ('Initial = ' + str(INITIAL_LEG_APERTURE))
    step_status_sub = rospy.Subscriber('ihmc_ros/valkyrie/output/footstep_status', FootstepStatusMessage, footstep_status_callback)
    steps_pub = rospy.Publisher('ihmc_ros/valkyrie/control/footstep_list', FootstepDataListMessage, queue_size=1)
    cmd_vel_sub = rospy.Subscriber('ihmc_ros/valkyrie/control/cmd_vel', Twist, cmd_vel_callback, queue_size=1)
    rate = rospy.Rate(10) # 10hz
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    if (INITIAL_LEG_APERTURE > 0.0):
        set_initial_leg_aperture(INITIAL_LEG_APERTURE)
    rospy.spin()

if __name__ == '__main__':
    try:
        robot_mover()
    except rospy.ROSInterruptException:
        pass
