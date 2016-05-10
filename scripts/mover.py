#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import numpy
import importlib

LEFT = 0
RIGHT = 1

global steps_pub
global rate
global USING_NEW_MSGS
USING_NEW_MSGS = True

##minimum speed that would be considered a valid movement command (in any direction)
MIN_SPEED = 0.01

SWING_HEIGHT = 0.1

from geometry_msgs.msg import Twist
if USING_NEW_MSGS:
    from ihmc_msgs.msg import FootstepDataListRosMessage
    from ihmc_msgs.msg import FootstepDataRosMessage
    from ihmc_msgs.msg import FootstepStatusRosMessage
else:
    from ihmc_msgs.msg import FootstepDataListMessage
    from ihmc_msgs.msg import FootstepDataMessage
    from ihmc_msgs.msg import FootstepStatusMessage

def create_footstep():
    global USING_NEW_MSGS
    if USING_NEW_MSGS:
        return FootstepDataRosMessage()
    else:
        return FootstepDataMessage()

def create_footstep_list():
    global USING_NEW_MSGS
    if USING_NEW_MSGS:
        return FootstepDataListRosMessage()
    else:
        return FootstepDataListMessage()

##do not increment subscriber queue size, queueing Rosmessages is undesirable
def cmd_vel_callback(msg):
    global seq
    global steps_pub
    global SPEED_LIMIT
    global ANGULAR_SPEED_LIMIT
    footstepArrayRosMessage = create_footstep_list()
    footstepArrayRosMessage.swing_time = SWING_TIME
    footstepArrayRosMessage.transfer_time = TRANSFER_TIME
    footstepArrayRosMessage.unique_id = seq
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
    ##filter joystick noise
    if (linX < MIN_SPEED and linX > -MIN_SPEED): 
        linX = 0.0
    if (linY < MIN_SPEED and linY > -MIN_SPEED):
        linY = 0.0
    if (angZ < MIN_SPEED and angZ > -MIN_SPEED):
        angZ = 0.0

    currentRight = createFootStepInPlace(RIGHT)
    currentLeft = createFootStepInPlace(LEFT)
    if (abs(linX) >= MIN_SPEED or abs(linY) >= MIN_SPEED or abs(angZ) >=MIN_SPEED):
        if (linY <=0):
            futureRight = createFootStepOffset(RIGHT, [linX*5.0,linY*3.0,0.0], angZ * 5.0)
            futureLeft = createLeftfromRight(futureRight)
            footstepArrayRosMessage.footstep_data_list.append(futureRight)
            footstepArrayRosMessage.footstep_data_list.append(futureLeft)
            steps_pub.publish(footstepArrayRosMessage);
            waitForFootsteps(len(footstepArrayRosMessage.footstep_data_list));
        else:
            futureLeft = createFootStepOffset(LEFT, [linX*5.0,linY*3.0,0.0], angZ * 5.0)
            futureRight = createRightfromLeft(futureLeft)
            footstepArrayRosMessage.footstep_data_list.append(futureLeft)
            footstepArrayRosMessage.footstep_data_list.append(futureRight)
            steps_pub.publish(footstepArrayRosMessage);
            waitForFootsteps(len(footstepArrayRosMessage.footstep_data_list));

def createLeftfromRight(rightStep, distance = 0.35):
    leftStep = create_footstep()
    leftStep.robot_side = LEFT
    leftStep.swing_height = SWING_HEIGHT;
    offset = [0.0, distance, 0.0]
    quat = leftStep.orientation = rightStep.orientation
    rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    transformedOffset = numpy.dot(rot[0:3, 0:3], offset)

    leftStep.location.x = transformedOffset[0] + rightStep.location.x
    leftStep.location.y = transformedOffset[1] + rightStep.location.y
    leftStep.location.z = transformedOffset[2] + rightStep.location.z

    return leftStep

def createRightfromLeft(leftStep, distance = 0.35):
    rightStep = create_footstep()
    rightStep.robot_side = RIGHT
    leftStep.swing_height = SWING_HEIGHT
    offset = [0.0, -distance, 0.0]
    quat = rightStep.orientation = leftStep.orientation
    rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    transformedOffset = numpy.dot(rot[0:3, 0:3], offset)

    rightStep.location.x = transformedOffset[0] + leftStep.location.x
    rightStep.location.y = transformedOffset[1] + leftStep.location.y
    rightStep.location.z = transformedOffset[2] + leftStep.location.z

    return rightStep


def transformFootSteptoFootFrame(target_frame, step):
    global tfBuffer
    worldToFoot = tfBuffer.lookup_transform('world', target_frame, rospy.Time(), rospy.Duration(1.0))

def createFootStepInPlace(stepSide):
    global tfBuffer
    footstep = create_footstep()
    footstep.robot_side = stepSide
    footstep.swing_height = SWING_HEIGHT
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
    if aperture > 0.0:
        footstepArrayRosMessage = create_footstep_list()
        footstepArrayRosMessage.swing_time = SWING_TIME
        footstepArrayRosMessage.transfer_time = TRANSFER_TIME
        footstepArrayRosMessage.unique_id = seq
        seq += 1
        rightFoot = createFootStepInPlace(RIGHT)
        leftStep = createLeftfromRight(rightFoot, aperture)
        leftStep.swing_height = SWING_HEIGHT
        footstepArrayRosMessage.footstep_data_list.append(leftStep)
        steps_pub.publish(footstepArrayRosMessage)
        waitForFootsteps(len(footstepArrayRosMessage.footstep_data_list))

def test():
    global seq
    while True:
        footstepArrayRosMessage = create_footstep_list()
        footstepArrayRosMessage.swing_time = SWING_TIME
        footstepArrayRosMessage.transfer_time = TRANSFER_TIME
        footstepArrayRosMessage.swing_height = SWING_HEIGHT
        footstepArrayRosMessage.unique_id = seq
        seq += 1
        footstepArrayRosMessage.footstep_data_list.append(createFootStepInPlace(LEFT));
        footstepArrayRosMessage.footstep_data_list.append(createFootStepInPlace(RIGHT));
        steps_pub.publish(footstepArrayRosMessage);
        waitForFootsteps(len(footstepArrayRosMessage.footstep_data_list));

def robot_mover():
    global rate
    global steps_pub
    global seq
    global tfBuffer
    global SPEED_LIMIT
    global ANGULAR_SPEED_LIMIT
    global SWING_TIME
    global TRANSFER_TIME
    global INITIAL_LEG_APERTURE
    global SWING_HEIGHT
    global USING_NEW_MSGS
    seq = 1
    rospy.init_node('robot_mover', anonymous=True)
    SPEED_LIMIT = rospy.get_param('~linear_speed_limit', 0.03)
    ANGULAR_SPEED_LIMIT = rospy.get_param('~angular_speed_limit', 0.045)
    SWING_TIME = rospy.get_param('~swing_time', 2.0)
    TRANSFER_TIME = rospy.get_param('~transfer_time', 2.0)
    INITIAL_LEG_APERTURE = rospy.get_param('~initial_leg_aperture', 0.0)
    SWING_HEIGHT = rospy.get_param('~swing_height', 0.1)
    if (INITIAL_LEG_APERTURE > 0.0):
        print ('Initial leg aperture = ' + str(INITIAL_LEG_APERTURE))
    if USING_NEW_MSGS:
        step_status_sub = rospy.Subscriber('ihmc_ros/valkyrie/output/footstep_status', FootstepStatusRosMessage, footstep_status_callback)
        steps_pub = rospy.Publisher('ihmc_ros/valkyrie/control/footstep_list', FootstepDataListRosMessage, queue_size=1)
    else:
        step_status_sub = rospy.Subscriber('ihmc_ros/valkyrie/output/footstep_status', FootstepStatusMessage, footstep_status_callback)
        steps_pub = rospy.Publisher('ihmc_ros/valkyrie/control/footstep_list', FootstepDataListMessage, queue_size=1)
    cmd_vel_sub = rospy.Subscriber('ihmc_ros/valkyrie/control/cmd_vel', Twist, cmd_vel_callback, queue_size=1)  
    rate = rospy.Rate(10) # 10hz
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    if (INITIAL_LEG_APERTURE > 0.0):
        set_initial_leg_aperture(INITIAL_LEG_APERTURE)
    ##test()
    rospy.spin()

if __name__ == '__main__':
    try:
        robot_mover()
    except rospy.ROSInterruptException:
        pass
