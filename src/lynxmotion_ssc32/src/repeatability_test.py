#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

joint_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)
joint = JointState()
joint.name = ["base", "shoulder", "elbow", "wrist", "wrist_twist", "gripper"]
joint.position = [0, 0, 0, 0, 0, 0]

STATE_MOVING_DOWN = 0
STATE_MOVING_UP = 1
STATE_BASE_BACK = 2
STATE_BASE_FORTH = 3

current_state = STATE_MOVING_DOWN
counter = 0

TARGET_SHOULDER = -0.565
TARGET_ELBOW = 0.141
TARGET_WRIST = 0.353
TARGET_BASE = 0.5

TIMESTEPS = 100
SHOULDER_INCREMENT = TARGET_SHOULDER / TIMESTEPS
ELBOW_INCREMENT = TARGET_ELBOW / TIMESTEPS
WRIST_INCREMENT = TARGET_WRIST / TIMESTEPS
BASE_INCREMENT = TARGET_BASE / TIMESTEPS

def state_machine():
    global joint
    global current_state
    global counter

    if current_state == STATE_MOVING_DOWN:
        joint.position[1] += SHOULDER_INCREMENT # shoulder
        joint.position[2] += ELBOW_INCREMENT # elbow
        joint.position[3] += WRIST_INCREMENT # wrist
        
        counter += 1
        if counter >= TIMESTEPS:
            counter = 0
            current_state = STATE_MOVING_UP
            
    elif current_state == STATE_MOVING_UP:
        joint.position[1] -= SHOULDER_INCREMENT # shoulder
        joint.position[2] -= ELBOW_INCREMENT # elbow
        joint.position[3] -= WRIST_INCREMENT # wrist
        
        counter += 1
        if counter >= TIMESTEPS:
            counter = 0
            current_state = STATE_BASE_BACK
            
    elif current_state == STATE_BASE_BACK:
        joint.position[0] += BASE_INCREMENT # base
        
        counter += 1
        if counter >= TIMESTEPS:
            counter = 0
            current_state = STATE_BASE_FORTH
            
    elif current_state == STATE_BASE_FORTH:
        joint.position[0] -= BASE_INCREMENT # base
        
        counter += 1
        if counter >= TIMESTEPS:
            counter = 0
            current_state = STATE_MOVING_DOWN  # Start cycle again

    joint.header.stamp = rospy.Time.now()
    joint_publisher.publish(joint)
    rospy.sleep(.05)


def grip_pen():
    joint.position[0] = 0
    joint.position[1] = 0
    joint.position[2] = 0
    joint.position[3] = 0
    joint.position[4] = 0
    joint.position[5] = 0
    for i in range(5):
        joint.position[i] = 0
        joint.header.stamp = rospy.Time.now()
        joint_publisher.publish(joint)
        rospy.sleep(1)
    
    joint.position[5] = 1.411 # Gripper for pen
    joint.header.stamp = rospy.Time.now()
    joint_publisher.publish(joint)
    rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('Movement')
    try:
        grip_pen()

        while not rospy.is_shutdown():
            state_machine()
            
    except rospy.ROSInterruptException:
        pass