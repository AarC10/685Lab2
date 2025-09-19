#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

joint_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)
joint = JointState()
joint.name = ["base", "shoulder", "elbow", "wrist", "wrist_twist", "gripper"]
joint.position = [0, 0, 0, 0, 0, 0]


TARGET_SHOULDER = -0.565
TARGET_ELBOW = 0.141
TARGET_WRIST = 0.353
TARGET_BASE = 0.5

TIMESTEPS = 100
SHOULDER_INCREMENT = TARGET_SHOULDER / TIMESTEPS
ELBOW_INCREMENT = TARGET_ELBOW / TIMESTEPS
WRIST_INCREMENT = TARGET_WRIST / TIMESTEPS
BASE_INCREMENT = TARGET_BASE / TIMESTEPS


STATE_HOME = 0 # Step 0
STATE_ROTATE_SHOULDER = 1 # Step 1
STATE_ROTATE_ELBOW = 2 # Step 2
STATE_ROTATE_WRIST = 3 # Step 3
STATE_OPEN_GRIPPER = 4 # Step 4
STATE_CLOSE_GRIPPER = 5 # Step 5

STEP_TIME = 0.5

HOME_TIME = 3
HOME_TIMESTEPS = int(HOME_TIME / STEP_TIME)



def set_gripper(open_position):
    target = 0 if open_position else 1.411
    for i in range(2):
        joint.position[5] = target
        joint.header.stamp = rospy.Time.now()
        joint_publisher.publish(joint)
        rospy.sleep(1)

def state_machine():
    global joint
    global current_state
    global counter

    if current_state == STATE_HOME:
        for i in range(6):
            joint.position[i] = 0

        current_state = STATE_ROTATE_SHOULDER
        counter += 1
        
        if counter >= HOME_TIMESTEPS:
            counter = 0
            current_state = STATE_ROTATE_SHOULDER

    elif current_state == STATE_ROTATE_SHOULDER:
        joint.position[1] += SHOULDER_INCREMENT
        
        counter += 1
        if counter >= TIMESTEPS:
            counter = 0
            current_state = STATE_ROTATE_ELBOW
            
    elif current_state == STATE_ROTATE_ELBOW:
        joint.position[2] += ELBOW_INCREMENT
        
        counter += 1
        if counter >= TIMESTEPS:
            counter = 0
            current_state = STATE_ROTATE_WRIST
            
    elif current_state == STATE_ROTATE_WRIST:
        joint.position[3] += WRIST_INCREMENT
        
        counter += 1
        if counter >= TIMESTEPS:
            counter = 0
            current_state = STATE_OPEN_GRIPPER
    elif current_state == STATE_OPEN_GRIPPER:
        set_gripper(open_position=True)
        counter = 0
        current_state = STATE_CLOSE_GRIPPER
    elif current_state == STATE_CLOSE_GRIPPER:
        set_gripper(open_position=False)
        rospy.sleep(10)
        counter = 0
        current_state = STATE_HOME

    joint.header.stamp = rospy.Time.now()
    joint_publisher.publish(joint)
    rospy.sleep(STEP_TIME)

def grip_pen():
    joint.position[0] = 0
    joint.position[1] = 0
    joint.position[2] = 0
    joint.position[3] = 0
    joint.position[4] = 0
    joint.position[5] = 0

if __name__ == '__main__':
    rospy.init_node('Movement')
    try:
        grip_pen()

        while not rospy.is_shutdown():
            state_machine()
            
    except rospy.ROSInterruptException:
        pass