#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

joint_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)
joint = JointState()
joint.name = ["base", "shoulder", "elbow", "wrist", "wrist_twist", "gripper"]
joint.position = [0, 0, 0, 0, 0, 0]
THETA_INCREMENT = 0.01
reverse = False
counter = 0

TARGET_SHOULDER = -0.565
TARGET_ELBOW = 0.141
TARGET_WRIST = 0.353

TIMESTEPS = 100
SHOULDER_INCREMENT = TARGET_SHOULDER / TIMESTEPS
ELBOW_INCREMENT = TARGET_ELBOW / TIMESTEPS
WRIST_INCREMENT = TARGET_WRIST / TIMESTEPS

def move0():
    global joint
    global reverse
    global counter

    if reverse:
        joint.position[1] -= SHOULDER_INCREMENT # shoulder
        joint.position[2] -= ELBOW_INCREMENT # elbow
        joint.position[3] -= WRIST_INCREMENT # wrist
    else:
        joint.position[1] += SHOULDER_INCREMENT # shoulder
        joint.position[2] += ELBOW_INCREMENT # elbow
        joint.position[3] += WRIST_INCREMENT # wrist

    joint.header.stamp = rospy.Time.now()

    counter += 1
    if counter > TIMESTEPS:
        counter = 0
        reverse = not reverse

    joint_publisher.publish(joint)
    rospy.sleep(.05)


if __name__ == '__main__':
    rospy.init_node('Movement')
    try:
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



        while not rospy.is_shutdown():
            move0()
            
    except rospy.ROSInterruptException:
        pass