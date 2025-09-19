#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

joint_publisher = rospy.Publisher('/custom_joint_states', JointState, queue_size=10)
joint = JointState()
joint.name = ["base", "shoulder", "elbow", "wrist", "wrist_twist", "gripper"]
joint.position = [0, 0, 0, 0, 0, 0]
THETA_INCREMENT = 0.3
reverse = False
counter = 0

def move0():
    global joint
    global reverse
    global counter
    increment = THETA_INCREMENT
    if reverse:
        increment *= -1

    joint.position[0] += increment # base
    joint.position[1] += increment # shoulder
    joint.position[2] += increment # elbow
    joint.position[3] += increment # wrist
    # Dont do wrist twist
    joint.position[5] += increment # gripper
    joint.header.stamp = rospy.Time.now()

    counter += 1
    if counter > 3:
        counter = 0
        reverse = not reverse

    joint_publisher.publish(joint)
    rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('Movement')
    try:
        joint.position[0] = 0
        joint.position[1] = 0
        joint.position[2] = 0
        joint.position[3] = 0
        joint.position[4] = 0
        joint.position[5] = 0

        while not rospy.is_shutdown():
            move0()
            
    except rospy.ROSInterruptException:
        pass