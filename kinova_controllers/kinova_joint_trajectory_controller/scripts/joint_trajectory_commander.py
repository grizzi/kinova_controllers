#!/usr/bin/env python

from __future__ import print_function

import rospy
from sensor_msgs.msg import JointState

joint_state_desired = JointState()

def joint_state_callback(msg):
    global joint_state_desired
    joint_state_desired = msg


if __name__ == "__main__":
    subscriber = rospy.Subscriber("/joint_states_desired", JointState, joint_state_callback, queue_size=1)
    publisher = rospy.Publisher("/kinova_joint_trajectory_controller/goal", queue_size=1)

    while True:
        print("--------------------------------------")
        ans = input("Send current desired goal? [y/n]")
        if ans == 'y':
            message = "Sending the robot to: ["
            for i in range(7):
                message += str(joint_state_desired.position[i]) + " "
            print(message)
            publisher.publish(joint_state_desired)
        rospy.sleep(1.0)
