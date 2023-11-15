#!/usr/bin/env python
import rospy
import numpy as np
from iiwa_msgs.msg import JointVelocity
from iiwa_msgs.msg import JointQuantity

joint_msg=JointVelocity()

def velocity_callback(msg):
    global joint_msg
    joint_msg.header.stamp=rospy.Time.now()
    joint_msg.velocity.a1=msg.velocity.a1
    joint_msg.velocity.a2=msg.velocity.a2
    joint_msg.velocity.a3=msg.velocity.a3
    joint_msg.velocity.a4=msg.velocity.a4
    joint_msg.velocity.a5=msg.velocity.a5
    joint_msg.velocity.a6=msg.velocity.a6
    joint_msg.velocity.a7=msg.velocity.a7

def main():
    global joint_msg
    rospy.init_node('iiwa_velocity_read')
    joints_pub = rospy.Publisher(
        '/iiwa/state/JointVelocity_2', JointVelocity, queue_size=1)
    rospy.Subscriber(
        '/iiwa/state/JointVelocity', JointVelocity, callback=velocity_callback,queue_size=1)
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        joints_pub.publish(joint_msg)
        rate.sleep()
    return


if __name__ == '__main__':
    main()