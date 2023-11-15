#!/usr/bin/env python3
import rospy
import numpy as np
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.msg import JointQuantity

joint_msg = JointPosition()
# joint_init=[0.0,0.17444,0.0,-1.657222,0.0,-0.2616666,0.0]
# joint_init=[0,0.31851959,0,-1.771044,0,-0.518767,0]
# joint_init = [1.00513, 1.55648, -2.0804, -1.4425,0.786968,0.69464,0.707731]
joint_init = [0.8763135075569153, 1.6519771814346313, -2.2607247829437256, -1.2878414392471313,0.8307074308395386,0.9772767424583435,0.7848111987113953]
# joint_init=[57.59, 89.18, -119.2, -82.65, 45.09,39.80,40.55]
# joint_init=[  50.20906551,   94.65132035, -129.5299887 ,  -73.78787915,47.5960298 ,   55.99383276,   44.9663694 ]

# array([  53.95600853,   97.60450632, -130.04766851,  -91.16617957,
#          81.71180299,   41.11419087,  -76.85254793+90])

###

position=[0.7235244512557983,
          1.6749423742294312,
          -2.3842573165893555,
          -1.018060326576233,
          0.7624757885932922,
          1.3368651866912842,
          0.9566935300827026]


data=[-1.347450852394104, 
      1.4942214932034403e-07, 
      5.301244776489966e-09, 
      0.0]

###



def main():
    
    
    global joint_msg
    rospy.init_node('iiwa_init_pose')
    joints_pub = rospy.Publisher(
        '/iiwa/command/JointPosition', JointPosition, queue_size=10)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.position.a1 = joint_init[0]
        joint_msg.position.a2 = joint_init[1]
        joint_msg.position.a3 = joint_init[2]
        joint_msg.position.a4 = joint_init[3]
        joint_msg.position.a5 = joint_init[4]
        joint_msg.position.a6 = joint_init[5]
        joint_msg.position.a7 = joint_init[6]
        joints_pub.publish(joint_msg)
        rate.sleep()

    return


if __name__ == '__main__':
    main()
