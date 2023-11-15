import rospy
import numpy as np
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.msg import JointQuantity

joint_msg=JointPosition()
# joint_init=[0.0,0.17444,0.0,-1.657222,0.0,-0.2616666,0.0]
# joint_init=[0,0.31851959,0,-1.771044,0,-0.518767,0]
joint_init=[0.6177676916122437, 1.3163151741027832, -1.7313841581344604, -1.0604543685913086, 0.9624993205070496, 0.8515076041221619, 0.45897889137268066]

def main():
    global joint_msg
    rospy.init_node('iiwa_init_pose')
    joints_pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        joint_msg.header.stamp=rospy.Time.now()
        joint_msg.position.a1=joint_init[0]
        joint_msg.position.a2=joint_init[1] 
        joint_msg.position.a3=joint_init[2]
        joint_msg.position.a4=joint_init[3]
        joint_msg.position.a5=joint_init[4]
        joint_msg.position.a6=joint_init[5]
        joint_msg.position.a7=joint_init[6]
        joints_pub.publish(joint_msg)
        rate.sleep()
    
    return   

if __name__ == '__main__':
    main()