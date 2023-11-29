import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform.rotation import Rotation as R


def main():
    
    msg=PoseStamped()
    
    init_position=[0.642, 0.06,0.05]
    angle = 90
    # 创建Rotation对象
    rotation = R.from_euler('y', angle, degrees=True).as_quat()
    
    msg.pose.position.x=init_position[0]
    msg.pose.position.y=init_position[1]
    msg.pose.position.z=init_position[2]
    msg.pose.orientation.x=rotation[0]
    msg.pose.orientation.y=rotation[1]
    msg.pose.orientation.z=rotation[2]
    msg.pose.orientation.w=rotation[3]
    
    rospy.init_node('iiwa_init_pose')
    joints_pub = rospy.Publisher(
        "/iiwa/command/CartesianPose_origin", PoseStamped, queue_size=1)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        joints_pub.publish(msg)
        rate.sleep()
    return


if __name__ == '__main__':
    main()