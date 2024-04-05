#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def pose_callback(data):
    # 获取时间戳
    timestamp = data.header.stamp
    # 获取位置信息
    position = data.pose.position
    # 获取姿态信息
    orientation = data.pose.orientation

    # 将数据格式化为字符串
    pose_data = "{} {} {} {} {} {} {} {}\n".format(rospy.Time.now().to_nsec(),
                                                   position.x, position.y, position.z,
                                                   orientation.x, orientation.y, orientation.z, orientation.w)

    # 将数据写入txt文件
    with open("pose_data.txt", "a") as file:
        file.write(pose_data)

def listener():
    rospy.init_node('pose_listener', anonymous=True)
    rospy.Subscriber("/surgical/state/CartesianPose", PoseStamped, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()