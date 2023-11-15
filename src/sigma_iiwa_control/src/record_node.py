#!/usr/bin/env python
import math as mt
import rospy
import numpy as np
from iiwa_msgs.msg import CartesianPose
from iiwa_msgs.msg import JointPosition
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import TwistStamped
import os

iiwa_joints = JointPosition()
iiwa_pose = CartesianPose()
sigma_pose = PoseStamped()
sigma_force = WrenchStamped()
sigma_twist=TwistStamped()

flag = 0

root_path = '/home/chy/sigma_iiwa_simulation/sigma_to_sim_workspace/data'

iiwa_joint_file_name = ''
iiwa_pose_file_name = ''
sigma_pose_file_name = ''
sigma_force_file_name = ''
sigma_twist_file_name = ''

OPERATOR = 0
GROUP = 0
TIME = 0


def init_file_name():
    global OPERATOR, TIME, GROUP, iiwa_joint_file_name, iiwa_pose_file_name, sigma_pose_file_name, sigma_force_file_name, sigma_twist_file_name, root_path
    root_path = root_path+'/'
    iiwa_joint_file_name = root_path+'iiwa_Joints_of_Operator_' + \
        str(OPERATOR)+'_'+str(GROUP)+'_'+str(TIME)+'.txt'
    iiwa_pose_file_name = root_path+'iiwa_Pose_of_Operator_' + \
        str(OPERATOR)+'_'+str(GROUP)+'_'+str(TIME)+'.txt'
    sigma_pose_file_name = root_path+'sigma_Pose_of_Operator_' + \
        str(OPERATOR)+'_'+str(GROUP)+'_'+str(TIME)+'.txt'
    sigma_force_file_name = root_path+'sigma_Force_of_Operator_' + \
        str(OPERATOR)+'_'+str(GROUP)+'_'+str(TIME)+'.txt'
    sigma_twist_file_name = root_path+'sigma_Twist_of_Operator_' + \
        str(OPERATOR)+'_'+str(GROUP)+'_'+str(TIME)+'.txt'


def boot_file():
    global iiwa_joint_file_name, iiwa_pose_file_name, sigma_pose_file_name, sigma_force_file_name, sigma_twist_file_name,root_path, file_iiwa_joint, file_iiwa_pose, file_sigma_pose, file_sigma_force, file_sigma_twist
    file_iiwa_joint = open(iiwa_joint_file_name, 'w')
    file_iiwa_pose = open(iiwa_pose_file_name, 'w')
    file_sigma_pose = open(sigma_pose_file_name, 'w')
    file_sigma_force = open(sigma_force_file_name, 'w')
    file_sigma_twist = open(sigma_twist_file_name, 'w')


def close_file():
    global file_iiwa_joint, file_iiwa_pose, file_sigma_pose, file_sigma_force,file_sigma_twist
    file_iiwa_joint.close()
    file_iiwa_pose.close()
    file_sigma_pose.close()
    file_sigma_force.close()
    file_sigma_twist.close()


def iiwa_joint_record_callback(msg):
    global file_iiwa_joint
    iiwa_joints = msg
    msg_array = np.zeros((7, 1), dtype=np.float)
    msg_array[0] = iiwa_joints.position.a1
    msg_array[1] = iiwa_joints.position.a2
    msg_array[2] = iiwa_joints.position.a3
    msg_array[3] = iiwa_joints.position.a4
    msg_array[4] = iiwa_joints.position.a5
    msg_array[5] = iiwa_joints.position.a6
    msg_array[6] = iiwa_joints.position.a7
    record(msg_array, file_iiwa_joint)
    return


def iiwa_pose_record_callback(msg):
    global file_iiwa_pose
    iiwa_pose = msg
    msg_array = np.zeros((7, 1), dtype=np.float)
    msg_array[0] = iiwa_pose.poseStamped.pose.position.x
    msg_array[1] = iiwa_pose.poseStamped.pose.position.y
    msg_array[2] = iiwa_pose.poseStamped.pose.position.z
    msg_array[3] = iiwa_pose.poseStamped.pose.orientation.x
    msg_array[4] = iiwa_pose.poseStamped.pose.orientation.y
    msg_array[5] = iiwa_pose.poseStamped.pose.orientation.z
    msg_array[6] = iiwa_pose.poseStamped.pose.orientation.w
    record(msg_array, file_iiwa_pose)
    return


def sigma_pose_record_callback(msg):
    global file_sigma_pose, flag

    sigma_pose = msg
    msg_array = np.zeros((7, 1), dtype=np.float)
    msg_array[0] = sigma_pose.pose.position.x
    msg_array[1] = sigma_pose.pose.position.y
    msg_array[2] = sigma_pose.pose.position.z
    msg_array[3] = sigma_pose.pose.orientation.x
    msg_array[4] = sigma_pose.pose.orientation.y
    msg_array[5] = sigma_pose.pose.orientation.z
    msg_array[6] = sigma_pose.pose.orientation.w
    flag += 1
    if flag == 9:
        record(msg_array, file_sigma_pose)
        flag = 0

    return


def sigma_force_record_callback(msg):
    global file_sigma_force,sigma_force
    sigma_force = msg
    msg_array = np.zeros((6, 1), dtype=np.float)
    msg_array[0] = sigma_force.wrench.force.x
    msg_array[1] = sigma_force.wrench.force.y
    msg_array[2] = sigma_force.wrench.force.z
    msg_array[3] = sigma_force.wrench.torque.x
    msg_array[4] = sigma_force.wrench.torque.y
    msg_array[5] = sigma_force.wrench.torque.z
    record(msg_array, file_sigma_force)
    return

def sigma_twist_record_callback(msg):
    global file_sigma_twist,sigma_twist
    sigma_twist=msg
    msg_array = np.zeros((6, 1), dtype=np.float)
    msg_array[0] = sigma_twist.twist.linear.x
    msg_array[1] = sigma_twist.twist.linear.y
    msg_array[2] = sigma_twist.twist.linear.z
    msg_array[3] = sigma_twist.twist.angular.x
    msg_array[4] = sigma_twist.twist.angular.y
    msg_array[5] = sigma_twist.twist.angular.z
    record(msg_array,file_sigma_twist)
    


def record(array, file):
    for i in range(np.size(array)):
        file.write(str(array[i][0])+' ')
    file.write('\n')
    return


def check_file_name():
    global iiwa_joint_file_name, iiwa_pose_file_name, sigma_pose_file_name, sigma_force_file_name
    if check(iiwa_joint_file_name) and check(iiwa_pose_file_name) and check(sigma_pose_file_name) and check(sigma_force_file_name):
        print('------------文件名检查通过------------')
        return True
    else:
        print('------------文件名重复------------')
        return False


def check(name):
    if os.access(name, os.F_OK):
        return False
    else:
        return True


def main():
    global OPERATOR, TIME, GROUP, root_path
    OPERATOR = input('-请输入操作者编号：')
    root_path = root_path+'/operator'+str(OPERATOR)
    if not os.path.exists(root_path):
        os.makedirs(root_path)
    GROUP = input('-请输入实验组号：')
    TIME = input('-请输入第几次：')
    init_file_name()

    if check_file_name():
        boot_file()
        rospy.init_node('record_msg')
        iiwa_joint_sub = rospy.Subscriber('/iiwa/state/JointPosition_2', JointPosition,
                                          callback=iiwa_joint_record_callback, queue_size=10)
        iiwa_pose_sub = rospy.Subscriber('/iiwa/state/CartesianPose_2', CartesianPose,
                                         callback=iiwa_pose_record_callback, queue_size=10)
        sigma_pose_sub = rospy.Subscriber('/sigma7/sigma0/pose', PoseStamped,
                                          callback=sigma_pose_record_callback, queue_size=10)
        sigma_force_sub = rospy.Subscriber('/sigma/force_feedback', WrenchStamped,
                                           callback=sigma_force_record_callback, queue_size=10)
        sigma_twist_sub=rospy.Subscriber('/sigma7/sigma0/twist', TwistStamped,
                                           callback=sigma_twist_record_callback, queue_size=10)

        rospy.spin()
        print('操作者：'+str(OPERATOR)+' 第 '+str(GROUP)+' 组 第 '+str(TIME)+' 次数据记录完成')
        close_file()

    else:
        print('failed')
    return


if __name__ == "__main__":
    main()
