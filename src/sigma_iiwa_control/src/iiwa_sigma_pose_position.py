#!/usr/bin/env python
import math as mt
import rospy
import numpy as np
from iiwa_msgs.msg import CartesianPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from sensor_msgs.msg import Joy
from pyquaternion import Quaternion
# from scipy.spatial. import Rotation as R
from scipy.spatial.transform.rotation import Rotation as R
from geometry_msgs.msg import WrenchStamped
from work_space_safer import WorkSafe
from data_record import DataRecord


# from src.iiwa_stack.iiwa_control.src import iiwa_sunrise

# a1 = -0.74
# a2 = 0.15
# a3 = 0.82
# a4 = 1.72
# a5 = 2.88
# a6 = 1.33
# a7 = 2.00
joint_flag = False
cartesian_pose = PoseStamped()
# cartesian_pose_2 = CartesianPose()

cartesian_save_pose=PoseStamped()

global_cart=PoseStamped()
sigma_pose = PoseStamped()
sigma_pose.pose.orientation.w = 1
rate_control_force = WrenchStamped()
endo_tool_pose = CartesianPose()

worksafe=WorkSafe()

data_record=DataRecord('ff1')

path_desired=Path()
training_force=WrenchStamped()

position_box = np.array([[0.0, 0.0,0.0], [0.0, 0.0,0.0]])

start_point=np.array([0.6645,0.0375,0.0530])

sample_velocity_vector = np.array([0, 0])

endo_tool_rotate = 0

diff_x, diff_y, diff_z, ox_diff, oy_diff, oz_diff, ow_diff = 0, 0, 0, 0, 0, 0, 1

# 与气泡控制相关的量
diff_x_origin, diff_y_origin, diff_z_origin = 0, 0, 0
rate_k = 0.006
force_k = 700
position_sphere_ratio = 0.07
rate_sphere_ratio = 0.09
rad_x, rad_y, rad_z = 0, 0, 0
position_control_flag = True
mode_change_flag = False


q0 = 0
q0_diff = 0
sigma_r = 0

position_ref_x = 0.0
position_ref_y = 0.0
position_ref_z = 0.0

orientation_ref_x = 0
orientation_ref_y = 0
orientation_ref_z = 0
orientation_ref_w = 1


# T = np.matrix([[1,0,0,0],[0,0,1,0.188703],[0,-1,0,0.157436],[0,0,0,1]])

# T=np.matrix([[1,0,0,-0.4817],
#              [0,1,0,0],
#              [0,0,1,0.08855],
#              [0,0,0,1]])

T = np.matrix([[1, 0, 0, 0],
               [0, 1, 0, 0],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])

T_iiwa_to_tool = np.matrix([[1, 0, 0, -0.4817],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0.08855],
                            [0, 0, 0, 1]])

sigma_flag = False
buttons_flag = 0


def cartesian_pose_callback_2(msg):
    global cartesian_pose_2
    cartesian_pose_2 = msg


def qua_to_rot(msg):
    end_ori_x = msg.poseStamped.pose.orientation.x
    end_ori_y = msg.poseStamped.pose.orientation.y
    end_ori_z = msg.poseStamped.pose.orientation.z
    end_ori_w = msg.poseStamped.pose.orientation.w

    end_Q = R.from_quat([end_ori_x, end_ori_y, end_ori_z, end_ori_w])
    end_R = end_Q.as_matrix()
    # print(f'{end_R}')
    return

# 将获取到的iiwa末端位姿转化为手术器械末端位姿


def iiwa_to_tool(msg):
    global endo_tool_pose, T_iiwa_to_tool
    iiwa_x = msg.poseStamped.pose.position.x
    iiwa_y = msg.poseStamped.pose.position.y
    iiwa_z = msg.poseStamped.pose.position.z
    iiwa_ori_x = msg.poseStamped.pose.orientation.x
    iiwa_ori_y = msg.poseStamped.pose.orientation.y
    iiwa_ori_z = msg.poseStamped.pose.orientation.z
    iiwa_ori_w = msg.poseStamped.pose.orientation.w

    # print(f'position1:{iiwa_x,iiwa_y,iiwa_z}')

    # 旋转矩阵
    iiwa_Q = R.from_quat([iiwa_ori_x, iiwa_ori_y, iiwa_ori_z, iiwa_ori_w])
    iiwa_R = iiwa_Q.as_matrix()
    # print(f'iiwa:{iiwa_R}')
    # print(f'iiwa:{[iiwa_ori_x,iiwa_ori_y,iiwa_ori_z,iiwa_ori_w]}')
    # 位置
    iiwa_P = np.matrix([[iiwa_x], [iiwa_y], [iiwa_z]])
    a = np.hstack((iiwa_R, iiwa_P))
    b = np.matrix([0, 0, 0, 1])
    iiwa_T = np.vstack((a, b))

    iiwa_tool_T = iiwa_T*T_iiwa_to_tool
    # print(f'iiwa:{iiwa_tool_T}')

    iiwa_tool_R = R.from_matrix([[iiwa_tool_T[0, 0], iiwa_tool_T[0, 1], iiwa_tool_T[0, 2]],
                                 [iiwa_tool_T[1, 0], iiwa_tool_T[1, 1],
                                     iiwa_tool_T[1, 2]],
                                 [iiwa_tool_T[2, 0], iiwa_tool_T[2, 1], iiwa_tool_T[2, 2]]])

    iiwa_tool_Q = iiwa_tool_R.as_quat()
    # print(f'Q:{iiwa_tool_Q}')
    endo_tool_pose.poseStamped.header = msg.poseStamped.header
    endo_tool_pose.redundancy = msg.redundancy
    endo_tool_pose.poseStamped.pose.position.x = iiwa_tool_T[0, 3]
    endo_tool_pose.poseStamped.pose.position.y = iiwa_tool_T[1, 3]
    endo_tool_pose.poseStamped.pose.position.z = iiwa_tool_T[2, 3]
    endo_tool_pose.poseStamped.pose.orientation.x = iiwa_tool_Q[0]
    endo_tool_pose.poseStamped.pose.orientation.y = iiwa_tool_Q[1]
    endo_tool_pose.poseStamped.pose.orientation.z = iiwa_tool_Q[2]
    endo_tool_pose.poseStamped.pose.orientation.w = iiwa_tool_Q[3]

    # print(f'position2:{iiwa_tool_T[0,3],iiwa_tool_T[1,3],iiwa_tool_T[2,3]}')

    return endo_tool_pose

# 获取笛卡尔坐标系


def cartesian_pose_callback(msg):
    global joint_flag
    joint_flag = True
    global cartesian_pose, position_box, sample_velocity_vector
    # qua_to_rot(msg)
    # cartesian_pose = iiwa_to_tool(msg)
    
    # print("flag")
    
    cartesian_pose = msg
    x_local = [cartesian_pose.pose.position.x,
               cartesian_pose.pose.position.y,
               cartesian_pose.pose.position.z]
    if position_box[0][0] == 0 and position_box[0][1] == 0 :
        x_before = x_local
        position_box[0] = x_before
        position_box[1] = x_before
    x_before = position_box[1]
    if np.sum((x_local-x_before)**2) > 0.0005:
        position_box[0] = x_before
        position_box[1] = x_local
        sample_velocity_vector = x_local-x_before
    # print(f'pose:{cartesian_pose}')
    # print(f'msg:{msg}')
    return

#期望轨迹获取
def path_callback(msg):
    global path_desired
    path_desired=msg
    return

# 获取sigma的四元数坐标
def pose_callback(msg):
    global sigma_flag
    sigma_flag = True
    global diff_x, diff_y, diff_z, ox_diff, oy_diff, oz_diff, ow_diff, q0, sigma_pose, q0_diff, sigma_r,position_box
    global diff_x_origin, diff_y_origin, diff_z_origin

    sigma_pose = msg
    position_x = msg.pose.position.x
    position_y = msg.pose.position.y
    position_z = msg.pose.position.z
    orientation_x = msg.pose.orientation.x
    orientation_y = msg.pose.orientation.y
    orientation_z = msg.pose.orientation.z
    orientation_w = msg.pose.orientation.w

    q0 = Quaternion(orientation_w, orientation_x, orientation_y, orientation_z)

    q0_ref_inv = Quaternion(
        orientation_ref_w, -orientation_ref_x, -orientation_ref_y, -orientation_ref_z)

    # q0_diff_inv = Quaternion(orientation_ref_w,orientation_ref_x,orientation_ref_y,orientation_ref_z)

    q0_diff0 = q0 * q0_ref_inv

    q0_diff = Quaternion(q0_diff0[0], -q0_diff0[1], -q0_diff0[2], q0_diff0[3])

    ox_diff = q0_diff[1]
    oy_diff = q0_diff[2]
    oz_diff = q0_diff[3]
    ow_diff = q0_diff[0]

    # sigma_q = R.from_quat([orientation_x, orientation_y, orientation_z, orientation_w])
    sigma_q = R.from_quat([ox_diff, oy_diff, oz_diff, ow_diff])
    # rr = sigma_q.as_quat()
    # print(rr)
    sigma_r = sigma_q.as_matrix()

    # orientation_ref_x = 0.0
    # orientation_ref_y = 0.0
    # orientation_ref_z = 0.0
    # orientation_ref_w = 1.0

    diff_x_origin = position_x - position_ref_x
    diff_y_origin = position_y - position_ref_y
    diff_z_origin = position_z - position_ref_z

    # diff_ox = orientation_x - orientation_ref_x
    # diff_oy = orientation_y - orientation_ref_y
    # diff_oz = orientation_z - orientation_ref_z
    # diff_ow = orientation_w - orientation_ref_w
    # print('msg.pose\n',msg.pose)
    # print(f'sub:{diff_x}')

    # diff_x,diff_y,diff_z=check_mode(diff_x_origin,diff_y_origin,diff_z_origin)
    
    diff_x = diff_x_origin*0.15
    diff_y = diff_y_origin*0.15
    diff_z = diff_z_origin*0.15  #10
    data_record.record_x_m_abs(sigma_pose)
    
    # print(diff_x,diff_y,diff_z)
    
    # diff_x,diff_y,diff_z=master_adaptive_control(diff_x_origin, diff_y_origin, diff_z_origin)  
    return


# 主手自适应控制
def master_adaptive_control(diff_x_origin, diff_y_origin, diff_z_origin):
    global rad_x, rad_y, rad_z, diff_x, diff_y, diff_z, predict, training, positionRate, cartesian_pose, sample_velocity_vector, position_box,path_desired,start_point,training_force
    global goal_pub
    #预测目标
    predict.get_local(np.array([position_box[1][0],position_box[1][1]]),sample_velocity_vector)
    predict.get_prob()
    goal_predict=predict.predict_goal
    if goal_predict>-1:
        goal_pub.publish(goal_predict)
    
    #动态气泡控制
    positionRate.get_x_init(diff_x_origin, diff_y_origin, diff_z_origin)
    positionRate.get_x_s(position_box[1][0],position_box[1][1],position_box[1][2])
    positionRate.get_x_start(start_point)
    if goal_predict>-1:
        positionRate.get_k_goal(goal_predict)
    positionRate.get_xsd()
    
    diff_x=positionRate.x_d[0]
    diff_y=positionRate.x_d[1]
    diff_z=positionRate.x_d[2]
    
    if len(path_desired.poses)!=0:
        training.get_path(path_desired)
        training.get_start_point(start_point)
        training.get_goal_point([predict.goal_position[goal_predict][0],predict.goal_position[goal_predict][1],0.0530])
        training.get_local(position_box[1],positionRate.v_master)
        training.get_force()
        training_force=training.Force
    return diff_x,diff_y,diff_z


def buttons_callback(msg):
    global buttons_flag,data_record
    buttons_flag = msg.buttons[0]
    data_record.record_pedral(msg.buttons[0])
    # print('buttons')
    return


# 改变控制模式 以参考位置为零点，检查当前位置与参考位置之间的距离，
# 当超出范围时，转变为速度模式（持续发布径向的位置，为假的速度控制),
def check_mode(diff_x_origin, diff_y_origin, diff_z_origin):

    global rad_x, rad_y, rad_z, diff_x, diff_y, diff_z, position_control_flag, mode_change_flag

    diff_r = mt.sqrt(diff_x_origin*diff_x_origin+diff_y_origin *
                     diff_y_origin+diff_z_origin*diff_z_origin)

    if diff_r < position_sphere_ratio:
        if not position_control_flag:
            position_control_flag = True
            mode_change_flag = True
        diff_x = diff_x_origin/4
        diff_y = diff_y_origin/4
        diff_z = diff_z_origin/4

    elif diff_r < rate_sphere_ratio:
        position_control_flag = False
        rad_x = (diff_x_origin-(diff_x_origin/diff_r)*position_sphere_ratio)
        rad_y = (diff_y_origin-(diff_y_origin/diff_r)*position_sphere_ratio)
        rad_z = (diff_z_origin-(diff_z_origin/diff_r)*position_sphere_ratio)

        # print('rad_x: ',rad_x,' rad_y: ',rad_y,' rad_z: ',rad_z)
        if (abs(diff_x) > abs((diff_x_origin/diff_r)*position_sphere_ratio/3)):
            diff_x = diff_x+rad_x*rate_k
            diff_y = diff_y+rad_y*rate_k
            diff_z = diff_z+rad_z*rate_k
        else:
            diff_x = (diff_x_origin/diff_r)*position_sphere_ratio/3+rad_x
            diff_y = (diff_y_origin/diff_r)*position_sphere_ratio/3+rad_y
            diff_z = (diff_z_origin/diff_r)*position_sphere_ratio/3+rad_z

    else:
        position_control_flag = False
        rad_x = (rate_sphere_ratio-position_sphere_ratio) * \
            (diff_x_origin/diff_r)
        rad_y = (rate_sphere_ratio-position_sphere_ratio) * \
            (diff_y_origin/diff_r)
        rad_z = (rate_sphere_ratio-position_sphere_ratio) * \
            (diff_z_origin/diff_r)

        if (abs(diff_x) > abs((diff_x_origin/diff_r)*position_sphere_ratio/3)):
            diff_x = diff_x+rad_x*rate_k
            diff_y = diff_y+rad_y*rate_k
            diff_z = diff_z+rad_z*rate_k
        else:
            diff_x = (diff_x_origin/diff_r) * \
                position_sphere_ratio/3+rad_x*rate_k
            diff_y = (diff_y_origin/diff_r) * \
                position_sphere_ratio/3+rad_y*rate_k
            diff_z = (diff_z_origin/diff_r) * \
                position_sphere_ratio/3+rad_z*rate_k

    # print('rad_x: ',rad_x,' rad_y: ',rad_y,' rad_z: ',rad_z)
    position_force(rad_x, rad_y, rad_z)

    # print('diff_x: ',diff_x,'diff_y: ',diff_y,'diff_z: ',diff_z)

    return diff_x, diff_y, diff_z

# 根据距离生成反馈力
def position_force(rad_x, rad_y, rad_z):
    global rate_control_force
    rate_control_force.wrench.force.x = -rad_x*force_k
    rate_control_force.wrench.force.y = -rad_y*force_k
    rate_control_force.wrench.force.z = -rad_z*force_k
    rate_control_force.header.stamp = rospy.Time.now()
    return rate_control_force


def main():
    global position_ref_x, position_ref_y, position_ref_z, orientation_ref_x, orientation_ref_y, orientation_ref_z, orientation_ref_w, rate_control_force
    global mode_change_flag, rate_control_force, rad_x, rad_y, rad_z,goal_pub,training_force,data_record,global_cart,buttons_flag,cartesian_save_pose,cartesian_pose
    rospy.init_node('iiwa_control')

    # rospy.Subscriber('/iiwa/state/CartesianPose_2', CartesianPose,
    #                  callback=cartesian_pose_callback, queue_size=1)
    rospy.Subscriber('/iiwa/state/CartesianPose_end', PoseStamped,
                     callback=cartesian_pose_callback)
    # rospy.Subscriber('/surgical/state/CartesianPose', PoseStamped,
    #                  callback=cartesian_pose_callback, queue_size=1)
    # rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, callback=cartesian_pose_callback_2, queue_size=10)
    rospy.Subscriber('/sigma7/sigma0/pose', PoseStamped,
                     callback=pose_callback, queue_size=1)
    # rospy.Subscriber('/sigma7/sigma0/buttons', Joy,
    #                  callback=buttons_callback, queue_size=1)
    rospy.Subscriber('/trajectory',Path,callback=path_callback,queue_size=1)
    rospy.Subscriber('/pedal/buttons', Joy, callback=buttons_callback, queue_size=1)
    # rospy.spin()

    # joint_pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
    # cartesian_pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=1)
    cartesian_pub = rospy.Publisher(
        '/iiwa/command/CartesianPose_origin', PoseStamped, queue_size=1)
    force_pub = rospy.Publisher(
        '/sigma/force_feedback', WrenchStamped, queue_size=1)
    goal_pub = rospy.Publisher('goal', Float32MultiArray, queue_size=1)

    rate = rospy.Rate(250)
    while not rospy.is_shutdown():
        # print(cartesian_pose)
        if cartesian_pose.pose.position.x != 0:
            cartesian_save_pose = cartesian_pose
            # print(cartesian_save_pose)
            print("---------------------------------flag1---------------------------------------------")
            # print(f'pose:{cartesian_pose}')
            break
        # if sigma_flag and joint_flag:
        #     data_flag = True
        #     break
    rate.sleep()
    
    print("---------------------------------flag2---------------------------------------------")

    while not rospy.is_shutdown():
        # start=rospy.get_time()

        # if data_flag:
        #     pose_x = cartesian_pose.position.x + diff_x
        #     pose_y = cartesian_pose.position.y + diff_y
        #     pose_z = cartesian_pose.position.z + diff_z
        #     data_flag = False
        # print(f'error is {cartesian_pose_2.poseStamped.pose.position.z-cartesian_pose.poseStamped.pose.position.z}')

        # print('mode_change_flag:',mode_change_flag)

        # print("cartesian init pose :",cartesian_save_pose)
        
        
        if  joint_flag and buttons_flag and not mode_change_flag:
            # print(cartesian_pose)
            # print('-----------------------------')

            iiwa_x = cartesian_save_pose.pose.position.x
            iiwa_y = cartesian_save_pose.pose.position.y
            iiwa_z = cartesian_save_pose.pose.position.z

            iiwa_ox = cartesian_save_pose.pose.orientation.x
            iiwa_oy = cartesian_save_pose.pose.orientation.y
            iiwa_oz = cartesian_save_pose.pose.orientation.z
            iiwa_ow = cartesian_save_pose.pose.orientation.w
            # print(f'pose:{cartesian_init_pose}')

            iiwa_q = R.from_quat([iiwa_ox, iiwa_oy, iiwa_oz, iiwa_ow])

            iiwa_r = iiwa_q.as_matrix()

            iiwa_p = np.matrix([[iiwa_x], [iiwa_y], [iiwa_z]])

            a = np.hstack((iiwa_r, iiwa_p))  # 横向组合两个矩阵
            b = np.matrix([0, 0, 0, 1])

            # iiwa当前位姿
            iiwa_T = np.vstack((a, b))  # 纵向组合两个矩阵 目标位姿矩阵
            # print(f'iiwa_T:{iiwa_T}')

            Tool_T = iiwa_T * T  # 默认 工具坐标系位姿矩阵
            # Tool_T = iiwa_T  # 默认 工具坐标系位姿矩阵

            Tool_r = np.matrix([[Tool_T[0, 0], Tool_T[0, 1], Tool_T[0, 2]],
                                [Tool_T[1, 0], Tool_T[1, 1], Tool_T[1, 2]],
                                [Tool_T[2, 0], Tool_T[2, 1], Tool_T[2, 2]]])

            Tool_p = np.matrix([[Tool_T[0, 3]],
                                [Tool_T[1, 3]],
                                [Tool_T[2, 3]]])

            Tool_r_new = sigma_r * Tool_r  # 工具坐标系位姿矩阵左乘 主手给的旋转矩阵

            Tool_p_diff = np.matrix([[-diff_x], [-diff_y], [diff_z]])

            unit = np.matrix([[1, 0, 0],
                              [0, 1, 0],
                              [0, 0, 1]])

            mat = np.hstack((unit, Tool_p))

            Tool_p_mat = np.vstack((mat, b))

            mat_diff = np.hstack((unit, Tool_p_diff))

            Tool_p_diff_mat = np.vstack((mat_diff, b))

            Tool_p_new_mat = Tool_p_diff_mat * Tool_p_mat  # 只计算位置的变化

            Tool_p_new = np.matrix([[Tool_p_new_mat[0, 3]],
                                    [Tool_p_new_mat[1, 3]],
                                    [Tool_p_new_mat[2, 3]]])

            c = np.hstack((Tool_r_new, Tool_p_new))

            Tool_T_new = np.vstack((c, b))  # 变换后的工具坐标系齐次矩阵

            T_inv = np.linalg.inv(T)  # 初始位姿矩阵的逆

            iiwa_T_new = Tool_T_new * T_inv  # 变为世界坐标系

            iiwa_r_new = R.from_matrix([[iiwa_T_new[0, 0], iiwa_T_new[0, 1], iiwa_T_new[0, 2]],
                                        [iiwa_T_new[1, 0], iiwa_T_new[1, 1],
                                            iiwa_T_new[1, 2]],
                                        [iiwa_T_new[2, 0], iiwa_T_new[2, 1], iiwa_T_new[2, 2]]])
            iiwa_q_new = iiwa_r_new.as_quat()

            iiwa_p_new = np.matrix([[iiwa_T_new[0, 3]],
                                    [iiwa_T_new[1, 3]],
                                    [iiwa_T_new[2, 3]]])

            cartesian_msg = PoseStamped()
            cartesian_msg.pose.position.x = iiwa_p_new[0, 0]
            cartesian_msg.pose.position.y = iiwa_p_new[1, 0]
            cartesian_msg.pose.position.z = iiwa_p_new[2, 0]
            # q1 = Quaternion(cartesian_init_pose.orientation.w,cartesian_init_pose.orientation.x,cartesian_init_pose.orientation.y,cartesian_init_pose.orientation.z)
            #
            # q_new = q0*q1
            cartesian_msg.pose.orientation.x = iiwa_q_new[0]
            cartesian_msg.pose.orientation.y = iiwa_q_new[1]
            cartesian_msg.pose.orientation.z = iiwa_q_new[2]
            cartesian_msg.pose.orientation.w = iiwa_q_new[3]

            cartesian_msg.header.frame_id = 'iiwa_link_0'
            cartesian_msg.header.stamp = rospy.Time.now()
            global_cart=cartesian_msg

            # print(cartesian_msg)
            force=worksafe.get_safe_force(np.array([iiwa_p_new[0, 0],iiwa_p_new[1, 0],iiwa_p_new[2, 0]]))
            training_force.wrench.force.x=force[0]
            training_force.wrench.force.y=force[1]
            training_force.wrench.force.z=force[2]
            training_force.header.stamp=rospy.Time.now()
            cartesian_pub.publish(cartesian_msg)
            # force_pub.publish(rate_control_force)
            # force_pub.publish(training_force)
            data_record.record_x_s(cartesian_msg)
            data_record.record_x_m(sigma_pose)
            data_record.record_x_s_abs(global_cart)
            # print(rate_control_force)

            # print(f'pose:{cartesian_msg}')

        else:
            
            # print("flag!!!!!")
            cartesian_save_pose = cartesian_pose

            position_ref_x = sigma_pose.pose.position.x
            position_ref_y = sigma_pose.pose.position.y
            position_ref_z = sigma_pose.pose.position.z
            orientation_ref_x = sigma_pose.pose.orientation.x
            orientation_ref_y = sigma_pose.pose.orientation.y
            orientation_ref_z = sigma_pose.pose.orientation.z
            orientation_ref_w = sigma_pose.pose.orientation.w

            rate_control_force.wrench.force.x = 0
            rate_control_force.wrench.force.y = 0
            rate_control_force.wrench.force.z = 0
            rad_x = 0
            rad_y = 0
            rad_z = 0

            # force_pub.publish(rate_control_force)

            mode_change_flag = False
            data_record.record_x_s_abs(global_cart)

            # print(f'while:{position_ref_x}')

            # print('----------------------------------------')
            # print('position_ref_x\n',position_ref_x,'\n','position_ref_y\n',position_ref_y,'\n','position_ref_z\n',position_ref_y)

        # print('feed back force: ',rate_control_force)
        # end=rospy.get_time()
        # print("------------duration----------",(end-start)*1000)
        rate.sleep()
    return


if __name__ == '__main__':
    main()
