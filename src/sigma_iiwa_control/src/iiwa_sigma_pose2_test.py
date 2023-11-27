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
from position_rate_control import PositionRate
from target_prediction import TagetPredict
from training_param_set import Training
from geometry_msgs.msg import TwistStamped
from data_record import DataRecord
from pose_adaptive import PoseAdaptive


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
cartesian_init_pose = PoseStamped()
cartesian_pose_2 = CartesianPose()
sigma_pose = PoseStamped()
sigma_twist = TwistStamped()
sigma_pose.pose.orientation.w = 1
rate_control_force = WrenchStamped()
endo_tool_pose = CartesianPose()

predict = TagetPredict()
training = Training()
poseAdaptive = PoseAdaptive()
positionRate = PositionRate()
path_desired = Path()
training_force = WrenchStamped()

time_before = 0.0
time_now = 0.0

global_cart = PoseStamped()

data_record = DataRecord('ff2')

sigma_velocity = np.array([0.0, 0.0, 0.0])

position_box = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

start_point = np.array([0.642, 0.06, 0.01])

sample_velocity_vector = np.array([0.0, 0.0, 0.0])
x_local = np.array([0.0, 0.0, 0.0])
x_before = np.array([0.0, 0.0, 0.0])
m_velocity = np.array([0.0, 0.0, 0.0])

s_velocity = np.array([0.0, 0.0])

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

# orientation_ref_x = 0
# orientation_ref_y = 0
# orientation_ref_z = 0
# orientation_ref_w = 1

state_flag = False


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
    global cartesian_pose, position_box, sample_velocity_vector, x_local, x_before, euler_local,qua_local, s_velocity
    global time_before, time_now
    # qua_to_rot(msg)
    # cartesian_pose = iiwa_to_tool(msg)

    cartesian_pose = msg
    x_local = np.array([cartesian_pose.pose.position.x,
                        cartesian_pose.pose.position.y,
                        cartesian_pose.pose.position.z])

    euler_local = R.from_quat([cartesian_pose.pose.orientation.x,
                               cartesian_pose.pose.orientation.y,
                               cartesian_pose.pose.orientation.z,
                               cartesian_pose.pose.orientation.w]).as_euler('zyx')
    
    qua_local=[cartesian_pose.pose.orientation.x,
                               cartesian_pose.pose.orientation.y,
                               cartesian_pose.pose.orientation.z,
                               cartesian_pose.pose.orientation.w]
    a = PoseStamped
    if position_box[0][0] == 0.0 and position_box[0][1] == 0.0:
        x_before = x_local
        position_box[0] = x_before
        position_box[1] = x_before

        time_now = msg.header.stamp.to_sec()

    x_before = position_box[1]
    if np.sqrt(np.sum((x_local-x_before)**2)) > 0.001:
        sample_velocity_vector = x_local-x_before
        position_box[0] = x_before
        position_box[1] = x_local

        time_before = time_now
        time_now = msg.header.stamp.to_sec()

        # print("sample: ",sample_velocity_vector)

    s_velocity = sample_velocity_vector/(time_now-time_before)

    # print("x_local: ",x_local)
    # print("x_before: ",x_before)
    # print(f'pose:{cartesian_pose}')
    # print(f'msg:{msg}')
    return

# 期望轨迹获取


def path_callback(msg):
    global path_desired
    path_desired = msg
    return


def twist_callback(msg):
    global sigma_twist, m_velocity
    sigma_twist = msg
    m_velocity[0] = sigma_twist.twist.linear.x
    m_velocity[1] = sigma_twist.twist.linear.y
    m_velocity[2] = -sigma_twist.twist.linear.z
    return

# 获取sigma的四元数坐标


def pose_callback(msg):
    global sigma_flag
    sigma_flag = True
    global diff_x, diff_y, diff_z, ox_diff, oy_diff, oz_diff, ow_diff, q0, sigma_pose, q0_diff, sigma_r
    global diff_x_origin, diff_y_origin, diff_z_origin,orientation_w, orientation_x, orientation_y, orientation_z,orientation_ref_w, orientation_ref_x, orientation_ref_y, orientation_ref_z
    
    # startt=rospy.Time.now()
    
    # 主手的姿态是反的
    sigma_ori_world=R.from_euler('z',mt.pi).as_matrix()
    
    sigma_pose = msg
    position_x = msg.pose.position.x
    position_y = msg.pose.position.y
    position_z = msg.pose.position.z
    
    orientation_x_0 = msg.pose.orientation.x
    orientation_y_0 = msg.pose.orientation.y
    orientation_z_0 = msg.pose.orientation.z
    orientation_w_0 = msg.pose.orientation.w

    q0 = R.from_quat([orientation_x_0,orientation_y_0,orientation_z_0,orientation_w_0]).as_matrix()
    # print("------------------------------")
    # print(q0)
    
    q0_plus=sigma_ori_world.T@q0
    # rot=sigma_ori_world.T@q0_plus
    
    # r_zero=R.identity().as_matrix()
    # q0_final=rot.T@r_zero
    
    # print(q0_final)
    
    quat_0=R.from_matrix(q0_plus).as_quat()
    
    orientation_x = quat_0[0]
    orientation_y = quat_0[1]
    orientation_z = quat_0[2]
    orientation_w = quat_0[3]
    
    sigma_pose.pose.orientation.x=orientation_x
    sigma_pose.pose.orientation.y=orientation_y
    sigma_pose.pose.orientation.z=orientation_z
    sigma_pose.pose.orientation.w=orientation_w
    
    


    q0_ref = R.from_quat(
        [orientation_ref_x, orientation_ref_y, orientation_ref_z,orientation_ref_w]).as_matrix()

    # q0_diff_inv = Quaternion(orientation_ref_w,orientation_ref_x,orientation_ref_y,orientation_ref_z)

    q0_diff_r = q0_plus @ q0_ref.T
    
    q0_diff=R.from_matrix(q0_diff_r).as_quat()

    # ox_diff = q0_diff[1]
    # oy_diff = q0_diff[2]
    # oz_diff = q0_diff[3]
    # ow_diff = q0_diff[0]

    ox_diff_origin = q0_diff[0]
    oy_diff_origin = q0_diff[1]
    oz_diff_origin = q0_diff[2]
    ow_diff_origin = q0_diff[3]

    # sigma_q = R.from_quat([orientation_x, orientation_y, orientation_z, orientation_w])

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

    # diff_x = diff_x_origin/4
    # diff_y = diff_y_origin/4
    # diff_z = diff_z_origin/4
    # print("d_x:%f ,d_y: %f,d_z:%f"%(diff_x_origin,diff_y_origin,diff_z_origin))
    # start=rospy.get_time()
    data_record.record_x_m_abs(sigma_pose)
    diff_x, diff_y, diff_z, ox_diff, oy_diff, oz_diff, ow_diff = master_adaptive_control(
        diff_x_origin, diff_y_origin, diff_z_origin, ox_diff_origin, oy_diff_origin, oz_diff_origin, ow_diff_origin)
    # end=rospy.get_time()
    # print("------------duration----------",(end-start)*1000)

    sigma_q = R.from_quat([ox_diff, oy_diff, oz_diff, ow_diff])
    # rr = sigma_q.as_quat()
    # print(rr)
    sigma_r = sigma_q.as_matrix()
    
    # endd=rospy.Time.now()
    
    # print("duration: %f ms"%((endd-startt).nsecs/1e6))
    

    return


# 主手自适应控制
def master_adaptive_control(diff_x_origin, diff_y_origin, diff_z_origin, ox_diff_origin, oy_diff_origin, oz_diff_origin, ow_diff_origin):
    global rad_x, rad_y, rad_z, diff_x, diff_y, diff_z, predict, training, cartesian_init_pose
    global positionRate, cartesian_pose, sample_velocity_vector, position_box, path_desired, start_point
    global training_force, x_local, buttons_flag, m_velocity, poseAdaptive, euler_local,qua_local, s_velocity, state_flag
    global orientation_ref_w, orientation_ref_x, orientation_ref_y, orientation_ref_z,ox_diff,oy_diff,oz_diff,ow_diff
    global orientation_w, orientation_x, orientation_y, orientation_z

    

    # 预测目标
    # start=rospy.get_time()
    # print("x_sample:",position_box[1])
    predict.get_local(
        np.array([position_box[1][0], position_box[1][1]]), sample_velocity_vector)
    predict.get_anti_force(training.anti_force)
    predict.get_prob()

    goal_predict = predict.predict_goal
    # goal_predict=1
    goal_trust = predict.goal_trust
    # goal_trust=1.0
    # print('-------------goal:%d------------'%(goal_predict))
    # print(predict.prob_before)
    # print('-------------goal trust:%f------------'%(goal_trust))

    # print("sample velocity: ",sample_velocity_vector)

    # 动态气泡控制
    positionRate.get_x_init(diff_x_origin, diff_y_origin,
                            diff_z_origin, buttons_flag, m_velocity)
    positionRate.get_x_s(x_local)
    positionRate.get_x_start(start_point)
    if goal_predict > -1:
        positionRate.get_k_goal(goal_predict, goal_trust)
    positionRate.get_xsd()

    # print("s_velocity: ",s_velocity)
    # print("o_ref after:",orientation_ref_x, orientation_ref_y, orientation_ref_z,orientation_ref_w)
    # if np.linalg.norm(positionRate.v_master) > 0:
    #     state_flag = True

    #     poseAdaptive.get_local(x_local)

    #     poseAdaptive.load_local_save_pose(cartesian_init_pose)

    #     qua_diff = poseAdaptive.next_pose(
    #         1.0/250, euler_local, s_velocity, goal_predict)

    #     # print("qua_diff:", qua_diff)
    #     ox_diff = qua_diff[0]
    #     oy_diff = qua_diff[1]
    #     oz_diff = qua_diff[2]
    #     ow_diff = qua_diff[3]
    # else:
    #     if state_flag:
    #         # print(ox_diff,oy_diff,oz_diff,ow_diff)
    #         # print("o_ref before:",orientation_ref_x, orientation_ref_y, orientation_ref_z,orientation_ref_w)
    #         [orientation_ref_x, orientation_ref_y, orientation_ref_z,orientation_ref_w]=reset_sigma_ref([ox_diff,oy_diff,oz_diff,ow_diff],[orientation_x, orientation_y, orientation_z,orientation_w])
    #         # print("o_ref after:",orientation_ref_x, orientation_ref_y, orientation_ref_z,orientation_ref_w)
            
    #         ox_diff_origin=ox_diff 
    #         oy_diff_origin=oy_diff
    #         oz_diff_origin=oz_diff 
    #         ow_diff_origin=ow_diff 
    #         # print(ox_diff,oy_diff,oz_diff,ow_diff)
    #         # print(state_flag)
    #         state_flag = False 
            
    #     # print(ox_diff,oy_diff,oz_diff,ow_diff)
    #     ox_diff = ox_diff_origin
    #     oy_diff = oy_diff_origin
    #     oz_diff = oz_diff_origin
    #     ow_diff = ow_diff_origin
        
    #     # print(ox_diff,oy_diff,oz_diff,ow_diff)
    
    ox_diff = ox_diff_origin
    oy_diff = oy_diff_origin
    oz_diff = oz_diff_origin
    ow_diff = ow_diff_origin
    
    diff_x = positionRate.x_d[0]
    diff_y = positionRate.x_d[1]
    diff_z = positionRate.x_d[2]
    # print("diff_x:",diff_x)
    # print("diff_y:",diff_y)
    # print("diff_z:",diff_z)
    # print("-----------------------")

    training.get_local(x_local, positionRate.v_master, positionRate.a_master)
    training.get_goal(goal_predict, buttons_flag)
    training.get_goal_trust(goal_trust)

    training.get_force()

    force = positionRate.boundary_force+training.Force

    training_force.wrench.force.x = force[0]
    training_force.wrench.force.y = force[1]
    training_force.wrench.force.z = force[2]

    # training_force.wrench.force.z=0.0

    training_force.header.stamp = rospy.Time.now()

    return diff_x, diff_y, diff_z, ox_diff, oy_diff, oz_diff, ow_diff


def buttons_callback(msg):
    global buttons_flag, data_record
    buttons_flag = True
    buttons_flag = msg.buttons[0]
    data_record.record_pedral(msg.buttons[0])
    # print('buttons')
    return

def reset_sigma_ref(qua_diff,qua_sigma):
    # print("------------------------")
    matrix_sigma=R.from_quat(qua_sigma).as_matrix()
    # print(matrix_sigma)
    matrix_diff=R.from_quat(qua_diff).as_matrix()
    
    # print(matrix_diff)
    matrix_ori=matrix_diff.T@matrix_sigma
    o_ref=R.from_matrix(matrix_ori).as_quat()
    # print(matrix_ori)
    return o_ref
    


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
    global mode_change_flag, rate_control_force, rad_x, rad_y, rad_z, goal_pub, data_record, global_cart, cartesian_init_pose
    rospy.init_node('iiwa_control')

    rospy.Subscriber('/iiwa/state/CartesianPose_end', PoseStamped,
                     callback=cartesian_pose_callback, queue_size=1)
    # rospy.Subscriber('/iiwa/state/CartesianPose_2', CartesianPose,
    #                  callback=cartesian_pose_callback, queue_size=1)
    # rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, callback=cartesian_pose_callback_2, queue_size=10)
    rospy.Subscriber('/sigma7/sigma0/pose', PoseStamped,
                     callback=pose_callback, queue_size=1)
    rospy.Subscriber('/sigma7/sigma0/buttons', Joy,
                     callback=buttons_callback, queue_size=1)
    rospy.Subscriber('/trajectory', Path, callback=path_callback, queue_size=1)
    rospy.Subscriber('/sigma7/sigma0/twist', TwistStamped,
                     callback=twist_callback, queue_size=1)
    # rospy.Subscriber('/pedal/buttons', Joy, callback=buttons_callback, queue_size=1)
    # rospy.spin()

    # joint_pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
    # cartesian_pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=1)
    cartesian_pub = rospy.Publisher(
        '/iiwa/command/CartesianPose_origin', PoseStamped, queue_size=1)
    force_pub = rospy.Publisher(
        '/sigma/force_feedback', WrenchStamped, queue_size=1)
    goal_pub = rospy.Publisher('goal', Float32MultiArray, queue_size=1)

    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        if cartesian_pose != 0:
            cartesian_init_pose = cartesian_pose
            # print(f'pose:{cartesian_pose}')
            break
        # if sigma_flag and joint_flag:
        #     data_flag = True
        #     break
    rate.sleep()
    while not rospy.is_shutdown():
        # if data_flag:
        #     pose_x = cartesian_pose.position.x + diff_x
        #     pose_y = cartesian_pose.position.y + diff_y
        #     pose_z = cartesian_pose.position.z + diff_z
        #     data_flag = False
        # print(f'error is {cartesian_pose_2.poseStamped.pose.position.z-cartesian_pose.poseStamped.pose.position.z}')

        # print('mode_change_flag:',mode_change_flag)

        if joint_flag and buttons_flag and not mode_change_flag:
            # start=rospy.get_time()
            # print(cartesian_pose)
            # print('-----------------------------')

            iiwa_x = cartesian_init_pose.pose.position.x
            iiwa_y = cartesian_init_pose.pose.position.y
            iiwa_z = cartesian_init_pose.pose.position.z

            iiwa_ox = cartesian_init_pose.pose.orientation.x
            iiwa_oy = cartesian_init_pose.pose.orientation.y
            iiwa_oz = cartesian_init_pose.pose.orientation.z
            iiwa_ow = cartesian_init_pose.pose.orientation.w
            # print(f'pose:{cartesian_init_pose}')

            iiwa_q = R.from_quat([iiwa_ox, iiwa_oy, iiwa_oz, iiwa_ow])

            iiwa_r = iiwa_q.as_matrix()

            iiwa_p = [iiwa_x,iiwa_y,iiwa_z]


            iiwa_r_new = sigma_r @ iiwa_r  

            iiwa_p_diff = [-diff_x,-diff_y,diff_z]

            iiwa_p_new = [iiwa_p[0]+iiwa_p_diff[0],iiwa_p[1]+iiwa_p_diff[1],iiwa_p[2]+iiwa_p_diff[2]]

            iiwa_q_new = R.from_matrix(iiwa_r_new).as_quat()

            cartesian_msg = PoseStamped()
            cartesian_msg.pose.position.x = iiwa_p_new[0]
            cartesian_msg.pose.position.y = iiwa_p_new[1]
            cartesian_msg.pose.position.z = iiwa_p_new[2]
            # q1 = Quaternion(cartesian_init_pose.orientation.w,cartesian_init_pose.orientation.x,cartesian_init_pose.orientation.y,cartesian_init_pose.orientation.z)
            #
            # q_new = q0*q1
            cartesian_msg.pose.orientation.x = iiwa_q_new[0]
            cartesian_msg.pose.orientation.y = iiwa_q_new[1]
            cartesian_msg.pose.orientation.z = iiwa_q_new[2]
            cartesian_msg.pose.orientation.w = iiwa_q_new[3]

            cartesian_msg.header.frame_id = 'iiwa_link_0'
            cartesian_msg.header.stamp = rospy.Time.now()
            global_cart = cartesian_msg

            # print(cartesian_msg)

            cartesian_pub.publish(cartesian_msg)

            # force_pub.publish(rate_control_force)
            force_pub.publish(training_force)
            data_record.record_x_s(cartesian_msg)
            data_record.record_x_m(sigma_pose)
            data_record.record_x_s_abs(global_cart)

            # end=rospy.get_time()
            # print("duration:",(end-start)*1000)
            # print(rate_control_force)

            # print(f'pose:{cartesian_msg}')

        else:
            cartesian_init_pose = cartesian_pose

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
            # print("----------------road 2----------------")
            # force_pub.publish(training_force)

            # print(f'while:{position_ref_x}')

            # print('----------------------------------------')
            # print('position_ref_x\n',position_ref_x,'\n','position_ref_y\n',position_ref_y,'\n','position_ref_z\n',position_ref_y)

        # print('feed back force: ',rate_control_force)
        rate.sleep()

    return


if __name__ == '__main__':
    main()
