import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as ani
import matplotlib.lines as lnes
import matplotlib.colors as mcolors
import math
import os
from math import sqrt, pow, acos
from scipy import stats
from matplotlib import style
import rosbag
from iiwa_msgs.msg import CartesianPose
from scipy.spatial import cKDTree
from threading import Thread

from scipy.spatial.transform.rotation import Rotation as R
from scipy.spatial.transform.rotation import Slerp
from geometry_msgs.msg import PoseStamped
from detection import Detection




class PoseAdaptive():
    paths_dd=[]
    paths_box = []
    paths_size = []
    path_tree=[]
    sub_paths_box=[]
    detect=Detection()
    cartesian_init_pose=PoseStamped()
    goal_position = np.array([[0.732, 0.04],[0.742,-0.01],[0.692,-0.04],[0.662,-0.06],[0.642, 0.06]],)
    
    
    
    mapping_points = np.zeros((goal_position.shape[0], 2), dtype=np.float)
    mapping_points_2 = np.zeros((goal_position.shape[0], 2), dtype=np.float)#存入第二近的点
    
    min_indexs = np.zeros((goal_position.shape[0], 1), dtype=np.int)
    min_indexs_2 = np.zeros((goal_position.shape[0], 1), dtype=np.int)#存入第二近的点的索引
    
    min_distances = np.zeros((goal_position.shape[0], 1), dtype=np.float)
    min_distances_2=np.zeros((goal_position.shape[0], 1), dtype=np.float)#存入第二近的距离
    min_distances_before = np.zeros((goal_position.shape[0], 1), dtype=np.float)
    
    goals_angle = [ [30, 0],[-45,-30],[-60,45],[-40,0],[0,0] ]
    goals_euler = np.zeros((len(goals_angle), 3))
    start_angle=[0,0]
    start_euler=np.zeros((1, 3))
    
    is_back=False
    
    
    position_now=[0,0]
    
    def __init__(self):
        self.load_standard_path()
        self.goals_euler=self.from_angle_to_euler()
        
    def get_local(self, local_position):
        self.local_point = local_position
        self.detect.get_x_d(local_position)
        self.position_now=[local_position[0],local_position[1]]
        
        # 判断是否在返回的路径
        self.is_back=self.detect.judge_is_back()
    
    def apply_transform(self,angleX, angleZ):
        # 构造旋转矩阵
        rotationY = R.from_euler('y', 90, degrees=True)
        rotationX = R.from_euler('x', angleX, degrees=True)
        rotationZ = R.from_euler('z', angleZ, degrees=True)

        transform = rotationZ * rotationX * rotationY
        
        print(transform.as_matrix())

        euler = transform.as_euler('zyx', degrees=False)
        return euler

    def from_angle_to_euler(self):
        goals_euler = np.zeros((len(self.goals_angle), 3))
        
        for i in range(len(self.goals_angle)):
            data = self.apply_transform(self.goals_angle[i][0], self.goals_angle[i][1])
            goals_euler[i][0] = data[0]
            goals_euler[i][1] = data[1]
            goals_euler[i][2] = data[2]
            
        data0 = self.apply_transform(self.start_angle[0], self.start_angle[1])
        self.start_euler[0][0] = data0[0]
        self.start_euler[0][1] = data0[1]
        self.start_euler[0][2] = data0[2]
        
        # print(self.start_euler)

        return goals_euler
    
    #载入轨迹
    def load_standard_path(self):
            i = 0
            m_str = '/home/chy/sigma_iiwa_simulation/ur_sigma/path/standard_path'+str(i)+'.txt'
            while os.path.exists(m_str):
                a = np.loadtxt(m_str)
                self.paths_dd.append(a[:,0:2])#存二维
                self.paths_box.append(a[:,:])
                
                self.sub_paths_box.append(a[::100])
                
                self.path_tree.append(cKDTree(a[:,0:2]))
                self.paths_size.append(len(a))
                i+=1
                m_str = '/home/chy/sigma_iiwa_simulation/ur_sigma/path/standard_path'+str(i)+'.txt'
            print("-------------load path success! we get %d pathes------------"%(i))
            print('sizes: ',self.paths_size)
            
            return
    
    
    def load_local_save_pose(self,cartesian_init):
        self.cartesian_init_pose=cartesian_init
        
    
    def next_pose(self,time_step,qua_now,velocity,goal):
        
        if self.is_back:
            goal=4# 4 表示目标为起始点
            
        radius=0.01

        goal_vector=self.goal_position[goal]-self.position_now
        if np.linalg.norm(goal_vector)<radius:
            goal_vector_plus=goal_vector-goal_vector/np.linalg.norm(goal_vector)*radius
        else:
            goal_vector_plus=goal_vector
        velocity_dim2=velocity[0:2]
        # print(velocity)
        v_vector=np.dot(velocity_dim2,goal_vector_plus)/np.linalg.norm(goal_vector_plus)
        
        # matrix1 = R.from_quat(qua_now).as_matrix()
        # matrix2 = R.from_quat(qua_goal).as_matrix()
        
        matrix1 = R.from_euler('zyx', qua_now).as_matrix()
        matrix2 = R.from_euler('zyx', self.goals_euler[goal]).as_matrix()
        
        # 计算从rotation1到rotation2的旋转矩阵
        rotation_matrix = np.dot(matrix2, matrix1.T)

        # 计算旋转角度和旋转轴
        rotation = R.from_matrix(rotation_matrix)
        rotvec = rotation.as_rotvec()
        
        # print("rotvec",np.linalg.norm(rotvec))
        
        if np.linalg.norm(rotvec)>math.pi:
            print("------------------------------------------")
            rotvec = -rotvec * (2*math.pi / np.linalg.norm(rotvec))

        k=np.linalg.norm(v_vector)*time_step*10/np.linalg.norm(goal_vector_plus)
        
        # print(k)
        
        # 根据给定的系数进行旋转
        rotated_rotvec = k * rotvec
        rotated_rotation=  R.from_rotvec(rotated_rotvec)
        # 获取旋转后的旋转矩阵
        rotated_matrix = np.dot(rotated_rotation.as_matrix(), matrix1)
        # if goal!=4:
            # print("goal quat: ",R.from_matrix(matrix2).as_quat())
        
        rot_init=R.from_quat([self.cartesian_init_pose.pose.orientation.x,
                              self.cartesian_init_pose.pose.orientation.y,
                              self.cartesian_init_pose.pose.orientation.z,
                              self.cartesian_init_pose.pose.orientation.w]).as_matrix()

        rot_change=np.dot(rotated_matrix,rot_init.T)
        
        # print(goal)

        return R.from_matrix(rot_change).as_quat()
    
    
    
    
    