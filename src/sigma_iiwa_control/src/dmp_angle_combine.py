
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

from target_prediction import TagetPredict

class DMPInter():
    paths_dd=[]
    paths_box = []
    paths_size = []
    path_tree=[]
    sub_paths_box=[]
    goal_position = np.array([[0.732, 0.04],[0.742,-0.01],[0.692,-0.04],[0.662,-0.06]])
    
    mapping_points = np.zeros((goal_position.shape[0], 2), dtype=np.float)
    mapping_points_2 = np.zeros((goal_position.shape[0], 2), dtype=np.float)#存入第二近的点
    
    min_indexs = np.zeros((goal_position.shape[0], 1), dtype=np.int)
    min_indexs_2 = np.zeros((goal_position.shape[0], 1), dtype=np.int)#存入第二近的点的索引
    
    min_distances = np.zeros((goal_position.shape[0], 1), dtype=np.float)
    min_distances_2=np.zeros((goal_position.shape[0], 1), dtype=np.float)#存入第二近的距离
    min_distances_before = np.zeros((goal_position.shape[0], 1), dtype=np.float)
    
    goals_angle = [ [30, 0],[-45,-30],[-60,45],[-40,0] ]
    goals_euler = np.zeros((len(goals_angle), 3))
    start_angle=[0,0]
    start_euler=np.zeros((1, 3))
    
    
    position_now=[0,0]
    
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
        
        print(self.start_euler)

        return goals_euler
    
    
    
    # def gen_inter_step(self):
        
    
    
    
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
    
    #才用shapely计算距离,各个轨迹的最近点以及第二近的点
    def get_distance(self,goal):
        
        self.min_distances_before[goal]=self.min_distances[goal]
        # print("position_now",self.position_now)
        dis,ind=self.path_tree[goal].query(self.position_now)
        dis_2,ind_2=self.path_tree[goal].query(self.position_now,k=2)
        self.min_distances[goal]=dis
        self.min_distances_2[goal]=dis_2[1]
        
        self.min_indexs[goal]=ind
        self.min_indexs_2[goal]=ind_2[1]
        
        self.mapping_points[goal]=self.paths_dd[goal][ind]
        self.mapping_points_2[goal]=self.paths_dd[goal][ind_2[1]]
        return
    
    # check当前点是否在两条轨迹之间
    def check_between(self,goal1,goal2):
        v1=self.position_now-self.mapping_points[goal1]
        v2=self.position_now-self.mapping_points[goal2]
        if np.dot(v1,v2)<0:
            return True
        else:
            return False
    
    #找到夹住当前位置的两条轨迹目标
    def get_between_goal(self):
        min_goal1=np.argmin(self.min_distances)
        m_min =np.inf
        min_goal2=-1
        m_min_goal2=-1
        goal_box= np.arange(self.goal_position.shape[0])
        stop_flag=False
        empty_flag=False
        goal_box=np.delete(goal_box,min_goal1)
        while not stop_flag:
            for i in goal_box:
                if m_min>self.min_distances[i]:
                    m_min=self.min_distances[i] 
                    m_min_goal2=i
            if self.check_between(min_goal1,m_min_goal2):
                min_goal2=m_min_goal2
                stop_flag=True
            else:
                mask = goal_box!= m_min_goal2
                goal_box=goal_box[mask]
                m_min =np.inf
            if goal_box.shape[0]==0:
                stop_flag=True
                empty_flag=True
        if empty_flag:
            min_goal2=min_goal1
        else:
            min_goal2=m_min_goal2
        return min_goal1,min_goal2
    
    #根据找到的目标找到四个差值点
    def get_four_interp(self):
        self.ip_point1=np.zeros((3,2))#第一行只有目标，后两行第一个是索引，第二个是距离
        self.ip_point2=np.zeros((3,2))
        min_goal1,min_goal2=self.get_between_goal()
        
        self.ip_point1[0][0]=min_goal1
        self.ip_point1[1][0]=self.min_indexs[min_goal1]
        self.ip_point1[1][1]=self.min_distances[min_goal1]
        self.ip_point1[2][0]=self.min_indexs_2[min_goal1]
        self.ip_point1[2][1]=self.min_distances_2[min_goal1]
        
        self.ip_point2[0][0]=min_goal2
        self.ip_point2[1][0]=self.min_indexs[min_goal2]
        self.ip_point2[1][1]=self.min_distances[min_goal2]
        self.ip_point2[2][0]=self.min_indexs_2[min_goal2]
        self.ip_point2[2][1]=self.min_distances_2[min_goal2]
    
    #用距离插值旋转矩阵
    def distance_interpo(self):
        d11=self.ip_point1[1,1]
        d12=self.ip_point1[2,1]
        
        l11=d12/(d11+d12)
        l12=d11/(d11+d12)
        
        euler11=self.paths_box[int(self.ip_point1[0][0])][int(self.ip_point1[1][0]),3:6]
        euler12=self.paths_box[int(self.ip_point1[0][0])][int(self.ip_point1[2][0]),3:6]
        
        qua1=self.qua_inter(l11,l12,euler11,euler12)
        
        d21=self.ip_point2[1,1]
        d22=self.ip_point2[2,1]
        
        l21=d22/(d21+d22)
        l22=d21/(d21+d22)
        
        euler21=self.paths_box[int(self.ip_point2[0][0])][int(self.ip_point2[1][0]),3:6]
        euler22=self.paths_box[int(self.ip_point2[0][0])][int(self.ip_point2[2][0]),3:6]
        
        qua2=self.qua_inter(l21,l22,euler21,euler22)
        
        D1=d11+d12
        D2=d21+d22
        
        L1=D2/(D1+D2)
        L2=D1/(D1+D2)
        
        # print("l1,l2:",l11,l12)
        # print("l1,l2:",l21,l22)
        # print("l1,l2:",L1,L2)
        
        QUA=self.qua_inter(L1,L2,qua1,qua2)
        self.QUA=QUA
        print("QUA: ",QUA)
        return QUA
        
        
    
    # def qua_inter(self,l1,l2,euler1,euler2):
        
    #     quat1 = R.from_euler('xyz', euler1).as_quat()
    #     quat2 = R.from_euler('xyz', euler2).as_quat()

    #     # 创建 Slerp 对象
    #     # interp_slerp = Slerp([0, 1], [quat1, quat2])

    #     # interpolated_quats = interp_slerp(l1)
        
    #     interpolated_quats=(l1*quat1+l2*quat2)/np.linalg.norm(l1*quat1+l2*quat2)

    #     #   将插值后的四元数转换为欧拉角
    #     return R.from_quat(interpolated_quats).as_euler('xyz')
    #####
    def qua_inter(self,l1,l2,euler1,euler2):
    # 将旋转姿态转换为旋转矩阵
        matrix1 = R.from_euler('zyx', euler1).as_matrix()
        matrix2 = R.from_euler('zyx', euler2).as_matrix()
        

        # 计算从rotation1到rotation2的旋转矩阵
        rotation_matrix = np.dot(matrix2, matrix1.T)

        # 计算旋转角度和旋转轴
        rotation = R.from_matrix(rotation_matrix)
        rotvec = rotation.as_rotvec()
        
        print("rotvec",np.linalg.norm(rotvec))
        
        if np.linalg.norm(rotvec)>math.pi:
            print("------------------------------------------")
            rotvec = -rotvec * (2*math.pi / np.linalg.norm(rotvec))

        # 根据给定的系数进行旋转
        rotated_rotvec = l2/(l1+l2) * rotvec
        rotated_rotation=  R.from_rotvec(rotated_rotvec)

        # 获取旋转后的旋转矩阵
        rotated_matrix = np.dot(rotated_rotation.as_matrix(), matrix1)
        
        return R.from_matrix(rotated_matrix).as_euler('zyx')
    ######
    # def qua_inter(self,l1,l2,euler1,euler2):
    # # 将旋转姿态转换为旋转矩阵
    #     matrix1 = R.from_euler('xyz', euler1).as_matrix()
    #     matrix2 = R.from_euler('xyz', euler2).as_matrix()
        

    #     interpolated_matrix = l1* matrix1 + l2 * matrix2

    #     # 将插值后的旋转矩阵转换为姿态
    #     interpolated_pose = R.from_matrix(interpolated_matrix)
        
        
        
    #     return interpolated_pose .as_euler('xyz')
    
    
    def path_gen(self,point1, point2, num_points):
        # 计算每个维度的步长
        step_x = (point2[0] - point1[0]) / (num_points + 1)
        step_y = (point2[1] - point1[1]) / (num_points + 1)

        # 插值生成点的数组
        interpolated_points = []
        for i in range(1, num_points + 1):
            interpolated_x = point1[0] + i * step_x
            interpolated_y = point1[1] + i * step_y
            interpolated_points.append([interpolated_x, interpolated_y])
        return interpolated_points
    
    def square_gen(self,point1, point2, num_points):
        # 计算每个维度的步长
        step_x = (point2[0] - point1[0]) / (num_points + 1)
        step_y = (point2[1] - point1[1]) / (num_points + 1)

        # 插值生成点的数组
        interpolated_points = []
        for i in range(1, num_points + 1):
            interpolated_x = point1[0] + i * step_x
            interpolated_y = point1[1] + i * step_y
            interpolated_points.append([interpolated_x, interpolated_y])
        return interpolated_points
    
    def next_pose(self,time_step,qua_now,qua_goal,velocity,goal):
        goal_vector=self.goal_position[goal]-self.position_now
        
        v_vector=np.dot(velocity,goal_vector)/np.linalg.norm(goal_vector)
        
        # matrix1 = R.from_quat(qua_now).as_matrix()
        # matrix2 = R.from_quat(qua_goal).as_matrix()
        
        matrix1 = R.from_euler('zyx', qua_now).as_matrix()
        matrix2 = R.from_euler('zyx', qua_goal).as_matrix()
        
        # 计算从rotation1到rotation2的旋转矩阵
        rotation_matrix = np.dot(matrix2, matrix1.T)

        # 计算旋转角度和旋转轴
        rotation = R.from_matrix(rotation_matrix)
        rotvec = rotation.as_rotvec()
        
        # print("rotvec",np.linalg.norm(rotvec))
        
        if np.linalg.norm(rotvec)>math.pi:
            print("------------------------------------------")
            rotvec = -rotvec * (2*math.pi / np.linalg.norm(rotvec))

        k=(1.0-np.linalg.norm(v_vector)*time_step/np.linalg.norm(goal_vector))*0.04
        
        # k=np.linalg.norm(v_vector)*time_step/np.linalg.norm(goal_vector)*2
        # k=0.09
        print(k)
        
        # 根据给定的系数进行旋转
        rotated_rotvec = k * rotvec
        rotated_rotation=  R.from_rotvec(rotated_rotvec)

        # 获取旋转后的旋转矩阵
        rotated_matrix = np.dot(rotated_rotation.as_matrix(), matrix1)
        
        return R.from_matrix(rotated_matrix).as_euler('zyx')
        
        

    
    
    def plot_coordinates(self,array,path):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        position=[]
        euler_angles=[]
        # 提取位置和欧拉角数据
        for i in range(len(array)):
            position.append(array[i][:, :3])
            euler_angles.append(array[i][:, 3:])
        # print(euler_angles[0])
        

        # 绘制每个点的位置和坐标系
        for j in range(len(position)):
            for i in range(len(position[j])):
                x, y, z = position[j][i]
                roll, pitch, yaw = euler_angles[j][i]

                # 绘制点的位置
                ax.scatter(x, y, z, color='b')

                rotation_matrix = R.from_euler('zyx', [roll, pitch, yaw], degrees=False).as_matrix()
                # if j==0:
                    # print(rotation_matrix)
                # if j==0:
                #     print(rotation_matrix[0][0],rotation_matrix[1][0],rotation_matrix[2][0])
                # 绘制坐标系
                v_x=np.array([rotation_matrix[0][0],rotation_matrix[1][0],rotation_matrix[2][0]])
                v_y=np.array([rotation_matrix[0][1],rotation_matrix[1][1],rotation_matrix[2][1]])
                v_z=np.array([rotation_matrix[0][2],rotation_matrix[1][2],rotation_matrix[2][2]])
                
                v_x=v_x/np.linalg.norm(v_x)
                v_y=v_y/np.linalg.norm(v_y)
                v_z=v_z/np.linalg.norm(v_z)
                # if j==1:
                    # print("v_x:    ",v_x)
                # ax.quiver(x, y, z, v_x[0],v_x[1],v_x[2],length=0.01, color='r')
                # ax.quiver(x, y, z, v_y[0],v_y[1],v_y[2],length=0.01, color='g')
                # ax.quiver(x, y, z, v_z[0],v_z[1],v_z[2],length=0.01, color='b')
        
        for m in range(len(self.goal_position)):
            x,y=self.goal_position[m]
            z=0.09
            
            rotation_matrix = R.from_euler('zyx', self.goals_euler[m], degrees=False).as_matrix()
            v_x=np.array([rotation_matrix[0][0],rotation_matrix[1][0],rotation_matrix[2][0]])
            v_y=np.array([rotation_matrix[0][1],rotation_matrix[1][1],rotation_matrix[2][1]])
            v_z=np.array([rotation_matrix[0][2],rotation_matrix[1][2],rotation_matrix[2][2]])
            
            ax.quiver(x, y, z, v_x[0],v_x[1],v_x[2],length=0.01, color='r')
            ax.quiver(x, y, z, v_y[0],v_y[1],v_y[2],length=0.01, color='g')
            ax.quiver(x, y, z, v_z[0],v_z[1],v_z[2],length=0.01, color='b')
            
            
        
        for k in range(len(path)):
            ax.scatter(path[k][0], path[k][1], path[k][2], color='k')
            rotation_now = R.from_euler('zyx', [path[k][3], path[k][4],path[k][5]], degrees=False).as_matrix()  
            v_x_now=np.array([rotation_now [0][0],rotation_now[1][0],rotation_now [2][0]])
            ax.quiver(path[k][0], path[k][1],path[k][2], v_x_now[0],v_x_now[1],v_x_now[2],length=0.01, color='r')   

        # 设置图形属性
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim([0.65, 0.75])
        ax.set_ylim([-0.05, 0.05])
        ax.set_zlim([0.05, 0.15])
        ax.set_title('Coordinates')
        # plt.gca().set_box_aspect((1,1,1))
        # ax.auto_scale_xyz(positions[:, 0], positions[:, 1], positions[:, 2])

        # 显示图形
        plt.show()
        
def main():
    m_DMP=DMPInter()
    rospy.init_node('my_node')
    predict=TagetPredict()
    m_DMP.load_standard_path()
    # input_array = np.array([[1, 2, 3, 0.1, 0.2, 0.3],
    #                    [4, 5, 6, 0.4, 0.5, 0.6],
    #                    [7, 8, 9, 0.7, 0.8, 0.9]])
    
    m_DMP.goals_euler=m_DMP.from_angle_to_euler()
    
    # start=m_DMP.goal_position[0]
    start=[0.643,0.058]
    
    endd=m_DMP.goal_position[1]
    status_now=[]
    
    position_nows=m_DMP.path_gen(start,endd,100)
    
    velocity=[0,0]
    
    position_before=position_nows[0]
    time_step=0.001
    
    pose_start=[m_DMP.start_euler[0][0],m_DMP.start_euler[0][1],m_DMP.start_euler[0][2]]

    for position in position_nows:
        # print("position",position)
        m_DMP.position_now=position
        
        velocity=([(position[0]-position_before[0])/time_step,(position[1]-position_before[1])/time_step])
        
        # for i in range(len(m_DMP.goal_position)):
        #     m_DMP.get_distance(i)
        predict.get_local(position,velocity)
        predict.get_prob()
        goal_predict=predict.predict_goal
        
        print(goal_predict)
        
        # m_DMP.get_four_interp()
        # qua=m_DMP.distance_interpo()
        
        qua=m_DMP.next_pose(time_step,pose_start,m_DMP.goals_euler[goal_predict],velocity,goal_predict)
        
        status_now.append([position[0],position[1],0.09,qua[0],qua[1],qua[2]])
        
        pose_start=qua
        position_before=position
            
        # print("min_dis: ",m_DMP.min_distances)
        # print("min_dis2: ",m_DMP.min_distances_2)
        
        
        # print("ip1:  ",m_DMP.ip_point1)
        # print("ip2:  ",m_DMP.ip_point2)
        rospy.sleep(0.001)

    print(qua)
        
    # print(status_now)
    print("euler_goal: ",m_DMP.goals_euler[2])
    # print("real_goal: ",m_DMP.paths_box[3][-1][3:6])
    
    m_DMP.plot_coordinates(m_DMP.sub_paths_box,status_now)
    return
    


if __name__ == '__main__':
    main()
    
    
    