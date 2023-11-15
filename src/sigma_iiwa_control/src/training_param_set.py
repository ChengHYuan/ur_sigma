import rospy
import rosbag
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
import numpy as np
import math
import os
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from detection import Detection


class Training(object):
    # 定义辅助训练时的相关参数
    curvature = 0.0  # 曲率
    training_proviciency = 1.0  # 熟练度

    goal_position = np.array([[0.732, 0.04, 0.02], [
                             0.742, -0.01, 0.02], [0.692, -0.04, 0.02], [0.662, -0.06, 0.02]])
    # 起始位置
    start_point = np.array([0.642, 0.06, 0.02])

    k_gain_base = 600.0  # 轨迹基础刚度
    d_gain_base = 400.0
    m_gain_base = 20.0

    Force = np.array([0.0, 0.0, 0.0])
    Velocity = np.array([0.0, 0.0, 0.0])
    Velocity_before = np.array([0.0, 0.0, 0.0])
    Accel = np.array([0.0, 0.0, 0.0])

    # 存储所有的标准轨迹
    paths_box = []
    paths_size = []
    line_geom = []
    #kd树
    trees=[]
    
    #抵抗力，用辅助力的矢量和计算
    anti_force=np.array([0.0,0.0])

    # 记录目标已经改变的flag
    change_flag = False
    
    #辅助力平滑变化参数
    force_rate=0.0
    #专门记录力施加时间的参数，防止突变
    force_step=0

    center = np.array([0.0, 0.0, 0.0])
    local_point = np.array([0.0, 0.0, 0.0])
    mapping_point = np.array([0.0, 0.0, 0.0])  # 最近点
    mapping_vector = np.array([0.0, 0.0, 0.0])  # 最近点切线方向
    mapping_accel = np.array([0.0, 0.0, 0.0])  # 最近点加速度
    min_index = -1

    goal = -1

    start_point = np.array([0.0, 0.0, 0.0])

    points_window = np.zeros((10, 3), dtype=float)
    
    is_back=False
    
    detection=Detection()

    # 初始化函数
    def __init__(self):
        self.load_standard_path()
        
        workspace = os.getcwd()
        foldername="data/operator/ff2/f2data"
        folderpath = os.path.join(workspace, foldername)
        try:
            os.mkdir(folderpath)
        except FileExistsError:
            pass
        
        filename1="force_m.txt"
        filename2="force_k.txt"
        filename3="force_d.txt"
        filename4="force.txt"
        filename5="velocity_s.txt"
        
        filepath1 = os.path.join(folderpath, filename1)
        filepath2 = os.path.join(folderpath, filename2)
        filepath3 = os.path.join(folderpath, filename3)
        filepath4 = os.path.join(folderpath, filename4)
        filepath5 = os.path.join(folderpath, filename5)
        
        
        self.file1=open(filepath1,"w")
        self.file2=open(filepath2,"w")
        self.file3=open(filepath3,"w")
        self.file4=open(filepath4,"w")
        self.file5=open(filepath5,"w")
        pass

    def __del__(self):
        self.file1.close()
        self.file2.close()
        self.file3.close()
        self.file4.close()
        self.file5.close()
        return
    
    # 从轨迹库中获取标准轨迹
    def load_standard_path(self):
        i = 0
        m_str = '/home/chy/sigma_iiwa_simulation/sigma_to_sim_workspace/path/standard_path' + \
            str(i)+'.txt'
        while os.path.exists(m_str):
            a = np.loadtxt(m_str)
            self.paths_box.append(a)
            self.paths_size.append(len(a))
            self.trees.append(cKDTree(a))
            i += 1
            m_str = '/home/chy/sigma_iiwa_simulation/sigma_to_sim_workspace/path/standard_path' + \
                str(i)+'.txt'
        print("-------------load path success! we get %d pathes------------" % (i))
        print('sizes: ', self.paths_size)
        
        return

    #清空关键数据
    def fresh_para(self):
        # self.Force.wrench.force.x = 0
        # self.Force.wrench.force.y = 0
        # self.Force.wrench.force.z = 0
        self.Velocity = np.array([0.0, 0.0, 0.0])
        self.Velocity_before = np.array([0.0, 0.0, 0.0])
        self.force_step=0
        self.force_rate=0.0
        self.Force=np.array([0.0, 0.0, 0.0])

    # 获取目标和最短点索引

    def get_goal(self, goal, button_flag):
        
        # 返回途中，目标不变
        if self.is_back and self.goal!=-1:
            goal=self.goal
            
        # 预测的目标与当前目标不一致时 记录flag
        
        if self.goal != goal:
            self.change_flag = True
        else:
            self.change_flag = False
        self.goal = goal

        if not button_flag:
            self.fresh_para()

        return


    # 计算目标的可信度
    def get_goal_trust(self, goal_trust):
        self.goal_trust = goal_trust
        if self.is_back:
            self.goal_trust=0.8

    # 获取当前位置、速度
    def get_local(self, local_position, local_velocity,local_accel):
        self.local_point = local_position
        self.detection.get_x_d(local_position)
        
        # 判断是否在返回的路径
        self.is_back=self.detection.judge_is_back()
        
        self.Velocity_before = self.Velocity
        self.Velocity = local_velocity*125  # 主端增量速度
        # if self.Velocity_before[0] == 0.0 and self.Velocity_before[1] == 0.0 and self.Velocity_before[2] == 0.0:
        #     self.Velocity_before = self.Velocity
        # self.Accel = (self.Velocity-self.Velocity_before)*150
        self.Accel=local_accel
        

    # 根据轨迹获取曲率 最小二乘拟合
    def sphere_surface(self):
        # 获取窗口点
        self.get_points_window()
        num_points = self.points_window.shape[0]
        # print(num_points)
        x, y, z = self.points_window[:,
                                     0], self.points_window[:, 1], self.points_window[:, 2]

        x_avr, y_avr, z_avr = sum(x)/num_points, sum(y) / \
            num_points, sum(z)/num_points
        xx_avr, yy_avr, zz_avr = sum(
            x*x)/num_points, sum(y*y)/num_points, sum(z*z)/num_points
        xy_avr, xz_avr, yz_avr = sum(
            x*y)/num_points, sum(x*z)/num_points, sum(y*z)/num_points

        xxx_avr = sum(x * x * x) / num_points
        xxy_avr = sum(x * x * y) / num_points
        xxz_avr = sum(x * x * z) / num_points
        xyy_avr = sum(x * y * y) / num_points
        xzz_avr = sum(x * z * z) / num_points
        yyy_avr = sum(y * y * y) / num_points
        yyz_avr = sum(y * y * z) / num_points
        yzz_avr = sum(y * z * z) / num_points
        zzz_avr = sum(z * z * z) / num_points

        A = np.array([[xx_avr - x_avr * x_avr, xy_avr - x_avr * y_avr, xz_avr - x_avr * z_avr],
                      [xy_avr - x_avr * y_avr, yy_avr -
                          y_avr * y_avr, yz_avr - y_avr * z_avr],
                      [xz_avr - x_avr * z_avr, yz_avr - y_avr * z_avr, zz_avr - z_avr * z_avr]])
        b = np.array([xxx_avr - x_avr * xx_avr + xyy_avr - x_avr * yy_avr + xzz_avr - x_avr * zz_avr,
                      xxy_avr - y_avr * xx_avr + yyy_avr - y_avr * yy_avr + yzz_avr - y_avr * zz_avr,
                      xxz_avr - z_avr * xx_avr + yyz_avr - z_avr * yy_avr + zzz_avr - z_avr * zz_avr])
        b = b / 2
        center = np.linalg.solve(A, b)
        x0, y0, z0 = center[0], center[1], center[2]
        r2 = xx_avr - 2 * x0 * x_avr + x0 * x0 + yy_avr - 2 * \
            y0 * y_avr + y0 * y0 + zz_avr - 2 * z0 * z_avr + z0 * z0
        r = r2 ** 0.5
        self.center = center
        self.curvature = 1/r
        return

    def get_curvature(self):
        self.get_points_window()
        x=self.points_window[:,0]
        y=self.points_window[:,1]
        z=self.points_window[:,2]
        dx_dt = np.gradient(x)
        dy_dt = np.gradient(y)
        dz_dt = np.gradient(z)
        d2x_dt2 = np.gradient(dx_dt)
        d2y_dt2 = np.gradient(dy_dt)
        d2z_dt2 = np.gradient(dz_dt)
        curvature = (d2x_dt2 * (dy_dt * d2z_dt2 - dz_dt * d2y_dt2) - d2y_dt2 * (dx_dt * d2z_dt2 - dz_dt * d2x_dt2) +
                     d2z_dt2 * (dx_dt * d2y_dt2 - dy_dt * d2x_dt2)) / ((dx_dt ** 2 + dy_dt ** 2 + dz_dt ** 2) ** 1.5)
        self.curvature=curvature[4]
        return

    # 根据映射点获取前后10个点
    def get_points_window(self):
        left = self.min_index-5 if self.min_index >= 5 else 0
        right = self.min_index + \
            5 if self.min_index <= self.paths_size[self.goal] - \
            5 else self.paths_size[self.goal]
        for i in range(left, right):
            self.points_window[i-left][0] = self.paths_box[self.goal][i][0]
            self.points_window[i-left][1] = self.paths_box[self.goal][i][1]
            self.points_window[i-left][2] = self.paths_box[self.goal][i][2]
        return

    # 初始化检索最近点
    def get_nearest_point_init(self):
        window = 0.1
        size = self.paths_size[self.goal]
        l1 = np.sqrt(np.sum((self.local_point-self.start_point)**2))
        l2 = np.sqrt(
            np.sum((self.local_point-self.goal_position[self.goal])**2))
        # print('path size is: ',size)
        k_position = l1/(l1+l2)
        k_up = k_position+window
        k_down = k_position-window
        if k_down < 0.0:
            k_down = 0.0
        if k_up > 1.0:
            k_up = 1.0
        min = np.inf
        position = np.array([0.0, 0.0, 0.0])
        for index in range(int(size*k_down), int(size*k_up)):  # 有限范围搜索
            position[0] = self.paths_box[self.goal][index][0]
            position[1] = self.paths_box[self.goal][index][1]
            position[2] = self.paths_box[self.goal][index][2]
            if np.sqrt(np.sum((position-self.local_point)**2)) < min:
                self.min_index = index
                min = np.sqrt(np.sum((position-self.local_point)**2))
        self.mapping_point[0] = self.paths_box[self.goal][self.min_index][0]
        self.mapping_point[1] = self.paths_box[self.goal][self.min_index][1]
        self.mapping_point[2] = self.paths_box[self.goal][self.min_index][2]
        if self.min_index > 0 and self.min_index < size-1:
            step = 1/size
            self.mapping_vector[0] = (self.paths_box[self.goal][self.min_index+1][0] -
                                      self.paths_box[self.goal][self.min_index-1][0])/(2*step)
            self.mapping_vector[1] = (self.paths_box[self.goal][self.min_index+1][1] -
                                      self.paths_box[self.goal][self.min_index-1][1])/(2*step)
            self.mapping_vector[2] = (self.paths_box[self.goal][self.min_index+1][2] -
                                      self.paths_box[self.goal][self.min_index-1][2])/(2*step)
            self.mapping_accel[0] = ((self.paths_box[self.goal][self.min_index+1][0] -
                                      self.paths_box[self.goal][self.min_index][0])/step-(self.paths_box[self.goal][self.min_index][0] -
                                                                                          self.paths_box[self.goal][self.min_index-1][0])/step)/step
            self.mapping_accel[1] = ((self.paths_box[self.goal][self.min_index+1][1] -
                                      self.paths_box[self.goal][self.min_index][1])/step-(self.paths_box[self.goal][self.min_index][1] -
                                                                                          self.paths_box[self.goal][self.min_index-1][1])/step)/step
            self.mapping_accel[2] = ((self.paths_box[self.goal][self.min_index+1][2] -
                                      self.paths_box[self.goal][self.min_index][2])/step-(self.paths_box[self.goal][self.min_index][2] -
                                                                                          self.paths_box[self.goal][self.min_index-1][2])/step)/step
        else:
            print('init index is out of range!!')
        return

    # 牛顿迭代检索最小值
    def get_nearest_point_it(self):
        size = self.paths_size[self.goal]
        s = float(self.min_index/size)
        s_1 = -1.0
        step = 0
        ind_s = self.min_index
        min_point = self.mapping_point
        min_vector = self.mapping_vector
        min_accel = self.mapping_accel
        local_point = self.local_point
        # print('min_vector:', self.mapping_vector)
        # print('min_point: ', self.mapping_point)
        delta_s = 2
        while np.abs(delta_s)*size >= 1:
            delta_s = np.sum((min_point-local_point)*min_vector) / \
                (np.sum(min_vector**2)+np.sum((min_point-local_point)*min_accel))
            s_1 = s-delta_s
            # print('delta_s: ',delta_s)

            ind_s = int(s_1*size)
            # print(delta_s*self.num)
            # print(self.path.poses[ind_s].pose.position.x,self.path.poses[ind_s+1].pose.position.x,self.path.poses[ind_s+2].pose.position.x)
            min_point[0] = self.paths_box[self.goal][ind_s][0]
            min_point[1] = self.paths_box[self.goal][ind_s][1]
            min_point[2] = self.paths_box[self.goal][ind_s][2]

            if ind_s > 0 and ind_s < size-1:
                step = 1/size
                # print('flag:',self.path.poses[ind_s+1].pose.position.x)
                # print('flag:',self.path.poses[ind_s-1].pose.position.x)
                min_vector[0] = (self.paths_box[self.goal][ind_s+1][0] -
                                 self.paths_box[self.goal][ind_s-1][0])/(2*step)
                min_vector[1] = (self.paths_box[self.goal][ind_s+1][1] -
                                 self.paths_box[self.goal][ind_s-1][1])/(2*step)
                min_vector[2] = (self.paths_box[self.goal][ind_s+1][2] -
                                 self.paths_box[self.goal][ind_s-1][2])/(2*step)
                min_accel[0] = ((self.paths_box[self.goal][ind_s+1][0] -
                                 self.paths_box[self.goal][ind_s][0])/step-(self.paths_box[self.goal][ind_s][0] -
                                                                            self.paths_box[self.goal][ind_s-1][0])/step)/step
                min_accel[1] = ((self.paths_box[self.goal][ind_s+1][1] -
                                 self.paths_box[self.goal][ind_s][1])/step-(self.paths_box[self.goal][ind_s][1] -
                                                                            self.paths_box[self.goal][ind_s-1][1])/step)/step
                min_accel[2] = ((self.paths_box[self.goal][ind_s+1][2] -
                                 self.paths_box[self.goal][ind_s][2])/step-(self.paths_box[self.goal][ind_s][2] -
                                                                            self.paths_box[self.goal][ind_s-1][2])/step)/step
                # print('flag:',min_vector)
            else:
                print('itr index is out of range!!')
            s = s_1
            step += 1
            if step > 100:
                print('itr time to long!!!')
                break
        self.mapping_point = min_point
        self.mapping_vector = min_vector
        self.mapping_accel = min_accel
        # print('min_point: ',min_point)
        print('itr is ok')
        self.min_index = ind_s
        return

    def get_path_velocity(self, index):
        step = 1.0/self.paths_size[self.goal]
        velocity = np.array([0.0, 0.0, 0.0])
        if index <= 0:
            index = 1
        elif index >= self.paths_size[self.goal]-1:
            index = self.paths_size[self.goal]-2
        else:
            pass
        velocity[0] = (self.paths_box[self.goal][index+1][0] -
                       self.paths_box[self.goal][index-1][0])/(2*step)
        velocity[1] = (self.paths_box[self.goal][index+1][1] -
                       self.paths_box[self.goal][index-1][1])/(2*step)
        velocity[2] = (self.paths_box[self.goal][index+1][2] -
                       self.paths_box[self.goal][index-1][2])/(2*step)
        return velocity

    def get_path_accel(self, index):
        step = 1.0/self.paths_size[self.goal]
        accel = np.array([0.0, 0.0, 0.0])
        if index <= 0:
            index = 1
        elif index >= self.paths_size[self.goal]-1:
            index = self.paths_size[self.goal]-2
        else:
            pass
        accel[0] = ((self.paths_box[self.goal][index+1][0] -
                     self.paths_box[self.goal][index][0])/step-(self.paths_box[self.goal][index][0] -
                                                                self.paths_box[self.goal][index-1][0])/step)/step
        accel[1] = ((self.paths_box[self.goal][index+1][1] -
                     self.paths_box[self.goal][index][1])/step-(self.paths_box[self.goal][index][1] -
                                                                self.paths_box[self.goal][index-1][1])/step)/step
        accel[2] = ((self.paths_box[self.goal][index+1][2] -
                     self.paths_box[self.goal][index][2])/step-(self.paths_box[self.goal][index][2] -
                                                                self.paths_box[self.goal][index-1][2])/step)/step
        return accel

    # 用kdTree计算最近距离，第二近距离，并返回索引
    def get_min_distance_kdtree(self):
        distance, indices = self.trees[self.goal].query(self.local_point)
        # distance_2, indices_2 = self.trees[self.goal].query(self.local_point)
        self.min_index = indices
        self.mapping_point = self.paths_box[self.goal][indices]
        self.mapping_vector = self.get_path_velocity(indices)
        self.mapping_accel = self.get_path_accel(indices)
        
        # print("-----velocity:",self.mapping_point)
        # print("-----accel:",self.mapping_accel)
        return

    # 根据某些参数获取熟练度因子
    def get_proficiency(self):

        return

    # 获取原始辅助力

    # 根据曲率、熟练度因子等改变刚度大小
    def get_k_d_gain(self):
        # 当映射点为空时，或者目标切换时，初始化映射点
        
        if (self.mapping_point[0] == 0.0 and self.mapping_point[1] == 0.0) or self.change_flag:
            self.get_min_distance_kdtree()
        else:
            # self.get_nearest_point_it()
            self.get_min_distance_kdtree()
        # self.sphere_surface()
        
        self.get_curvature()
        # print("-----curvature:",self.curvature)
        self.k_gain = (1+math.log(self.curvature+1, math.e))*self.k_gain_base
        self.d_gain = (1+math.log(self.curvature+1, math.e))*self.d_gain_base
        # print(self.k_gain)
        # print(self.d_gain)

        # print("-----------------")
        return

    #将力逐步施加
    def force_raise_rate(self):
        step=self.force_step
        base_step=20.0
        self.force_rate=1.0 / \
            (1+math.exp(-(step/base_step)+6))
        return
    
    # 获取力

    def get_force(self):
        self.get_k_d_gain()
        # 获取各个单位向量
        # e_s = self.Velocity/np.linalg.norm(self.Velocity) if np.sum((self.Velocity)**2)>0.0 else np.array([0.0, 0.0, 0.0])
        e_path = self.mapping_vector/np.linalg.norm(self.mapping_vector)
        # e_a=self.mapping_accel/np.sqrt(np.sum((self.mapping_accel)**2))

        force_k = self.k_gain*(self.mapping_point-self.local_point) if np.linalg.norm(self.Velocity)>0 else np.array([0.0, 0.0, 0.0])
        # print("----------",self.min_index)

        force_d_fake = self.d_gain * \
            (self.Velocity-np.dot(self.Velocity, e_path)*e_path)
        force_d=np.array([force_d_fake[0],force_d_fake[1],-force_d_fake[2]])
        

        force_m = self.Accel*self.m_gain_base if np.linalg.norm(self.Velocity)>0 else np.array([0.0, 0.0, 0.0])
        
        # print(force_k)
        # print(force_d)
        # print("-----------------")
        # print(self.Accel)
        force_origin = force_d+force_m+force_k
        # cos_f_s=np.dot(force_origin,self.mapping_vector)/(np.sqrt(np.sum(force_origin))*np.sqrt(np.sum(self.mapping_vector)))

        # force=(force_origin-np.dot(force_origin,e_s)*e_s)/(cos_f_s**2)*self.goal_trust
        # force=(force_origin-np.dot(force_origin,e_s)*e_s)*self.goal_trust
        force=force_origin
        self.anti_force=force[:2]
        
        if (force[0]!=0.0 or force[1]!=0.0 or force[2]!=0.0) and self.force_step<300:
            self.force_raise_rate()
            force=force*self.force_rate
            self.force_step=self.force_step+1 if self.force_step<300 else 300
        
        # print("---------",force)
        #从位置控制转为速度控制时，辅助力逐步上升
        
        # force=self.goal_trust*force
        
        force = force_origin*self.goal_trust
        
        self.record1(force_m)
        self.record2(force_k)
        self.record3(force_d)
        self.record4(force)
        self.record5(self.Velocity)
        
        # print(self.Velocity)
        self.Force[0]= -force[0]
        self.Force[1]= -force[1]
        self.Force[2] = force[2]

        return

    def record1(self,array):
        
        self.file1.write(str(rospy.Time.now().to_nsec())+' ')
        for i in range(np.size(array)):
            self.file1.write(str(array[i])+' ')
        self.file1.write('\n')
        return
    def record2(self,array):
        # print("force k",array)
        self.file2.write(str(rospy.Time.now().to_nsec())+' ')
        for i in range(np.size(array)):
            self.file2.write(str(array[i])+' ')
        self.file2.write('\n')
        return
    def record3(self,array): 
        self.file3.write(str(rospy.Time.now().to_nsec())+' ')
        for i in range(np.size(array)):
            self.file3.write(str(array[i])+' ')
        self.file3.write('\n')
        return
    def record4(self,array):
        self.file4.write(str(rospy.Time.now().to_nsec())+' ')
        for i in range(np.size(array)):
            self.file4.write(str(array[i])+' ')
        self.file4.write('\n')
        return
    def record5(self,array):
        self.file5.write(str(rospy.Time.now().to_nsec())+' ')
        for i in range(np.size(array)):
            self.file5.write(str(array[i])+' ')
        self.file5.write('\n')
        return
    
    #计算工作空间内任一点的姿态
    def gen_world_ori(self):
        self.path
    
    # 根据四个点插值
    

def get_path_from_bag():
    path = Path()
    position = np.empty(shape=(0, 2))
    # bag_file = '/home/chy/bagfile/end_path_1.bag'
    bag_file = '/home/chy/bagfile/end_demo4.bag'
    bag_data = rosbag.Bag(bag_file, "r")
    info = bag_data.get_type_and_topic_info()
    print(info)
    # perception_data = bag_data.read_messages('/iiwa/state/CartesianPose_end')
    perception_data = bag_data.read_messages('/iiwa/state/CartesianPose_end')
    flag = 0
    for topic, msg, t in perception_data:
        # if flag % 15 == 0:
        # pose = msg.poseStamped
        # x = msg.poseStamped.pose.position.x
        # y = msg.poseStamped.pose.position.y
        pose = msg
        x = msg.poseStamped.pose.position.x
        y = msg.poseStamped.pose.position.y
        # print(x,y)
        position = np.append(position, [[x, y]], axis=0)
        path.poses.append(pose)
        # flag += 1
        # print(flag)
    return path, position


def gene_line(start, goal):
    traj_x = 0.0
    traj_y = 0.0
    position = np.empty(shape=(0, 2))
    path = Path()
    # traj_velocity = np.zeros((100, 2), dtype=np.float)
    for i in range(1000):
        traj_x = start[0]+i*(goal[0]-start[0])/1000
        traj_y = start[1]+i*(goal[1]-start[1])/1000
        t_pose = PoseStamped()
        t_pose.pose.position.x = traj_x
        t_pose.pose.position.y = traj_y
        t_pose.pose.position.z = start[2]
        path.poses.append(t_pose)
        position = np.append(position, [[traj_x, traj_y]], axis=0)
        # traj_velocity[i] = (goal-start)/np.linalg.norm(goal-start)
    return path, position


def local_transfer(point):
    point[0] += 0.002
    point[1] -= 0.001
    return point





# if __name__ == '__main__':
def main():
    position_s = np.array([0.6645, 0.0125, 0.05])
    v_master = np.array([8.07179677e-05, 8.07179677e-05, 8.07179677e-05])
    training = Training()

    path, traj_position = get_path_from_bag()
    # path, traj_position = gene_line(start_point, goal_position[8])

    traj_x, traj_y = np.array_split(traj_position, 2, axis=1)
    training.get_goal(2)
    training.get_goal_trust(0.4)
    training.get_local(position_s, v_master)
    training.get_force()
    training_force = training.Force
    # print(training_force)
    # print(training.mapping_vector)
    # print(training.mapping_point-training.local_point)
    # print(np.sum((training.mapping_vector)*(training.mapping_point-training.local_point)))

    # print(traj_position)
    f1, ax = plt.subplots(1, 1, sharex=False, sharey=False, figsize=(12, 8))
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.plot(traj_x, traj_y, color='k', label='line', linewidth=2)

    for i in range(10):
        position_s = local_transfer(position_s)
        ax.scatter(position_s[0], position_s[1], marker='.',
                   color='red', label='local_point%d' % i, s=300)
        training.get_local(position_s, v_master)
        training.get_force()
        ax.scatter(training.mapping_point[0], training.mapping_point[1],
                   marker='.', color='blue', label='min_point%d' % i, s=300)

    ax.legend(fontsize=10)
    plt.suptitle('Simulation', fontsize=16)
    plt.tight_layout()
    plt.show()
