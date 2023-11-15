import rospy
import math
import numpy as np
import math
import os


class PositionRate:
    radius = 0.07  # 设置气泡初始半径
    r_fade_rate = 1.0  # 设置气泡半径缩放比例
    r_fade_rate_before = 1.0  # 之前气泡半径缩放比例
    stay_para = 0.0  # 驻留参数
    stay_base = 16.0
    goal_base_distance = 0.13  # 目标间的最远距离

    k_position = np.array([0.15,0.15,0.15])  # 位置控制基础缩放比例
    k_distance = 1.0  # 与目标距离相关的衰减参数 （时变）
    k_goal = 1.0  # 由于预估的目标位置而产生的参数（时不变）
    k_punish = 1.0  # 惩罚参数 和理想轨迹差异越大，速度越小
    k_diffcult = 1.0  # 困难参数，当轨迹曲率较大时减小

    v_master = np.array([0.0, 0.0, 0.0])
    a_master = np.array([0.0, 0.0, 0.0])
    v_master_true=np.array([0.0, 0.0, 0.0])#采集到的主手速度

    k_trust = 1.0  # 目标置信度

    x_center = np.array([0.0, 0.0, 0.0])  # 气泡球心
    x_init = np.array([0.0, 0.0, 0.0])  # petal闭合时的位置
    x_local = np.array([0.0, 0.0, 0.0])  # 当前主手位置（相对闭合位置）
    x_local_before = np.array([0.0, 0.0, 0.0])  # 之前主手位置

    x_d = np.array([0.0, 0.0, 0.0])  # 主手差值
    x_d_save = np.array([0.0, 0.0, 0.0])  # 主手存储值，位置控制时使用
    x_s = np.array([0.0, 0.0, 0.0])  # 从手实际值
    
    #判断是否接近目标
    close_flag=False

    velocity_mode = False

    x_start = np.array([0.0, 0.0])  # 起始位置

    goal_position = np.array(
        [[0.732, 0.04], [0.742, -0.01], [0.692, -0.04], [0.662, -0.06]])

    l_max = 0.10606  # 起始点与终止点的最远距离
    
    boundary_force=np.array([0.0,0.0,0.0])
    boundary_k=10
    

    # 初始化函数

    def __init__(self):
        
        workspace = os.getcwd()
        foldername="data/operator/ff2/f2data"
        folderpath = os.path.join(workspace, foldername)
        try:
            os.mkdir(folderpath)
        except FileExistsError:
            pass
        
        filename1="radius_data1.txt"
        filename2="x_data.txt"
        filename3="distance_data.txt"
        filename4="x_m_data.txt"
        filename5="x_center.txt"
        filename6="boundry_force.txt"
        
        filepath1 = os.path.join(folderpath, filename1)
        filepath2 = os.path.join(folderpath, filename2)
        filepath3 = os.path.join(folderpath, filename3)
        filepath4 = os.path.join(folderpath, filename4)
        filepath5 = os.path.join(folderpath, filename5)
        filepath6 = os.path.join(folderpath, filename6)
        
        self.file1=open(filepath1,"w")
        self.file2=open(filepath2,"w")
        self.file3=open(filepath3,"w")
        self.file4=open(filepath4,"w")
        self.file5=open(filepath5,"w")
        self.file6=open(filepath6,"w")
        pass

    def __del__(self):
        self.file1.close()
        self.file2.close()
        self.file3.close()
        self.file4.close()
        self.file5.close()
        self.file6.close()
        return
    # 获取当前主手位置（相对于闭合点）
    def get_x_init(self, x_origin, y_origin, z_origin, button_flag,m_velocity):
        if not button_flag:
            self.fresh_para()
        if  self.x_local_before[0] == 0.0 and self.x_local_before[1]==0.0:
            self.x_local_before[0] = x_origin
            self.x_local_before[1] = y_origin
            self.x_local_before[2] = z_origin
        else:
            self.x_local_before = self.x_local
        # print("x_local_before:",self.x_local_before)
        self.v_master_true=m_velocity
        self.x_local[0] = x_origin
        self.x_local[1] = y_origin
        self.x_local[2] = z_origin
        
        self.record4(self.x_local,self.file4)
        return

    def fresh_para(self):
        self.x_d = np.array([0.0, 0.0, 0.0])
        self.goal = 0
        self.k_distance = 1.0
        self.k_trust = 1.0
        self.x_d_save = np.array([0.0, 0.0, 0.0])
        self.r_fade_rate = 1.0
        self.r_fade_rate_before = 1.0
        self.stay_para = 0
        self.x_center = np.array([0.0, 0.0, 0.0])
        self.v_master=np.array([0.0, 0.0, 0.0])

        self.a_master = np.array([0.0, 0.0, 0.0])
        self.boundary_force=np.array([0.0, 0.0, 0.0])
        # print("para freashed!!!!")

    # 获取初始点位置
    def get_x_start(self, start_point):
        # 只取x,y坐标
        self.x_start[0] = start_point[0]
        self.x_start[1] = start_point[1]
        return

    # 获取从手当前位置
    def get_x_s(self, x_s):
        self.x_s = x_s
        self.record2(self.x_s,self.file2)
        # print(x_s)
        return

    # 从手期望位置获取 主手位置和主手积分的加权和
    def get_xsd(self):
        self.get_k_distance()
        # print(self.x_center)
        if self.k_distance>0.8:
            self.close_flag=False
        # self.x_center_berth()
        self.get_radius_rate()
        # print("k_distance: ",self.k_distance)
        
        if self.k_distance>0.8:
            if np.sqrt(np.sum((self.x_local-self.x_center)**2)) > self.r_fade_rate*self.radius:  # 速度模式
                if self.velocity_mode == False:
                    self.velocity_mode = True
                radius_d = self.radius*self.r_fade_rate
                self.get_x_center()
                x_vector = (self.x_local-self.x_center) / \
                    np.sqrt(np.sum((self.x_local-self.x_center)**2))
                delta_v = (
                    np.sqrt(np.sum((self.x_local-self.x_center)**2))-radius_d)
                
                #获取气泡边界力
                force_vector=-(self.x_local-self.x_center)/np.sum((self.x_local-self.x_center)**2)
                self.boundary_force=delta_v*force_vector*self.boundary_k
                
                # print(self.boundary_force)
                
                #限速
                delta_v = delta_v if delta_v < 0.005 else 0.005
                
                self.v_master = self.k_goal*self.k_distance*self.k_punish * \
                    self.k_diffcult*x_vector*delta_v*0.005
                
                self.a_master=self.v_master_true
                # print(self.a_master)

                # print("----------v_master-------",self.v_master)
                # print("----------k_goal-------",self.k_goal)
                # print("----------k_distance-------",self.k_distance)
                # print("----------velocity_origin-------",np.sqrt(np.sum((self.x_local-self.x_center)**2))-radius_d)

                self.x_d = self.x_d+self.v_master
                if (self.stay_para <= 125):
                    self.stay_para += 1.0


            else:  # 位置模式
                if self.velocity_mode:
                    self.k_position[2]=0.15
                    self.x_d_save = self.x_d-(self.x_local-self.x_center)*self.k_position
                    self.velocity_mode = False
                #计算当处于起始点或目标点附近时，让z方向缩放比例变大
                
                # print('k_position:',self.k_position)
                self.x_d = self.x_d_save+(self.x_local-self.x_center)*self.k_position
                # print("self save: ",self.x_d_save)
                # print("---x_center",self.x_center)
                # print("---x_d",self.x_d)
                if self.stay_para > 0:
                    self.stay_para -= 1.0

        else:# 在精细区域内
            if np.sqrt(np.sum((self.x_local[0:2]-self.x_center[0:2])**2)) > self.r_fade_rate*self.radius:  # 速度模式
                if self.velocity_mode == False:
                    self.velocity_mode = True
                radius_d = self.radius*self.r_fade_rate
                self.get_x_center()
                x_vector = (self.x_local-self.x_center) / \
                    np.sqrt(np.sum((self.x_local-self.x_center)**2))
                delta_v = (
                    np.sqrt(np.sum((self.x_local-self.x_center)**2))-radius_d)
                
                #获取气泡边界力
                force_vector=-(self.x_local-self.x_center)/np.sum((self.x_local-self.x_center)**2)
                self.boundary_force=delta_v*force_vector*self.boundary_k
                self.boundary_force[2]=0.0
                
                # print(self.boundary_force)
                
                #限速
                delta_v = delta_v if delta_v < 0.003 else 0.003
                
                self.v_master = self.k_goal*self.k_distance*self.k_punish * \
                    self.k_diffcult*x_vector*delta_v*0.005
                
                self.a_master=self.v_master_true


                # print("----------v_master-------",self.v_master)
                # print("----------k_goal-------",self.k_goal)
                # print("----------k_distance-------",self.k_distance)
                # print("----------velocity_origin-------",np.sqrt(np.sum((self.x_local-self.x_center)**2))-radius_d)

                self.x_d = self.x_d+self.v_master
                if (self.stay_para <= 125):
                    self.stay_para += 1.0
            
            else:  # 位置模式
                if self.velocity_mode:
                    if not self.close_flag:
                        self.close_flag=True
                        self.x_center=self.x_local.copy()
                        self.x_d_save=self.x_d
                        # print('--------------')
                        self.v_master=np.array([0.0,0.0,0.0])
                        self.k_position[2]=0.15
                # print('k_position:',self.k_position)
                self.x_d = self.x_d_save+(self.x_local-self.x_center)*self.k_position
                # print("self save: ",self.x_d_save)
                # print("---x_center",self.x_center)
                # print("---x_d",self.x_d)
                if self.stay_para > 0:
                    self.stay_para -= 1.0
                
        # if np.sqrt(np.sum((self.x_local-self.x_center)**2)) > self.r_fade_rate*self.radius:  # 速度模式
        #     if self.velocity_mode == False:
        #         self.velocity_mode = True
        #     radius_d = self.radius*self.r_fade_rate
        #     self.get_x_center()
        #     x_vector = (self.x_local-self.x_center) / \
        #         np.sqrt(np.sum((self.x_local-self.x_center)**2))
        #     delta_v = (
        #         np.sqrt(np.sum((self.x_local-self.x_center)**2))-radius_d)
            
        #     #获取气泡边界力
        #     force_vector=-(self.x_local-self.x_center)/np.sum((self.x_local-self.x_center)**2)
        #     self.boundary_force=delta_v*force_vector*self.boundary_k
            
        #     # print(self.boundary_force)
            
        #     #限速
        #     delta_v = delta_v if delta_v < 0.003 else 0.003
            
        #     self.v_master = self.k_goal*self.k_distance*self.k_punish * \
        #         self.k_diffcult*x_vector*delta_v*0.005
            
        #     self.a_master=self.v_master_true
        #     # print(self.a_master)

        #     # print("----------v_master-------",self.v_master)
        #     # print("----------k_goal-------",self.k_goal)
        #     # print("----------k_distance-------",self.k_distance)
        #     # print("----------velocity_origin-------",np.sqrt(np.sum((self.x_local-self.x_center)**2))-radius_d)

        #     self.x_d = self.x_d+self.v_master
        #     if (self.stay_para <= 125):
        #         self.stay_para += 1.0
        
        
        # else:  # 位置模式
        #     if self.velocity_mode:
        #         if (self.k_distance<=0.5) and (not self.close_flag):
        #             self.close_flag=True
        #             self.x_center=self.x_local.copy()
        #             self.x_d_save=self.x_d
        #             print('--------------')
        #             self.v_master=np.array([0.0,0.0,0.0])
        #             self.k_position[2]=0.5
        #         else:
        #             self.k_position[2]=0.1
        #             self.x_d_save = self.x_d-(self.x_local-self.x_center)*self.k_position
        #         self.velocity_mode = False
        #     #计算当处于起始点或目标点附近时，让z方向缩放比例变大
            
        #     print('k_position:',self.k_position)
        #     self.x_d = self.x_d_save+(self.x_local-self.x_center)*self.k_position
        #     # print("self save: ",self.x_d_save)
        #     # print("---x_center",self.x_center)
        #     # print("---x_d",self.x_d)
        #     if self.stay_para > 0:
        #         self.stay_para -= 1.0
        self.record5(self.x_center,self.file5)
        self.record6(self.boundary_force,self.file6)
        # print("boundary_force: ",self.boundary_force)
        # print(self.v_master)
        return

    def get_radius_rate(self):  # 通过sigmoid函数进行平滑变化
        self.r_fade_rate_before = self.r_fade_rate
        self.stay_para=self.stay_para*self.k_distance
        self.r_fade_rate = 0.85 / \
            (1+math.exp((self.stay_para/self.stay_base)-4))+0.15
        # print("-----radius rate: ",self.r_fade_rate)
        # print("---------------stay_para:",self.stay_para)
        self.record1(self.r_fade_rate,self.file1)
        return

    def get_k_goal(self, goal, goal_trust):
        self.goal = goal
        # 计算初始位置与目标位置距离对速度的影响
        self.k_goal = np.sqrt(
            np.sum((self.goal_position[goal]-self.x_start)**2))/self.goal_base_distance
        # print("k_goal: ",self.k_goal)
        return

    def get_k_distance(self):
        scale = 400
        # 用 sigmoid 函数让 goal 快速衰减
        distance_goal = np.sqrt(
            np.sum((self.x_s[:2]-self.goal_position[self.goal])**2))
        distance_start=np.sqrt(
            np.sum((self.x_s[:2]-self.x_start)**2))
        k_goal=0.8/(1.0+math.exp(-distance_goal*scale+4.0))+0.2
        k_start=0.8/(1.0+math.exp(-distance_start*scale+4.0))+0.2
        #当处于起始点和目标点时相同待遇
        self.k_distance = k_goal*k_start
        
        self.record3(self.k_distance,self.file3)
        # print('distance k :%f'%self.k_distance)
        
        return
    
    def get_x_center(self):
        # print(self.x_local)
        x_vector = (self.x_local_before-self.x_center) / \
            np.sqrt(np.sum((self.x_local_before-self.x_center)**2))
        # print(x_vector)
        # print(np.sqrt(np.sum((self.x_local-self.x_center)**2)))
        # print(self.r_fade_rate*self.radius)
        # print('--------------------------------')
        self.x_center = self.x_center+x_vector * \
            (self.r_fade_rate_before-self.r_fade_rate)*self.radius
        return

    
    def get_k_punish(self):
        return

    def get_k_diffcult(self):
        return
                
    def record1(self,array,file):
        file.write(str(rospy.Time.now().to_nsec())+' ')
        file.write(str(array))
        file.write('\n')
        return
    def record2(self,array,file):
        file.write(str(rospy.Time.now().to_nsec())+' ')
        for i in range(np.size(array)):
            file.write(str(array[i])+' ')
        file.write('\n')
        return
    def record3(self,array,file):
        file.write(str(rospy.Time.now().to_nsec())+' ')
        file.write(str(array)+' ')
        file.write('\n')
        return  
    def record4(self,array,file):
        file.write(str(rospy.Time.now().to_nsec())+' ')
        for i in range(np.size(array)):
            file.write(str(array[i])+' ')
        file.write('\n')
        return
    def record5(self,array,file):
        file.write(str(rospy.Time.now().to_nsec())+' ')
        for i in range(np.size(array)):
            file.write(str(array[i])+' ')
        file.write('\n')
        return 
    def record6(self,array,file):
        file.write(str(rospy.Time.now().to_nsec())+' ')
        for i in range(np.size(array)):
            file.write(str(array[i])+' ')
        file.write('\n')
        return 


# if __name__=='__main__':
def main():
    PR = PositionRate()
    diff_x_origin = 0.1
    diff_y_origin = 0.1
    diff_z_origin = 0.1
    position_s = np.array([0.7395, 0.0375, 0.05])
    start_point = np.array([0.6645, 0.0375, 0.0530])
    for i in range(0, 350):
        PR.get_x_init(diff_x_origin, diff_y_origin, diff_z_origin)
        PR.get_x_s(position_s)
        PR.get_x_start(start_point)
        PR.get_k_goal(11, 0.5)
        PR.get_xsd()
        # print(PR.v_master)
        print(PR.x_d)
        # print(PR.r_fade_rate_before)
    x_d = PR.x_d
    # print(x_d)


# if __name__=='__main__':
#     main()
