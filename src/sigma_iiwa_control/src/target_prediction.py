import rospy
import numpy as np
import matplotlib.pyplot as plt
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


class TagetPredict:
    # 所有目标位置
    # goal_position = np.array([(3.0, 6.0), (6.0, 5.0), (7.0, 3.0)])
    goal_position = np.array([[0.732, 0.04],[0.742,-0.01],[0.692,-0.04],[0.662,-0.06]])
    
    # 起始位置
    start_position = np.array([0.642,0.06])

    # 初始化概率 全部为1
    prob_before = np.ones((goal_position.shape[0], 1), dtype=np.float)
    prob_now = np.ones((goal_position.shape[0], 1), dtype=np.float)
    prob_max1=0.0
    prob_max2=0.0
    
    #概率的log值之和
    prob_log = np.ones((goal_position.shape[0], 1), dtype=np.float)
    #遗忘因子
    forget_factor=0.99
    
    #高斯分布参数
    mean=0.0
    sigma=0.1
    # 信息熵
    entropy = 0.0
    # 目标可信度
    goal_trust = 0.0

    # 存储所有的标准轨迹
    paths_box = []
    paths_size = []

    # 映射点存储
    mapping_points = np.zeros((goal_position.shape[0], 2), dtype=np.float)
    mapping_points_2 = np.zeros((goal_position.shape[0], 2), dtype=np.float)#存入第二近的点
    mapping_velocities = np.zeros((goal_position.shape[0], 2), dtype=np.float)
    mapping_accels = np.zeros((goal_position.shape[0], 2), dtype=np.float)
    min_indexs = np.zeros((goal_position.shape[0], 1), dtype=np.int)
    min_indexs_2 = np.zeros((goal_position.shape[0], 1), dtype=np.int)#存入第二近的点的索引
    min_distances = np.zeros((goal_position.shape[0], 1), dtype=np.float)
    min_distances_2=np.zeros((goal_position.shape[0], 1), dtype=np.float)#存入第二近的距离
    min_distances_before = np.zeros((goal_position.shape[0], 1), dtype=np.float)

    path_tree=[]

    #多线程
    ths=[]
    
    #树
    trees=[]
    
    
    # 目标的速度方向
    goal_velocity = np.zeros((goal_position.shape[0], 2), dtype=np.float)
    # 角度差
    theta_error = np.zeros((goal_position.shape[0], 1), dtype=np.float)

    # 存储之前的速度和位置
    position_before = np.array([0, 0])
    velocity_before = np.array([0, 0])
    box_size = 3
    position_box = [[0.6645, 0.0375]]

    position_now_predict = [0, 0]
    velocity_now_predict = [0, 0]
    
    position_now=[0,0]

    # 目标速度是否可知
    goal_vector_flag = False

    predict_goal = -1
    

    # 归一化的预测位置
    s = 0.005

    def __init__(self):
        self.load_standard_path()
        
        workspace = os.getcwd()
        foldername="data/operator/ff2/f2data"
        folderpath = os.path.join(workspace, foldername)
        try:
            os.mkdir(folderpath)
        except FileExistsError:
            pass
        
        filename1="goal_trust_data.txt"
        filename2="goal_prob.txt"

        
        filepath1 = os.path.join(folderpath, filename1)
        filepath2 = os.path.join(folderpath, filename2)

        
        self.file1=open(filepath1,"w")
        self.file2=open(filepath2,"w")
        
        return

    def __del__(self):
        self.file1.close()
        self.file2.close()

    # 获取当前位置和速度方向
    def get_local(self, position, velocity):
        self.position_now = position
        # print("position:",self.position_now)
        self.velocity_now = velocity
        self.position_box.append(position)
        if len(self.position_box) > self.box_size:
            self.position_box.pop(0)
        return

    # 获取抵抗力
    def get_anti_force(self,anti_force):
        self.anti_force=anti_force
        return
        
    # 计算夹角高斯分布概率
    def normal_distribution_angle(self, theta):
        # 高斯函数均值方差
        # if theta > 0:
        #     return stats.norm.cdf(-theta, self.mean, self.sigma)*2
        # else:
        #     return stats.norm.cdf(theta, self.mean, self.sigma)*2
        return stats.norm.pdf(theta, self.mean, self.sigma)

    # 计算距离高斯分布概率
    def normal_distribution_distance(self,goal):
        mean=0.0
        sigma=0.1
        # print(self.min_distances)
        # print("min mapping point:",self.mapping_points)
        # print("min index",self.min_indexs)
        # print("local point :",self.position_now)
        return stats.norm.pdf(self.min_distances[goal], mean, sigma)
    
    #计算距离变化梯度概率影响
    def gradiant_distance(self,goal):
        
        return math.exp(-(self.min_distances[goal]-self.min_distances_before[goal])*400)
    
    #计算遗忘因子处理后的概率
    def forget_prob(self):
        #Log概率更新
        self.prob_log=self.prob_log*self.forget_factor+np.log(self.prob_now)
        return np.exp(self.prob_log)
    
    
    # 概率更新
    def get_prob(self):
        s = self.s
        # start=rospy.get_time()
        for i in range(self.goal_position.shape[0]):
            # n_to_goal = self.goal_position[i]-self.position_now
            #采用艾米特插值
            # self.get_interpolation(
            #     s, self.goal_position[i], self.goal_velocity[i])
            # self.line_interpolation(
            #     s, self.goal_position[i], self.goal_velocity[i])
            
            #采用直线插值
            # self.least_square(
            #     s, self.goal_position[i], self.goal_velocity[i])
            # self.theta_error[i] = self.angle_of_vector(
            #     self.velocity_now_predict, self.velocity_now)
            # # print('theta %d is %f'%(i,self.theta_error[i]))
            # prob = self.normal_distribution_angle(self.theta_error[i])
            # if np.sum(n_to_goal*self.velocity_now) < 0 and prob > 1:
            #     pass
            # else:
            #     self.prob_now[i] = prob
                
            #采用到参考轨迹的距离计算概率
            # if self.mapping_points[i][0] == 0.0 and self.mapping_points[i][1] == 0.0:
            #     self.get_nearest_point_init(i)
            # else:
            #     self.get_nearest_point_it(i)

            self.get_distance(i)
            self.prob_now[i] =self.gradiant_distance(i)*self.normal_distribution_distance(i)
            # th=Thread(target=self.get_one_prob,args=(i,))
            # th.start()
            # self.ths.append(th)
            # self.prob_now[i] =self.normal_distribution_distance(i)
        # for th in self.ths:
        #     th.join()
        # end=rospy.get_time()
        # print("------------duration----------",(end-start)*1000)
        #概率归一化
        
        self.prob_now = self.prob_now/np.sum(self.prob_now)
        # print('theta %d is %f'%(3,self.theta_error[3]))
        
        # self.prob_before = self.prob_before*self.prob_now
        self.prob_before = self.forget_prob()
        
        # 概率归一化
        # if np.sum(self.prob_before)>1:
        #     self.prob_before=self.prob_before/np.sum(self.prob_before)
        self.prob_before = self.prob_before/np.sum(self.prob_before)
        # print(self.prob_before)
        
        # 计算信息熵
        self.get_entropy()
        
        #计算最大概率和第二概率之间的差值，如果差值较小，则不变,消除目标预测的抖动
        self.prob_max1 = np.sort(self.prob_before,axis=0)[-1]
        self.prob_max2 = np.sort(self.prob_before,axis=0)[-2]
        # print("------------max1-------:",self.prob_max1)
        # print("------------max2-------:",self.prob_max2)
        if self.predict_goal==-1:
            self.predict_goal = np.argmax(self.prob_before)
        if  self.prob_max1-self.prob_max2 >0.05:
            self.predict_goal = np.argmax(self.prob_before)
        
        # if np.max(self.prob_before)>0.9:
        #     self.predict_goal=np.argmax(self.prob_before)
        # print("prob:",self.prob_before)
        self.record1()
        self.record2()
        
        return self.prob_before
    

    
    # 计算信息熵
    def get_entropy(self):
        entropy = 0.0
        for prob in self.prob_before:
            if prob != 0:
                entropy += -prob*np.log2(prob)
            else:
                entropy += 0.0
        self.entropy = entropy
        self.goal_trust = 1-entropy/np.log2(self.goal_position.shape[0])
        # print("trust:",self.goal_trust)

    # 计算向量夹角

    def angle_of_vector(self, v1, v2):
        vector_prod = v1[0] * v2[0] + v1[1] * v2[1]
        length_prod = sqrt(pow(v1[0], 2) + pow(v1[1], 2)) * \
            sqrt(pow(v2[0], 2) + pow(v2[1], 2))
        cos = vector_prod * 1.0 / (length_prod * 1.0 + 1e-6)
        return acos(cos)

    # 计算三次曲线插值
    def get_interpolation(self, s, goal_position, goal_velocity):
        if not self.goal_vector_flag:
            a0 = self.position_before
            a1 = self.velocity_before
            a2 = 1.5*goal_position-1.5*self.position_before-1.5*self.velocity_before
            a3 = -0.5*goal_position+0.5*self.position_before+0.5*self.velocity_before
        else:
            a0 = self.position_before
            a1 = self.velocity_before
            a2 = 3*goal_position-goal_velocity - \
                3*self.position_before-2*self.velocity_before
            a3 = -2*goal_position+goal_velocity + \
                2*self.position_before+self.velocity_before
        self.position_now_predict = a0+a1*s+s*s*a2+s*s*s*a3
        self.velocity_now_predict = a1+2*a2*s+3*a3*s*s
        return self.position_now_predict[0], self.position_now_predict[1]

    # 直线插值
    def line_interpolation(self, s, goal_position, goal_velocity):
        a0 = self.position_before
        a1 = goal_position-self.position_before
        self.position_now_predict = a0+a1*s
        self.velocity_now_predict = a1
        return self.position_now_predict[0], self.position_now_predict[1]

    # 直线最小二乘拟合
    def least_square(self, s, goal_position, goal_velocity):  # 需要多个参数（x,y） 用于生成预测的直线
        box = np.array(self.position_box)
        x, y = np.array_split(box, 2, axis=1)
        x = np.append(x, goal_position[0])
        y = np.append(y, goal_position[1])
        x0 = x[0]
        x1 = x[-1]
        N = x.shape[0]
        sumx = np.sum(x)
        sumy = np.sum(y)
        sumx2 = np.sum(x**2)
        sumxy = np.sum(x*y)
        a1 = (sumxy-sumx*sumy/N)/(sumx2-sumx**2/N)
        a0 = sumy/N-a1*sumx/N

        self.position_now_predict[0] = s*(x1-x0)+x0
        self.position_now_predict[1] = a0+a1*x0+a1*s*(x1-x0)
        self.velocity_now_predict[0] = (x1-x0)
        self.velocity_now_predict[1] = a1*(x1-x0)
        x = x[:-1]
        y = y[:-1]
        y = np.append(y, goal_position[1])
        return self.position_now_predict[0], self.position_now_predict[1]

    # 计算goal的速度方向(默认为当前位置和目标位置的连线)
    def get_goal_velocity(self, position):
        for i in range(self.goal_velocity.shape[0]):
            self.goal_velocity[i] = (
                self.goal_position[i]-position)/np.linalg.norm(self.goal_position[i]-position)
            # print(self.goal_velocity[i])
        return

    # 从轨迹库中获取标准轨迹
    def load_standard_path(self):
        i = 0
        m_str = '/home/chy/sigma_iiwa_simulation/sigma_to_sim_workspace/path/standard_path'+str(i)+'.txt'
        while os.path.exists(m_str):
            a = np.loadtxt(m_str)
            self.paths_box.append(a[:,0:2])
            self.path_tree.append(cKDTree(a[:,0:2]))
            self.paths_size.append(len(a))
            i+=1
            m_str = '/home/chy/sigma_iiwa_simulation/sigma_to_sim_workspace/path/standard_path'+str(i)+'.txt'
        print("-------------load path success! we get %d pathes------------"%(i))
        print('sizes: ',self.paths_size)
        return


    # 初始化检索最近点
    def get_nearest_point_init(self, goal):
        window = 0.1
        size = 0
        l1 = np.sqrt(np.sum((self.position_now-self.start_position)**2))
        l2 = np.sqrt(np.sum((self.position_now-self.goal_position[goal])**2))
        
        size = self.paths_size[goal]
        # print('path %d size is: %d' % (goal, size))
        k_position = l1/(l1+l2)
        k_up = k_position+window
        k_down = k_position-window
        if k_down < 0.0:
            k_down = 0.0
        if k_up > 1.0:
            k_up = 1.0
        min = np.inf
        position = np.array([0.0, 0.0])
        for index in range(int(size*k_down), int(size*k_up)):  # 有限范围搜索
            position[0] = self.paths_box[goal][index][0]
            position[1] = self.paths_box[goal][index][1]
            if np.sqrt(np.sum((position-self.position_now)**2)) < min:
                self.min_indexs[goal] = index
                min = np.sqrt(np.sum((position-self.position_now)**2))

        self.mapping_points[goal][0] = self.paths_box[goal][self.min_indexs[goal][0]][0]
        self.mapping_points[goal][1] = self.paths_box[goal][self.min_indexs[goal][0]][1]
        if self.min_indexs[goal] == 0:
            self.min_indexs[goal] = 1
            print('init index is 0!!')
        elif self.min_indexs[goal] == size-1:
            print('init index is max!!')
            self.min_indexs[goal] == size-2
        else:
            pass
        step = 1/size
        self.mapping_velocities[goal][0] = (self.paths_box[goal][self.min_indexs[goal][0]+1][0] -
                                            self.paths_box[goal][self.min_indexs[goal][0]-1][0])/(2*step)
        self.mapping_velocities[goal][1] = (self.paths_box[goal][self.min_indexs[goal][0]+1][1] -
                                            self.paths_box[goal][self.min_indexs[goal][0]-1][1])/(2*step)
        self.mapping_accels[goal][0] = ((self.paths_box[goal][self.min_indexs[goal][0]+1][0] -
                                         self.paths_box[goal][self.min_indexs[goal][0]][0])/step-(self.paths_box[goal][self.min_indexs[goal][0]][0] - self.paths_box[goal][self.min_indexs[goal][0]-1][0])/step)/step
        self.mapping_accels[goal][1] = ((self.paths_box[goal][self.min_indexs[goal][0]+1][1] -
                                         self.paths_box[goal][self.min_indexs[goal][0]][1])/step-(self.paths_box[goal][self.min_indexs[goal][0]][1] - self.paths_box[goal][self.min_indexs[goal][0]-1][1])/step)/step
        self.min_distances_before[goal]=self.min_distances[goal]
        self.min_distances[goal]=min
        print("goal %d init success"%(goal))
        
        return

    # 牛顿迭代检索最小值
    def get_nearest_point_it(self, goal):
        size = self.paths_size[goal]
        s = float(self.min_indexs[goal]/size)
        s_1 = -1.0
        step = 0
        ind_s = self.min_indexs[goal]
        min_point = self.mapping_points[goal]
        min_vector = self.mapping_velocities[goal]
        min_accel = self.mapping_accels[goal]
        local_point = [self.position_now[0],self.position_now[1]]
        # print('ind_s',self.min_indexs)
        # print('min_accel:', self.mapping_accels)
        # print('min_vector:', self.mapping_velocities)
        # print('min_point: ', self.mapping_points)
        delta_s = 2
        
        while np.abs(delta_s)*size >= 1:
           
            delta_s = np.sum((min_point-local_point)*min_vector) / \
                (np.sum(min_vector**2)+np.sum((min_point-local_point)*min_accel))
            s_1 = s-delta_s
            print(goal)
            print('delta_s: ',delta_s)
            if s_1>1:
                s_1=1
            if s_1<0:
                s_1=0
            
            ind_s = int(s_1*size)
            # print(delta_s*self.num)
            # print(self.path.poses[ind_s].pose.position.x,self.path.poses[ind_s+1].pose.position.x,self.path.poses[ind_s+2].pose.position.x)
            min_point[0] = self.paths_box[goal][ind_s][0]
            min_point[1] = self.paths_box[goal][ind_s][1]

            if self.min_indexs[goal] <= 0:
                self.min_indexs[goal] = 1
                print('min index is 0!!')
            elif self.min_indexs[goal] >= size-1:
                print('min index is max!!')
                self.min_indexs[goal] = size-2
            else:
                pass
            step = 1/size
            min_vector[0] = (self.paths_box[goal][ind_s+1][0] -
                                                self.paths_box[goal][ind_s-1][0])/(2*step)
            min_vector[1] = (self.paths_box[goal][ind_s+1][1] -
                                                self.paths_box[goal][ind_s-1][1])/(2*step)
            min_accel[0] = ((self.paths_box[goal][ind_s+1][0] -
                                             self.paths_box[goal][ind_s][0])/step-(self.paths_box[goal][ind_s][0] - self.paths_box[goal][ind_s-1][0])/step)/step
            min_accel[1] = ((self.paths_box[goal][ind_s+1][1] -
                                             self.paths_box[goal][ind_s][1])/step-(self.paths_box[goal][ind_s][1] - self.paths_box[goal][ind_s-1][1])/step)/step
            s = s_1
            step += 1
            if s==1 or s==0:
                break
            if step > 100:
                print('itr time to long!!!')
                break
        self.mapping_points[goal] = min_point
        self.mapping_velocities[goal] = min_vector
        self.mapping_accels[goal] = min_accel
        
        self.min_distances_before[goal]=self.min_distances[goal]
        # print(self.position_now)
        # print(local_point)
        # print(min_point)
        # print(local_point-min_point)
        self.min_distances[goal]=np.sqrt(np.sum((local_point-min_point)**2))
        # print('%d distance:%f'%(goal,self.min_distances[goal]))
        # print('min_point: ',min_point)
        # print('goal %d itr is ok'%goal)
        # print(self.min_indexs)
        self.min_indexs[goal] = ind_s
        return

    #才用shapely计算距离,各个轨迹的最近点以及第二近的点
    def get_distance(self,goal):
        self.min_distances_before[goal]=self.min_distances[goal]
        dis,ind=self.path_tree[goal].query(self.position_now)
        # dis_2,ind_2=self.path_tree[goal].query(self.position_now,k=2)
        # print(dis)
        self.min_distances[goal]=dis
        # self.min_distances_2[goal]=dis_2
        
        self.min_indexs[goal]=ind
        # self.min_indexs_2[goal]=ind_2
        
        self.mapping_points[goal]=self.paths_box[goal][ind]
        # self.mapping_points_2[goal]=self.paths_box[goal][ind_2]
        return

    # 画出概率分布函数
    def print_pdf(self):
        style.use('fivethirtyeight')
        x = np.linspace(-7, 7, 1000)
        f, ax = plt.subplots(1, 1, sharex=True, sharey=True, figsize=(12, 8))
        mu = self.mean
        sd = self.sigma
        y = stats.norm(mu, sd).pdf(x)
        ax.plot(x, y)
        ax.plot(0, 0, label='mu={:3.2f}\nsigma={:3.2f}'.format(
            mu, sd), alpha=0)
        ax.legend(fontsize=10)
        plt.suptitle('Gaussian PDF', fontsize=16)
        plt.tight_layout()
        plt.show()
    
    def record1(self):
        self.file1.write(str(rospy.Time.now().to_nsec())+' ')
        self.file1.write(str(self.goal_trust[0])+' ')
        self.file1.write('\n')
        return
    
    def record2(self):
        self.file2.write(str(rospy.Time.now().to_nsec())+' ')
        for i in range(np.size(self.prob_before)):
            self.file2.write(str(self.prob_before[i][0])+' ')
        self.file2.write('\n')
        return

def gene_line(start, goal):
    traj_x = np.zeros(100)
    traj_y = np.zeros(100)
    traj_velocity = np.zeros((100, 2), dtype=np.float)
    for i in range(100):
        traj_x[i] = start[0]+i*(goal[0]-start[0])/100
        traj_y[i] = start[1]+i*(goal[1]-start[1])/100
        traj_velocity[i] = (goal-start)/np.linalg.norm(goal-start)
    return traj_x, traj_y, traj_velocity


def get_path_from_bag():
    position = np.empty(shape=(0, 2))
    velocity = np.empty(shape=(0, 2))
    position_before = [0, 0]
    velocity_now = [[0, 0]]
    bag_file = '/home/chy/bagfile/end_demo3.bag'
    bag_data = rosbag.Bag(bag_file, "r")
    info = bag_data.get_type_and_topic_info()
    print(info)
    perception_data = bag_data.read_messages('/iiwa/state/CartesianPose_end')
    for topic, msg, t in perception_data:
        x = msg.poseStamped.pose.position.x
        y = msg.poseStamped.pose.position.y
        # 按照距离采样
        if position.size == 0:
            # 将位置坐标添加
            position = np.append(position, [[x, y]], axis=0)  # 添加整行元素
            position_before[0] = x
            position_before[1] = y
        else:
            if sqrt((x-position[-1][0])**2+(y-position[-1][1])**2) > 0.0001:
                position = np.append(position, [[x, y]], axis=0)  # 添加整行元素
                velocity_now[0][0] = x-position_before[0]
                velocity_now[0][1] = y-position_before[1]
                velocity_now = velocity_now/np.linalg.norm(velocity_now)
                velocity = np.append(velocity, velocity_now, axis=0)
            position_before = [x, y]

    return position, velocity


def main():
# if __name__ == '__main__':

    print(get_path_from_bag())

    predict = TagetPredict()

    predict.print_pdf()
    style.use('fivethirtyeight')
    length = 100
    s_pre = int(length/20)

    s = np.linspace(0, 1, length)
    f1, ax = plt.subplots(1, 2, sharex=False, sharey=False, figsize=(12, 8))
    colors = ["darkred", "red", 'darkorange', "blue", "darkblue", 'burlywood',
              'cadetblue', 'chartreuse', 'chocolate', 'coral', 'cornflowerblue', 'deepskyblue']

    ax[0].grid(True, linestyle="--", alpha=0.5)
    ax[1].grid(True, linestyle="--", alpha=0.5)
    ax[0].set_xlim([0.60, 0.80])
    ax[0].set_ylim([-0.07, 0.07])
    ax[1].set_xlim([-1, 150])
    ax[1].set_ylim([-0.01, 1.01])

    # # #当前位置
    # position = np.array([2, 2])
    # velocity = np.array([1, 5])

    # # 初始位置生成轨迹
    # traj_x, traj_y, traj_velocity = gene_line(
    #     position, predict.goal_position[2])
    traj_position, traj_velocity = get_path_from_bag()
    traj_x, traj_y = np.array_split(traj_position, 2, axis=1)
    ax[1].set_xlim([-1, len(traj_position)])
    ax[1].set_ylim([-0.01, 1.01])
    # traj_position = np.array(list(zip(traj_x, traj_y)))

    ax[0].plot(traj_x, traj_y, color='k', label='line', linewidth=2)
    # 画出目标点
    for i in range(predict.goal_position.shape[0]):
        ax[0].scatter(predict.goal_position[i][0], predict.goal_position[i]
                      [1], marker='x', color=colors[i], label='goal_'+('%d' % i), s=300)

    line = []

    def init():
        # line1, = ax[0].plot([], [], color='g', linestyle='--',
        #                     label='line1', linewidth=2)
        # line2, = ax[0].plot([], [], color='b', linestyle='--',
        #                     label='line2', linewidth=2)
        # line3, = ax[0].plot([], [], color='r', linestyle='--',
        #                     label='line3', linewidth=2)
        point1 = ax[0].scatter([], [], marker='.', color='g',
                               label='predict1', s=300)
        point2 = ax[0].scatter([], [], marker='.', color='b',
                               label='predict2', s=300)
        point3 = ax[0].scatter([], [], marker='.', color='r',
                               label='predict3', s=300)
        point4 = ax[0].scatter([], [], marker='+', color='c',
                               label='local_point', s=300)
        point5 = ax[0].scatter([], [], marker='+', color='k',
                               label='local_point', s=300)
        return point1, point2, point3, point4,point5

    def buildmebarchart(m=int):
        predict.position_before = traj_position[m]
        predict.velocity_before = traj_velocity[m]

        # predict.get_goal_velocity(position)
        # predict.goal_vector_flag=True

        # 生成插值轨迹
        x_1 = np.zeros((100, 1), dtype=np.float)
        y_1 = np.zeros((100, 1), dtype=np.float)
        x_2 = np.zeros((100, 1), dtype=np.float)
        y_2 = np.zeros((100, 1), dtype=np.float)
        x_3 = np.zeros((100, 1), dtype=np.float)
        y_3 = np.zeros((100, 1), dtype=np.float)
        for i in range(s.shape[0]):
            # x_1[i], y_1[i] = predict.get_interpolation(
            #     s[i], predict.goal_position[4], predict.goal_velocity[4])
            # x_2[i], y_2[i] = predict.get_interpolation(
            #     s[i], predict.goal_position[1], predict.goal_velocity[1])
            # x_3[i], y_3[i] = predict.get_interpolation(
            #     s[i], predict.goal_position[2], predict.goal_velocity[2])
            # x_1[i], y_1[i] = predict.line_interpolation(
            #     s[i], predict.goal_position[4], predict.goal_velocity[4])
            # x_2[i], y_2[i] = predict.line_interpolation(
            #     s[i], predict.goal_position[1], predict.goal_velocity[1])
            # x_3[i], y_3[i] = predict.line_interpolation(
            #     s[i], predict.goal_position[2], predict.goal_velocity[2])
            x_1[i], y_1[i] = predict.least_square(
                s[i], predict.goal_position[0], predict.goal_velocity[0])
            x_2[i], y_2[i] = predict.least_square(
                s[i], predict.goal_position[1], predict.goal_velocity[1])
            x_3[i], y_3[i] = predict.least_square(
                s[i], predict.goal_position[2], predict.goal_velocity[2])
        # line1, = ax[0].plot(x_1, y_1, color='g', linestyle='--',
        #                     label='line1', linewidth=2)
        # line2, = ax[0].plot(x_2, y_2, color='b', linestyle='--',
        #                     label='line2', linewidth=2)
        # line3, = ax[0].plot(x_3, y_3, color='r', linestyle='--',
        #                     label='line3', linewidth=2)
        # 标出预测点
        
        point1 = ax[0].scatter(predict.mapping_points[0][0], predict.mapping_points[0][1],
                               marker='.', color='g', label='predict1', s=300)
        point2 = ax[0].scatter(predict.mapping_points[1][0], predict.mapping_points[1][1],
                               marker='.', color='b', label='predict2', s=300)
        point3 = ax[0].scatter(predict.mapping_points[2][0], predict.mapping_points[2][1],
                               marker='.', color='r', label='predict3', s=300)
        point4 = ax[0].scatter(predict.mapping_points[3][0], predict.mapping_points[3][1],
                               marker='.', color='c', label='predict4', s=300)
        point5 = ax[0].scatter(
            traj_x[m+1], traj_y[m+1], marker='x', color='k', label='local_point', s=300)

        predict.get_local([traj_x[m+1][0], traj_y[m+1][0]], traj_velocity[m+5])
        # predict.get_goal_velocity(predict.position_before)

        prob = predict.get_prob()
        # print("nearest_point:",predict.mapping_points)
        # print("mapping index: ",predict.min_indexs)
        
        for j in range(prob.shape[0]):
            ax[1].scatter(m, prob[j], marker='.', color=colors[j],
                          label='prob_'+('%d' % i), s=200)
        ax[1].scatter(m, predict.goal_trust, marker='.', color=colors[j+1],
                          label='goal_trust'+('%d' % i), s=200)
        # ax[1].scatter(m, prob[8], marker='.', color=colors[5],
        #               label='prob_'+('%d' % i), s=200)
        # if np.max(prob) < 0.9:
        #     print(prob)
        # else:
        #     print('goal is :%d' % (np.argmax(prob)+1))
        # print('------------------')
        print('goal is :%d  entropy is :%f' %
              (predict.predict_goal, predict.entropy))
        print('----------------goal trust is %f ----------------' %
              (predict.goal_trust))

        return  point1, point2, point3, point4,point5

    animator = ani.FuncAnimation(
        f1, buildmebarchart, interval=10, init_func=init, frames=traj_position.shape[0]-6, blit=True, repeat=False)
    ax[0].legend(fontsize=10)
    ax[1].legend(fontsize=10)
    plt.suptitle('Simulation', fontsize=16)
    plt.tight_layout()
    plt.show()
