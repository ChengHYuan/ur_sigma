import numpy as np
import math

class Detection(object) :
    goal_position = np.array(
        [[0.732, 0.04], [0.742, -0.01], [0.692, -0.04], [0.662, -0.06]])
    
    start_point=np.array([0.642,0.06])
    x_d=np.array([0.0,0.0,0.0])
    goal_flag=False
    start_flag=False
    is_back=False
    
    def get_x_d(self,x_d):
        self.x_d=x_d
    
    def fresh_flags(self):
        # 判断是否接近目标
        is_near=False
        for i in range(np.shape(self.goal_position)[0]):
            if np.sqrt(np.sum(((self.x_d[0:2]-self.goal_position[i])**2)))<0.01:
                is_near=True
        if is_near:
            self.goal_flag=True
        else:
            self.goal_flag=False
        
        # 判断是否接近起点
        if np.sqrt(np.sum(((self.x_d[0:2]-self.start_point)**2)))<0.01:
            self.start_flag=True
        else:
            self.start_flag=False
    
    def judge_is_back(self):
        self.fresh_flags()
        
        if self.goal_flag:
            self.is_back=True
        
        if self.start_flag:
            self.is_back=False
            
        return self.is_back
    