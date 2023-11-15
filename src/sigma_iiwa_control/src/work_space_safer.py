import numpy as np


class WorkSafe(object):
    init_point=[0.702, 0,0.08]
    lx=0.16
    ly=0.16
    lz=0.16
    workspace=np.array([[init_point[0]-lx/2,init_point[0]+lx/2],
                       [init_point[1]-ly/2,init_point[1]+ly/2],
                       [0.01,lz+0.01]])
    
    force_k=2000
    
    end_point=np.array([0.0,0.0,0.0])
    delta_over=np.array([0.0,0.0,0.0])
    safe_force=np.array([0.0,0.0,0.0])
    
    def get_safe_force(self,end_point):
        self.end_point=end_point
        self.check_end_point()
        self.safe_force=self.delta_over*self.force_k
        # print("end_point :",self.end_point)
        # print("safe force: ",self.safe_force)
        return self.safe_force
        
        
    def check_end_point(self):
        # print("workspace",self.workspace)
        if self.end_point[0]<self.workspace[0][0]:
            self.delta_over[0]=self.end_point[0]-self.workspace[0][0]
        elif self.end_point[0]>self.workspace[0][1] :
            self.delta_over[0]=self.end_point[0]-self.workspace[0][1]
        else:
            self.delta_over[0]=0.0
        
        if self.end_point[1]<self.workspace[1][0]:
            self.delta_over[1]=self.end_point[1]-self.workspace[1][0]
        elif self.end_point[1]>self.workspace[1][1] :
            self.delta_over[1]=self.end_point[1]-self.workspace[1][1]
        else:
            self.delta_over[1]=0.0
            
        if self.end_point[2]<self.workspace[2][0]:
            self.delta_over[2]=self.end_point[2]-self.workspace[2][0]
        elif self.end_point[2]>self.workspace[2][1] :
            self.delta_over[2]=self.end_point[2]-self.workspace[2][1]
        else:
            self.delta_over[2]=0.0
        self.delta_over[2]=-self.delta_over[2]
        
        # print("force delta: ",self.delta_over)
        