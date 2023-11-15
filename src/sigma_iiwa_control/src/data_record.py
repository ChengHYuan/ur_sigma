import os
from geometry_msgs.msg import PoseStamped
import rospy



class DataRecord(object):
    x_m=PoseStamped()
    x_s=PoseStamped()
    x_m_abs=PoseStamped()
    x_s_abs=PoseStamped()
    flag=0
    pedal_time=0
    
    def __init__(self,name):
        str=name
        workspace = os.getcwd()
        foldername="data/operator/"
        foldername_1= os.path.join(foldername,str)
        folderpath = os.path.join(workspace, foldername_1)
        try:
            os.mkdir(folderpath)
        except FileExistsError:
            pass
        
        filename1="x_s.txt"
        filename2="x_m.txt"
        filename3="pedal_num.txt"
        filename4="x_s_abs.txt"
        filename5="x_m_abs.txt"
        
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
        
    def __del__(self):
        self.file1.close()
        self.file2.close()
        self.file3.close()
        self.file4.close()
        self.file5.close()
        
        
    def record_x_m(self,x_m):
        self.x_m=x_m
        self.file2.write(str(rospy.Time.now().to_nsec())+' ')
        self.file2.write(str(self.x_m.pose.position.x)+' '+str(self.x_m.pose.position.y)+' '+str(self.x_m.pose.position.z))
        self.file2.write(' '+str(self.x_m.pose.orientation.x)+' '+str(self.x_m.pose.orientation.y)+' '+str(self.x_m.pose.orientation.z)+' '+str(self.x_m.pose.orientation.w))
        self.file2.write('\n')
        return
    def record_x_s(self,x_s):
        self.x_s=x_s
        self.file1.write(str(rospy.Time.now().to_nsec())+' ')
        self.file1.write(str(self.x_s.pose.position.x)+' '+str(self.x_s.pose.position.y)+' '+str(self.x_s.pose.position.z))
        self.file1.write(' '+str(self.x_s.pose.orientation.x)+' '+str(self.x_s.pose.orientation.y)+' '+str(self.x_s.pose.orientation.z)+' '+str(self.x_s.pose.orientation.w))
        self.file1.write('\n')
        return
    def record_x_m_abs(self,x_m_abs):
        self.x_m_abs=x_m_abs
        self.file5.write(str(rospy.Time.now().to_nsec())+' ')
        self.file5.write(str(self.x_m_abs.pose.position.x)+' '+str(self.x_m_abs.pose.position.y)+' '+str(self.x_m_abs.pose.position.z))
        self.file5.write(' '+str(self.x_m_abs.pose.orientation.x)+' '+str(self.x_m_abs.pose.orientation.y)+' '+str(self.x_m_abs.pose.orientation.z)+' '+str(self.x_s.pose.orientation.w))
        self.file5.write('\n')
        return
    def record_x_s_abs(self,x_s_abs):
        self.x_s_abs=x_s_abs
        self.file4.write(str(rospy.Time.now().to_nsec())+' ')
        self.file4.write(str(self.x_s_abs.pose.position.x)+' '+str(self.x_s_abs.pose.position.y)+' '+str(self.x_s_abs.pose.position.z))
        self.file4.write(' '+str(self.x_s_abs.pose.orientation.x)+' '+str(self.x_s_abs.pose.orientation.y)+' '+str(self.x_s_abs.pose.orientation.z)+' '+str(self.x_s.pose.orientation.w))
        self.file4.write('\n')
        return
    def record_pedral(self,flag):
        if flag==1 and self.flag==0:
            self.flag=1
            self.pedal_time=self.pedal_time+1
            self.file3.write(str(rospy.Time.now().to_nsec())+' ')
            self.file3.write(str(self.pedal_time))
            self.file3.write('\n')
        elif flag==0 and self.flag==1:
            self.flag=0
        
            
            
            
        