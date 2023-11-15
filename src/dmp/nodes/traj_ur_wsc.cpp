//
// Created by jiyguo on 2022/4/7.
//

//
// Created by jiyguo on 2022/4/6.
//

#include <fstream>
#include "string"
#include "sstream"
#include "robotController/robotController.h"
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


const bool IF_SIM = false;
using namespace std;

typedef vector<double> DVector;
int ma_id=0;

// 宏定义
const int HZ = 125;                 // 频率
const int DIM=2;                    // dimensiona
const double DT = 1.0 / HZ;         // dt
const DVector x_0 = {0.78-0.03 , -0.2};
const DVector goal = {0.85-0.08 , -0.38};

ros::Publisher markerPub;
ros::Publisher markerPath_rawPub;

void visualize(const string& file_name,char color);

void marker_pub(const DVector& place);
void marker_array_arrow(const DVector& place,const DVector& ori,visualization_msgs::MarkerArray& marker_array);
void marker_line(const DVector& place,visualization_msgs::Marker& marker,int idx,char color);
int main(int argc, char **argv){
    ros::init(argc, argv, "traj_ur");
    ros::NodeHandle n;
    ros::Rate rate(HZ);
    markerPub = n.advertise<visualization_msgs::Marker>("dmp/target_marker",10, true);
    ros::Publisher markerPathPub = n.advertise<visualization_msgs::Marker>("dmp/markerPath",10, true);
    markerPath_rawPub = n.advertise<visualization_msgs::Marker>("dmp/markerPath_raw",10, true);
    ros::Publisher marker_arrowPub = n.advertise<visualization_msgs::MarkerArray>("dmp/marker_array_arrow",10, true);
    marker_pub(goal);

    visualization_msgs::MarkerArray ma;

//    string file_name = "/home/jiyguo/ur_ws/src/dmp/data/my_dmp_ws.txt";
    visualize("/home/jiyguo/ur_ws/src/dmp/data/my_dmp_ws_tmp.txt", 'g');

    string file_name = "/home/jiyguo/ur_ws/src/dmp/data/my_dmp_ws.txt";
    std::ifstream my_data(file_name,ios::in);
    if (!my_data.is_open())
    {
        cout<<file_name<<endl;
        cout << "Can not open this file" << endl;
        return -1;
    }

    robotController myController(&n,IF_SIM);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration(1).sleep();

    KDL::Frame targetFrame;
    targetFrame.M = KDL::Rotation::RotX(M_PI);
    targetFrame.p.z(1.1 );
    bool flag = true;
    string tmp;
    stringstream ss_tmp;
    string single_s;
    DVector pos(2);
    DVector vec(2);
    visualization_msgs::Marker marker;
    int count=0;
    while(ros::ok() && getline(my_data,tmp)){
        count++;
        ss_tmp.str(tmp);
        single_s.clear();
        ss_tmp>>single_s;
        pos[0] = -stod(single_s)-0.08;
        targetFrame.p.x(pos[0]);
        single_s.clear();
        ss_tmp>>single_s;
        pos[1] = stod(single_s);
        targetFrame.p.y(pos[1]);
        single_s.clear();
        ss_tmp>>single_s;
        vec[0] = stod(single_s);
        single_s.clear();
        ss_tmp>>single_s;
        vec[1] = stod(single_s);
        tmp.clear();
        ss_tmp.clear();
        single_s.clear();
        if(flag){
            myController.moveToFrame(myController.homeJoint,targetFrame,6, true);
            sleep(3);
            flag = false;
        }else{
            myController.moveToFrame(targetFrame,DT, false);
        }
        myController.getRobotFrame(targetFrame);
        pos[0] = targetFrame.p.x();
        pos[1] = targetFrame.p.y();
        if(count%10==0){
            marker_array_arrow(pos,vec,ma);
        }
        marker_line( pos,marker,count,'r');
        marker_arrowPub.publish(ma);
        markerPathPub.publish(marker);
        rate.sleep();
    }
    my_data.close();

    return 0;
}

void visualize(const string& file_name,char color)
{
    visualization_msgs::Marker marker;
    std::ifstream raw_data(file_name,ios::in);
    if (!raw_data.is_open()) {
        cout<<file_name<<endl;
        cout << "Can not open this file" << endl;
        return;
    }
    int count=100000;
    string tmp;
    stringstream ss_tmp;
    string single_s;
    DVector pos(2);
    DVector vec(2);
    while(ros::ok()&&getline(raw_data,tmp)){
        ss_tmp.str(tmp);
        single_s.clear();
        ss_tmp>>single_s;
        pos[0] = -stod(single_s)-0.08;
        single_s.clear();
        ss_tmp>>single_s;
        pos[1] = stod(single_s);
        tmp.clear();
        ss_tmp.clear();
        single_s.clear();
        marker_line( pos,marker,count,color);
        markerPath_rawPub.publish(marker);

    }
}


void marker_array_arrow(const DVector& place,const DVector& ori,visualization_msgs::MarkerArray& marker_array){
    if(abs(ori[1])<=0.000001||abs(ori[0])<=0.000001){
        return;
    }
    visualization_msgs::Marker marker;
    marker.id = ++ma_id;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = place[0];
    marker.pose.position.y = place[1];
    marker.pose.position.z = 1.1;
    KDL::Rotation R;
//    cout<<atan(ori[0]/ori[1])/M_PI*180<<endl;
    R.DoRotZ(-M_PI/2+atan(ori[0]/ori[1]));
    double x,y,z,w;
    R.GetQuaternion(x,y,z,w);
    marker.pose.orientation.x = x;
    marker.pose.orientation.y = y;
    marker.pose.orientation.z = z;
    marker.pose.orientation.w = w;
    double size = sqrt(pow(ori[0],2)+ pow(ori[1],2));
    marker.scale.x = 0.02*size;
    marker.scale.y = 0.0015;
    marker.scale.z = 0.0015;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    marker_array.markers.push_back(marker);
}

void marker_line(const DVector& place,visualization_msgs::Marker& marker,int idx,char color){
    geometry_msgs::Point p;
    marker.id = idx;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    p.x = place[0];
    p.y = place[1];
    p.z = 1.1;
    marker.scale.x = 0.003;
//    marker.scale.y = 0.001;
//    marker.scale.z = 0.001;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    if(color=='g'){
        marker.color.g = 1.0;
    }else if(color=='r'){
        marker.color.r = 1.0;
    }else{
        marker.color.b = 1.0;
    }
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    marker.points.push_back(p);
}

void marker_pub(const DVector& place){
    visualization_msgs::Marker marker;
    marker.id = 0;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = place[0];
    marker.pose.position.y = place[1];
    marker.pose.position.z = 1.1;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    markerPub.publish(marker);
}
