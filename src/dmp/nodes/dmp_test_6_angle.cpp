#include "ros/ros.h"
#include "dmp/dmp.h"
#include "dmp/DMPPointStamp.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include<std_msgs/String.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "tinyxml2.h"
#include <iiwa_msgs/CartesianPose.h>
#include <Eigen/Dense>
#include<string>
#include<cmath>

using namespace std;
using namespace tinyxml2;

#define DMP_SIZE 6

geometry_msgs::PoseStamped origin;
vector<double>start_point = { 0.642,0.06,0.07 };
vector<double>target_point = { start_point[0] + 0.012,start_point[1]-0.012 ,start_point[2]};
// vector<double>target_point = { start_point[0] + 0.06,start_point[1]-0.06 ,start_point[2]};
vector<vector<double>>goals = { {0.732, 0.04},{0.742,-0.01},{0.692,-0.04},{0.662,-0.06} };
//添加目标点的姿态,俯仰角到欧拉角
vector<vector<double>>goals_angle = { {30, 0},{-45,-30},{-60,45},{-40,0} };//先沿x轴旋转，再沿z轴旋转，世界坐标系
vector<vector<double>>goals_euler = { {0,0,0},{0,0,0},{0,0,0},{0,0,0} };
vector<Eigen::Matrix3d> goals_rotate(goals.size());

//欧拉角转四元数
Eigen::Quaterniond eula_to_quaternion(Eigen::Vector3d &eula) {
    
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eula(0), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eula(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eula(2), Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quaternion(rollAngle * pitchAngle * yawAngle);
    return quaternion;
}

//四元数转欧拉角防奇异方法
static void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
}

Eigen::Vector3d applyTransform(double angleX, double angleZ) {
    //构造旋转矩阵
    Eigen::Matrix3d rotationY, rotationX, rotationZ;
    angleX = angleX / 180.0 * M_PI;
    angleZ = angleZ / 180.0 * M_PI;
    rotationY = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
    rotationX = Eigen::AngleAxisd(angleX, Eigen::Vector3d::UnitX());
    rotationZ = Eigen::AngleAxisd(angleZ, Eigen::Vector3d::UnitZ());

    Eigen::Matrix3d transform = rotationZ * rotationX * rotationY;

    Eigen::Vector3d euler = transform.eulerAngles(0, 1, 2);
    return euler;
}

//将所有的旋转角变为欧拉角
void from_angle_to_eula(vector<vector<double>>goals_angle) {
    Eigen::Vector3d data;
    for (int i = 0;i < goals_angle.size();++i) {
        data = applyTransform(goals_angle[i][0], goals_angle[i][1]);
        goals_euler[i][0] = data.x();
        goals_euler[i][1] = data.y();
        goals_euler[i][2] = data.z();
    }
}

void save_traj(dmp::DMPTraj& traj, int index) {//存储dmp点
    ofstream ofs; //定义流对象
    string file_name = "/home/chy/sigma_iiwa_simulation/ur_sigma/path/standard_path" + to_string(index) + ".txt";
    ofs.open(file_name, ios::out);//以写的方式打开文件
    // for (int i = 0;i < traj.points.size();++i) {
    //     // 计算点到起始点的向量
    //     Eigen::Vector3d v1(traj.points[i].positions[0] - start_point[0],
    //                        traj.points[i].positions[1] - start_point[1],
    //                        traj.points[i].positions[2] - start_point[2]);

    //     //向量复原
    //     v1 =  goals_rotate[index].transpose()*v1;
    //     for (int j = 0;j < 3;++j) {
    //         ofs << start_point[j]+v1[j] << " ";
    //     }
    //     for (int j = 3;j < 6;++j) {
    //         ofs << traj.points[i].positions[j] << " ";
    //     }
    //     // ofs << traj.points[i].positions[0] << " "<<traj.points[i].positions[1]<< " "<<traj.points[i].positions[2]<<" "  ;
    //     ofs << endl;
    // }
    for (int i = 0;i < traj.points.size();++i) {
        for (int j = 0;j < 6;++j) {
            ofs << traj.points[i].positions[j] << " ";
        }
        ofs << endl;
    }

    ROS_INFO("path %d is saved!!!!", index);
    ofs.close();
}


void cos_generate(std::vector<dmp::DMPPoint>& points, double time_step) {
    for(int i=0;i<100;i++){
        double t = i*time_step;
        dmp::DMPPoint xpoint;
        xpoint.positions.push_back(cos(0.2*M_PI*t));
        xpoint.velocities.push_back(-0.2*M_PI*sin(0.2*M_PI*t));
        // y axis
        xpoint.positions.push_back(cos(0.5*M_PI*t));
        xpoint.velocities.push_back(-0.1*M_PI*sin(0.5*M_PI*t));
        points.push_back(xpoint);
        // std::cout<< cos(0.2*M_PI*t) <<" , "<< -0.2*M_PI*sin(0.2*M_PI*t) <<std::endl;
    }
}


void trangle_demo_generate(std::vector<dmp::DMPPoint> &points,double time_step){
    for(int i=0;i<100;i++){
        dmp::DMPPoint xpoint;
        xpoint.positions.push_back(0);
        xpoint.velocities.push_back(0.01);
        xpoint.positions.push_back(i*0.05);
        xpoint.velocities.push_back(0.01);
        points.push_back(xpoint);
    }
    for(int i=0;i<50;i++){
        dmp::DMPPoint xpoint;
        xpoint.positions.push_back(i*0.1);
        xpoint.velocities.push_back(0.01);
        xpoint.positions.push_back(5-i*0.05);
        xpoint.velocities.push_back(0.01);
        points.push_back(xpoint);
    }
    for(int i=0;i<25;i++){
        dmp::DMPPoint xpoint;
        xpoint.positions.push_back(5+3*sin(i*M_PI/50));
        xpoint.velocities.push_back(0.01);
        xpoint.positions.push_back(2.5+2*sin(i*M_PI/150));
        xpoint.velocities.push_back(0.01);
        points.push_back(xpoint);
    }
}

void time_generate(std::vector<dmp::DMPPoint> points, std::vector<double>& times, double time_step) {
    for(int i=0;i<points.size();i++){
        times.push_back(i*time_step);
    }
}

//生成peg转移的标准轨迹
void peg_transfer_generate(std::vector<dmp::DMPPoint>& points, double time_step) {
    int num = 2000;
    vector<double>start = start_point;
    vector<double>target = target_point;
    double height = 0.05;
    
    Eigen::Matrix3d rotationY;
    rotationY = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
    Eigen::Matrix3d transform = rotationY;
    Eigen::Vector3d euler = transform.eulerAngles(0, 1, 2);

    vector<double>start_eula = {  euler.x(),euler.y(),euler.z() };
    vector<double>end_eula = goals_euler[0];
    //第一阶段 上升
    // for (int i = 0;i < num*0.2;i++) {
    //     dmp::DMPPoint xpoint;
    //     xpoint.positions.push_back(start[0]);
    //     xpoint.positions.push_back(start[1]);
    //     xpoint.positions.push_back(start[2] + height / (num * 0.2) * i);
    //     xpoint.positions.push_back(eula[0]);
    //     xpoint.positions.push_back(eula[1]);
    //     xpoint.positions.push_back(eula[2]);
    //     xpoint.velocities.push_back(0);
    //     xpoint.velocities.push_back(0);
    //     xpoint.velocities.push_back(0);
    //     xpoint.velocities.push_back(0);
    //     xpoint.velocities.push_back(0);
    //     xpoint.velocities.push_back(0);
    //     points.push_back(xpoint);
    // }
    for(int i=0;i<num*1.4;i++){
        dmp::DMPPoint xpoint;
        xpoint.positions.push_back(start[0]+(target[0]-start[0])/(num*1.4)*i);
        xpoint.positions.push_back(start[1]+(target[1]-start[1])/(num*1.4)*i);
        xpoint.positions.push_back(height);
        xpoint.positions.push_back(start_eula[0]+(end_eula[0]-start_eula[0])/(num*1.4)*i);
        xpoint.positions.push_back(start_eula[1]+(end_eula[1]-start_eula[1])/(num*1.4)*i);
        xpoint.positions.push_back(start_eula[2]+(end_eula[2]-start_eula[2])/(num*1.4)*i);
        xpoint.velocities.push_back(0);
        xpoint.velocities.push_back(0);
        xpoint.velocities.push_back(0);
        xpoint.velocities.push_back(0);
        xpoint.velocities.push_back(0);
        xpoint.velocities.push_back(0);
        points.push_back(xpoint);
    }
    // for (int i = 0;i < num*0.2;i++) {
    //     dmp::DMPPoint xpoint;
    //     xpoint.positions.push_back(target[0]);
    //     xpoint.positions.push_back(target[1]);
    //     xpoint.positions.push_back(height - height / (num * 0.2) * i);
    //     xpoint.positions.push_back(eula[0]);
    //     xpoint.positions.push_back(eula[1]);
    //     xpoint.positions.push_back(eula[2]);
    //     xpoint.velocities.push_back(0);
    //     xpoint.velocities.push_back(0);
    //     xpoint.velocities.push_back(0);
    //     xpoint.velocities.push_back(0);
    //     xpoint.velocities.push_back(0);
    //     xpoint.velocities.push_back(0);
    //     points.push_back(xpoint);
    // }
}

void get_dmp_demo_from_bag(std::vector<dmp::DMPPoint> &points,double &time_step){
    rosbag::Bag demo_bag;
    // demo_bag.open("/home/chy/bagfile/path01.bag", rosbag::bagmode::Read);//以写的模式 打开bag文件的绝对地址
    demo_bag.open("/home/chy/bagfile/end_path_2.bag", rosbag::bagmode::Read);
    ROS_INFO("------------file is loaded------------");
    
    vector<string> demo_topic;
    demo_topic.push_back(string("/iiwa/state/CartesianPose_end"));
    rosbag::View view(demo_bag, rosbag::TopicQuery(demo_topic));//创建一个迭代器，指向bag中的元素
    rosbag::View::iterator it = view.begin();

    vector<double> local_velocity(DMP_SIZE);
    double time_duration;
    geometry_msgs::PoseStamped pose_before;
    geometry_msgs::PoseStamped pose_local;

    for (;it != view.end();++it) {
        auto mes=*it;//获取一帧的内容
        string topic_local = mes.getTopic();
        Eigen::Quaterniond demo_qua;
        Eigen::Vector3d demo_eula_before;
        Eigen::Vector3d demo_eula;
        if (topic_local == "/iiwa/state/CartesianPose_end") {
            iiwa_msgs::CartesianPose::ConstPtr pose_msg = mes.instantiate<iiwa_msgs::CartesianPose>();
            if (pose_msg != NULL) {
                pose_local = pose_msg->poseStamped;
                if (it == view.begin()) {
                    origin = pose_msg->poseStamped;
                }
                dmp::DMPPoint demo;
                demo.positions.push_back(pose_local.pose.position.x);//将x y z 位置发送给dmp demo
                demo.positions.push_back(pose_local.pose.position.y);
                demo.positions.push_back(pose_local.pose.position.z);
                demo_qua.x() = pose_local.pose.orientation.x;
                demo_qua.y() = pose_local.pose.orientation.y;
                demo_qua.z() = pose_local.pose.orientation.z;
                demo_qua.w() = pose_local.pose.orientation.w;
                
                toEulerAngle(demo_qua, demo_eula.x(), demo_eula.y(), demo_eula.z());
                demo.positions.push_back(demo_eula.x());
                demo.positions.push_back(demo_eula.y());
                demo.positions.push_back(demo_eula.z());
                
                if (demo.positions.size() == 0) {
                    pose_before = pose_local;
                    for (int i = 0;i < DMP_SIZE;++i) {
                        local_velocity[i] = 0;
                    }
                }
                else {
                    // time_duration = pose_local.header.stamp.toSec() - pose_before.header.stamp.toSec();//通过差分计算速度
                    time_duration = 0.008;
                    // if (time_step == 0) {
                    //     time_step = time_duration;
                    //     ROS_INFO("time step is %f",time_step);
                    // }
                    // ROS_INFO("duration: %f", time_duration);
                    local_velocity[0] = (pose_local.pose.position.x - pose_before.pose.position.x) / time_duration;
                    local_velocity[1] = (pose_local.pose.position.y - pose_before.pose.position.y) / time_duration;
                    local_velocity[2] = (pose_local.pose.position.z - pose_before.pose.position.z) / time_duration;
                    local_velocity[3] = (demo_eula.x() - demo_eula_before.x()) / time_duration;
                    local_velocity[4] = (demo_eula.y() - demo_eula_before.y()) / time_duration;
                    local_velocity[5] = (demo_eula.z() - demo_eula_before.z()) / time_duration;
                }
                for (int i = 0;i < DMP_SIZE;++i) {
                    // demo.velocities.push_back(local_velocity[i]);
                    demo.velocities.push_back(local_velocity[i]);
                }
                points.push_back(demo);
                // std::cout << "-------" << demo << "---------" << endl;
                pose_before = pose_local;
                demo_eula_before = demo_eula;
                // demo.positions.push_back(pose_msg->poseStamped.pose.position.z);
                // demo.velocities.push_back();
            }
            else{
                ROS_INFO("topic message is empty!!!");
            }
        }

        // if (points.size() / 2 >= 500) {
        //     break;
        // }
    }
    demo_bag.close();
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "dmp_ent");
    ros::NodeHandle n;
    XMLDocument doc;
    doc.LinkEndChild(doc.NewDeclaration("xml version=\"1.0\" encoding=\"UTF-8\""));
    doc.LinkEndChild(doc.NewComment("this is a DMP library data"));
    XMLElement* root = doc.NewElement("DMPs");
    doc.InsertEndChild(root);

    ros::ServiceClient clientLfd = n.serviceClient<dmp::LearnDMPFromDemo>("learn_dmp_from_demo");
    ros::ServiceClient clientPlan = n.serviceClient<dmp::GetDMPPlan>("get_dmp_plan");
    ros::ServiceClient clientActive = n.serviceClient<dmp::SetActiveDMP>("set_active_dmp");


    // ros::Publisher trajPub = n.advertise<dmp::DMPPointStamp>("dmp/trajectory",100);
    // ros::Publisher trajDesPub = n.advertise<dmp::DMPPointStamp>("dmp/des_trajectory",100);
    ros::Publisher trajPub = n.advertise<geometry_msgs::PoseStamped>("dmp/trajectory", 100);
    ros::Publisher trajDesPub = n.advertise<geometry_msgs::PoseStamped>("dmp/des_trajectory", 100);
    ros::Publisher pathPub = n.advertise<nav_msgs::Path>("xy/trajectory", 10, true);
    ros::Publisher pathDesPub = n.advertise<nav_msgs::Path>("xy/des_trajectory", 10, true);
    ros::Publisher iiwa_pub = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose_origin", 1, true);

    nav_msgs::Path path;
    path.header.frame_id = "world";
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header.frame_id = "world";


    Eigen::Quaterniond qua;//经过dmp后的四元数需要归一化
    Eigen::Quaterniond qua_des;
    Eigen::Vector3d eula;
    Eigen::Vector3d eula_des;

    nav_msgs::Path pathDes;
    pathDes.header.frame_id = "world";

    dmp::LearnDMPFromDemo srv;
    std::vector<dmp::DMPPoint> points;
    std::vector<double> times;

    double time_step = 0.008;//125hz

    from_angle_to_eula(goals_angle);

    peg_transfer_generate(points, time_step);
    // trangle_demo_generate(points, time_step);
    // get_dmp_demo_from_bag(points,time_step);
    // cos_generate(points,time_step);//生成轨迹点（cos）
    time_generate(points, times, time_step);//生成时间点

    ROS_INFO("demo points size:  %d", points.size());


    double k_gain = 25;
    // double k_gain = 25;
    // double d_gain = k_gain/4;
    double d_gain = k_gain / 4;
    for (int i = 0;i < DMP_SIZE;++i) {//每个维度设置相同的增益
        srv.request.k_gains.push_back(k_gain);
        srv.request.d_gains.push_back(d_gain);
    }

    srv.request.num_bases = 100;
    srv.request.demo.points = points;
    srv.request.demo.times = times;

    string dim = to_string(srv.request.k_gains.size());
    root->SetAttribute("dim", dim.c_str());


    if (clientLfd.call(srv)) {
        // sleep(5);
        double tau = srv.response.tau;
        std::vector<dmp::DMPData>  dmpList = srv.response.dmp_list;
        for (int i = 0;i < dmpList.size();++i) {
            auto dmpData = dmpList[i];
            XMLElement* dmp_dataElement = doc.NewElement("dmp");
            string id = to_string(i);
            dmp_dataElement->SetAttribute("id", id.c_str());
            XMLElement* kElement = doc.NewElement("kgain");
            kElement->LinkEndChild(doc.NewText(to_string(dmpData.k_gain).c_str()));
            XMLElement* dElement = doc.NewElement("dgain");
            dElement->LinkEndChild(doc.NewText(to_string(dmpData.d_gain).c_str()));
            dmp_dataElement->SetAttribute("id", id.c_str());
            dmp_dataElement->LinkEndChild(kElement);
            dmp_dataElement->LinkEndChild(dElement);
            XMLElement* weightsElement = doc.NewElement("weights");
            weightsElement->SetAttribute("nums", to_string(dmpData.weights.size()).c_str());
            for (int j = 0;j < dmpData.weights.size();++j) {
                XMLElement* weightElement = doc.NewElement("w");
                weightElement->SetAttribute("id", to_string(j).c_str());
                weightElement->LinkEndChild(doc.NewText(to_string(dmpData.weights[j]).c_str()));
                weightsElement->InsertEndChild(weightElement);
            }
            dmp_dataElement->LinkEndChild(weightsElement);
            root->LinkEndChild(dmp_dataElement);
        }
        XMLPrinter printer;
        // doc.Print(&printer);
        std::cout << printer.CStr() << endl;
        doc.SaveFile("myXML.xml");


        dmp::SetActiveDMP srvSetActive;
        dmp::DMPTraj traj;
        srvSetActive.request.dmp_list = dmpList;
        vector<double> goal_indexes = { 0, 1,2,3 };
        vector<vector<double>> goal_box;


        Eigen::Vector3d goal_vector;
        for (int i = 0;i < goal_indexes.size();++i) {
            //计算初始目标向量和当前目标向量之间的夹角和旋转矩阵
            Eigen::Vector2d origin_goal(target_point[0] - start_point[0], target_point[1] - start_point[1]);
            Eigen::Vector2d local_goal(goals[i][0] - start_point[0], goals[i][1] - start_point[1]);
            double theta = acos(origin_goal.dot(local_goal) / (origin_goal.norm() * local_goal.norm()));
            if (origin_goal[0] * local_goal[1] - origin_goal[1] * local_goal[0] < 0) { theta = -theta; }

            // cout << "theta: " << theta << endl;
            Eigen::AngleAxisd rotate_vector(-theta, Eigen::Vector3d(0, 0, 1));
            goals_rotate[i] = rotate_vector.matrix();
            goal_vector.x() = goals[i][0] - start_point[0];
            goal_vector.y() = goals[i][1] - start_point[1];
            goal_vector.z()=0.0;

            //计算旋转后的目标向量
            goal_vector = goals_rotate[i] * goal_vector;
            // cout << "goal_vector: "<<goal_vector << endl;
            //计算旋转后的目标位置，并且存储
            goal_box.push_back({ start_point[0] + goal_vector.x(),start_point[1] + goal_vector.y(), start_point[2] });
            // goal_box.push_back({ goals[i][0],goals[i][1], start_point[2] });

        }
        // cout << "goal box: " << goal_box[0][0] << " " << goal_box[0][1] << " " << goal_box[0][2] << endl;
        // cout << "goal box: " << goal_box[1][0] << " " << goal_box[1][1] << " " << goal_box[1][2] << endl;
        // cout << "goal box: " << goal_box[2][0] << " " << goal_box[2][1] << " " << goal_box[2][2] << endl;
        // cout << "goal box: " << goal_box[3][0] << " " << goal_box[3][1] << " " << goal_box[3][2] << endl;

        for (int goal_index = 0;goal_index < goal_box.size();++goal_index) {
        // int goal_index = 1;
        if (clientActive.call(srvSetActive)) {
                dmp::GetDMPPlan srvPlan;
                for (int i = 0;i < 3;++i) srvPlan.request.x_0.push_back(start_point[i]);//设置起始点位置
                for (int i = 3;i < 6;++i) srvPlan.request.x_0.push_back(points[0].positions[i]);//设置起始点位置

                for (int i = 0;i < DMP_SIZE;++i) srvPlan.request.x_dot_0.push_back(0);//设置起始点速度
                srvPlan.request.t_0 = 0;
                for (int i = 0;i < 3;++i)srvPlan.request.goal.push_back(goal_box[goal_index][i]);//设置终止点位置
                for (int i = 3;i < 6;++i)srvPlan.request.goal.push_back(goals_euler[goal_index][i-3]);//设置终止点位置

                for (int i = 0;i < DMP_SIZE;++i) srvPlan.request.goal_thresh.push_back(0.001);//设置终止条件
                // srvPlan.request.seg_length = 20;
                srvPlan.request.seg_length = -1;
                srvPlan.request.tau = tau;
                srvPlan.request.dt = 0.01;
                srvPlan.request.integrate_iter = 1;
                if (clientPlan.call(srvPlan) && srvPlan.response.at_goal) {
                    traj = srvPlan.response.plan;
                    ROS_INFO("DMP traj size:    %d", traj.points.size());
                }
                else {
                    ROS_INFO("clientPlan services call failed ! ");
                }
            }
            ROS_INFO("DMP services call successful ! ");
            //        cout<<traj.points.size()<<endl;


            //将旋转后的轨迹点复原
            for (int i = 0;i < traj.points.size();++i) {
                // 计算点到起始点的向量
                Eigen::Vector3d v1(traj.points[i].positions[0] - start_point[0],
                    traj.points[i].positions[1] - start_point[1],
                    traj.points[i].positions[2] - start_point[2]);

                //向量复原
                v1 = goals_rotate[goal_index].transpose() * v1;
                for (int j = 0;j < 3;++j) {
                    traj.points[i].positions[j] = start_point[j] + v1[j];
                }
            }


            save_traj(traj, goal_index);
        }




        geometry_msgs::PoseStamped cur;
        geometry_msgs::PoseStamped des;
        ros::Rate rate(125);
        //    while(ros::ok()){
        for (int i = 0; i < traj.points.size(); ++i) {

            // cout<<"---------------"<<traj.times[i]<<"---------------"<<endl;
            cur.header.stamp = ros::Time::now();
            cur.pose.position.x = traj.points[i].positions[0];
            cur.pose.position.y = traj.points[i].positions[1];
            cur.pose.position.z = traj.points[i].positions[2];
            eula.x() = traj.points[i].positions[3];
            eula.y() = traj.points[i].positions[4];
            eula.z() = traj.points[i].positions[5];

            qua = eula_to_quaternion(eula);

            cur.pose.orientation.x = qua.x();
            cur.pose.orientation.y = qua.y();
            cur.pose.orientation.z = qua.z();
            cur.pose.orientation.w = qua.w();

            // ROS_INFO("quat: %f", cur.pose.orientation.x * cur.pose.orientation.x +
            //                    cur.pose.orientation.y * cur.pose.orientation.y +
            //                    cur.pose.orientation.z * cur.pose.orientation.z +
            //                    cur.pose.orientation.w * cur.pose.orientation.w);
            trajPub.publish(cur);
            iiwa_pub.publish(cur);

            if (i < points.size()) {
                double time = i * 0.01;
                des.header.stamp = ros::Time::now();
                des.pose.position.x = points[i].positions[0];
                des.pose.position.y = points[i].positions[1];
                des.pose.position.z = points[i].positions[2];

                eula_des.x() = points[i].positions[3];
                eula_des.y() = points[i].positions[4];
                eula_des.z() = points[i].positions[5];
                qua_des = eula_to_quaternion(eula_des);

                des.pose.orientation.x = qua_des.x();
                des.pose.orientation.y = qua_des.y();;
                des.pose.orientation.z = qua_des.z();;
                des.pose.orientation.w = qua_des.w();;
                trajDesPub.publish(des);
            }
            // path.header.stamp = ros::Time::now();
            // this_pose_stamped.header.stamp = ros::Time::now();
            // this_pose_stamped.pose.position.x = traj.points[i].positions[0];
            // this_pose_stamped.pose.position.y = traj.points[i].positions[1];
            // this_pose_stamped.pose.position.z = origin.pose.position.z;
            // this_pose_stamped.pose.orientation = origin.pose.orientation;
            // path.poses.push_back(this_pose_stamped);
            // pathPub.publish(path);
            // iiwa_pub.publish(this_pose_stamped);

            // pathDes.header.stamp = ros::Time::now();
            // this_pose_stamped.pose.position.x = cos(0.2*M_PI*time);
            // this_pose_stamped.pose.position.y = cos(0.5*M_PI*time);
            // this_pose_stamped.pose.position.z = 0;
            // pathDes.poses.push_back(this_pose_stamped);
            // pathDesPub.publish(pathDes);
            // iiwa_pub.publish(this_pose_stamped);
            rate.sleep();
        }
        ROS_INFO("DMP path end!!");
        //    }
    }
    else {
        ROS_INFO("Failled call DMP services! ");
    }
    return 0;
}