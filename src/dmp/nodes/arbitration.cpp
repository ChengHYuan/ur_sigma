
#include "ros/ros.h"
#include <fstream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include "std_msgs/Float64MultiArray.h"
#include "dmp/DMPPoint.h"
#include "robotController/robotController.h"
#include "Eigen/Core"
#include "dtw.h"
#include "my_dmp.h"
#include "tinyxml2.h"
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include "myATI.h"

const bool IF_SIM = false;

using namespace std;
using namespace tinyxml2;
using namespace myDmp;
using namespace dtw;

typedef vector<int> IVector;

// 宏定义
const int N_BASES = 15;             // 高斯核函数数量
const int HZ = 125;                 // 频率
const int N_LIBS=3;                 // 技能库数量
const int DIM=2;                    // dimensions
const double DT = 1.0 / HZ;         // dt
const double F_M = 20;              // 最大力
// Admittance gain
//const DVector M_D = {300.0,300.0};
const DVector M_D = {100.0,100.0}; // 惯性参数

DVector C_D(DIM,0);  // 阻尼参数
// trajectory information

//const DVector x_0 = {0.52 , -0.22};
//const DVector goal = {0.50277 , -0.41711};

const DVector x_0 = {0.52 , -0.22};
const DVector goal = {0.44 , -0.38};

//const DVector x_0 = {0.16264 , -0.1453};
//const DVector goal = {0.14541 , -0.34241};

const DVector x_dot_0 = {0.0 , 0.0};
const DVector goal_thresh = {0.005 , 0.005};

typedef Eigen::Matrix<double,DIM,DIM> DMatrix;

DVector force_total(DIM,0);
DVector force_robot(DIM,0);
DVector force_human(DIM,0);
DVector taus(N_LIBS,0);
//DVector prob(N_LIBS,0);
dmp::DMPPoint cur_state;
// callback
//void cur_stateCB(const dmp::DMPPoint& point){
//    cur_state.positions = point.positions;
//    cur_state.velocities = point.velocities;
//}

nav_msgs::Path points;
geometry_msgs::PoseStamped this_pose_stamped;
// 可视化
void marker_line(const DVector& place,visualization_msgs::Marker& marker,int idx,char color);
void Visualize(ros::Publisher &pub, double* pos);
// 力传感器读数callback
void force_humanCB(const std_msgs::Float64MultiArray::ConstPtr& force_);
// 根据当前轨迹，查找也符合路径长度约束的dmp库中的轨迹段
bool find_path(const DMPTraj& path , double& length,
               int& counter, const double& max_len,
               vector<DVector>& templ_vec);
// 生成dmp库里的参考轨迹
void gen_path(const vector<MyDmp*>& my_dmps , vector<DMPTraj>& trajs);
// 人类置信度
double confi_h(const DVector& force_);
// 仲裁函数
double arbitration_fun(const double& c_h,const double& c_r);

// 查找参考轨迹中距离当前位置最近的点
int find_local_nearest(const DMPPoint& curState,int templ_counter,const DMPTraj& traj,int last){
    int index = 0;
    double m_dist = 1000;
    double dist;
    int start = max(templ_counter-125,last);
    int end = min(templ_counter+125,(int)traj.times.size());
    for (int i = start; i < end; ++i) {
        dist = dtw::distance(curState.positions,traj.points[i].positions);
        if(dist<m_dist){
            index = i;
            m_dist = dist;
        }
    }
    return index;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "arbitration");
    ros::NodeHandle n;
    ros::Rate rate(HZ);

    vector<ros::Publisher> dmp_paths_pub;
    ros::Publisher dmp_path_pub;

    geometry_msgs::PointStamped prob;
    // 可视化
    ros::Publisher dmp_path_online = n.advertise<visualization_msgs::Marker>("/shared_control/markerPath",10, true);
    dmp_path_pub = n.advertise<visualization_msgs::Marker>("dmp/markerPath1",10, true);
    dmp_paths_pub.push_back(dmp_path_pub);
    dmp_path_pub = n.advertise<visualization_msgs::Marker>("dmp/markerPath2",10, true);
    dmp_paths_pub.push_back(dmp_path_pub);
    dmp_path_pub = n.advertise<visualization_msgs::Marker>("dmp/markerPath3",10, true);
    dmp_paths_pub.push_back(dmp_path_pub);

    // 数据输出的路径和文件名
    string argument;
    if(argc>1){
        argument = argv[1];
    }
//    string file_name = "/home/jiyguo/ur_ws/src/data/ad/"+ argument +".txt";
    string file_name = "/home/jiyguo/ur_ws/src/data/sc/"+ argument +".txt";
    std::ofstream my_data(file_name,ios::trunc);
    if (!my_data.is_open())
    {
        cout<<file_name<<endl;
        cout << "Can not open this file" << endl;
        return -1;
    }

    points.header.frame_id="world";
    this_pose_stamped.header.frame_id = "world";

    robotController myController(&n,IF_SIM);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration(1).sleep();

    KDL::Frame frame;
    KDL::Wrench wrench;
    KDL::Rotation rot = KDL::Rotation::RotZ(-M_PI/4.0);
    ForceSensor ft_sensor;
    ft_sensor.init();
//    while(ros::ok()){
//        myController.getRobotFrame(frame);
//        frame.M = frame.M * rot;
//        wrench = ft_sensor.getCompsenCartForce(frame);
//        for (int i = 0; i < 3; ++i) {
//            std::cout<<wrench.force.data[i]<<" , ";
//        }
//        std::cout<<endl;
//    }

    // 参考轨迹总时间
    for (int i = 0; i < N_LIBS; ++i) {
        taus[i] = 10;
    }
    // 创建dmp和dtw对象
    vector<MyDmp*> my_dmps(N_LIBS);
    vector<Dtw*> my_dtws(N_LIBS);
    for (int i = 0; i < N_LIBS; ++i) {
        my_dmps[i] = new MyDmp( x_0, goal, x_dot_0, goal_thresh, N_BASES, DT);
        my_dtws[i] = new Dtw(x_0);
    }
    // 生成库中轨迹
    vector<DMPTraj> trajs(N_LIBS);
    gen_path(my_dmps , trajs);
    KDL::Frame tmp_frame;
    myController.getRobotFrame(tmp_frame);
    string color = "rgb";
    for (int j = 0; j < N_LIBS; ++j) {
        string dmps_file = "/home/jiyguo/ur_ws/src/data/dmp"+ to_string(j)+"_tmp.txt";
        std::ofstream mydata_dmps(dmps_file,ios::trunc);
        visualization_msgs::Marker marker;
        for(int i=0;i<trajs[j].times.size();++i){
            tmp_frame.p.x(trajs[j].points[i].positions[0]);
            tmp_frame.p.y(trajs[j].points[i].positions[1]);
            mydata_dmps<<tmp_frame.p.x()<<" "<<tmp_frame.p.y()<<endl;
            DVector tmp = {tmp_frame.p.x(),tmp_frame.p.y()};
            marker_line(tmp, marker, j ,color[j]);
            }
        dmp_paths_pub[j].publish(marker);
        mydata_dmps.close();
        points.poses.clear();
    }

    /*
     * 初始化变量
     * */
    IVector templ_counters(N_LIBS,0);
    DVector templ_lens(N_LIBS,0);
    vector<DVector> templ_vecs(N_LIBS);
    bool at_goal = false;
    DVector confidances(N_LIBS);
    DVector dtw_dists(N_LIBS,0);
    DVector dgain(DIM,0);
    DVector sig_vec;
    double length_sig = 0;
    double c_h = 0;
    double c_r = 0;
    double alpha = 0;

    // 前一时刻状态初始化为初始状态
    DMPPoint pre_state;
    pre_state.positions = x_0;
    pre_state.velocities = x_dot_0;
    DVector couple_term(DIM,0);
    DVector force_dmp;
    KDL::Frame cur_frame;
    KDL::Twist cur_twist;
    KDL::Frame targetFrame;
    targetFrame.M = KDL::Rotation::RotX(M_PI);
    targetFrame.p.x(x_0[0] );
    targetFrame.p.y(x_0[1] );
    targetFrame.p.z(1.3 );
    myController.moveToFrame(myController.homeJoint,targetFrame,5,true); //走到初始位置
    sleep(3);
    DVector xdd(DIM);
    dmp::DMPPoint plan_state;
    plan_state.positions.resize(DIM);
    plan_state.velocities.resize(DIM);
    cur_state.positions.resize(DIM);
    cur_state.velocities.resize(DIM);
    plan_state.positions = x_0;
    double cur_t=0;
    int timer_count=0;
    bool flag = false;
    int counter = 0;
    visualization_msgs::Marker marker;
    // Online
    while(ros::ok() && !at_goal && counter<=20.0/DT ){

        myController.getRobotFrame(frame);
        frame.M = frame.M * rot;
        wrench = ft_sensor.getCompsenCartForce(frame);
        for (int i = 0; i < DIM; ++i) {
            force_human[i] = wrench.force.data[i];
        }
//        std::cout<<endl;
//        // 人类力输入仿真测试
//        if( cur_t<=2 ){
//            force_human[0] = 5;
//            force_human[1] = -5;
//        }else{
//            force_human[0] = 0;
//            force_human[1] = 0;
//        }

        // get currunt ee frame & twist
        myController.getRobotFrame(cur_frame);
        myController.getRobotVelocity(cur_twist);
        for (int i = 0; i < DIM; ++i) {
            cur_state.positions[i] = cur_frame.p.data[i];
            cur_state.velocities[i]= cur_twist.vel.data[i];
        }

        length_sig += dtw::distance(cur_state.positions, pre_state.positions);// 对在线生成轨迹积分

        pre_state = cur_state;
        sig_vec = cur_state.positions;
        for (int i=0;i<N_LIBS;++i) {
            templ_vecs.clear();
            find_path(trajs[i],templ_lens[i],templ_counters[i],length_sig,templ_vecs);
            my_dtws[i]->add_data( templ_vecs , sig_vec);
            dtw_dists[i] = my_dtws[i]->get_distance();
            my_data<<dtw_dists[i]<<" ";
        }
        my_data<<sig_vec[0]<<" "<<sig_vec[1]<<" ";
        confidances.clear();
        dtw::Confidances(dtw_dists, confidances);
        prob.point.x = confidances[0];
        prob.point.y = confidances[1];
        prob.point.z = confidances[2];
        my_data<<prob.point.x<<" "<<prob.point.y<<" "<<prob.point.z<<" ";

        // 分类的dmp编号
        size_t idx = std::max_element(confidances.begin(), confidances.end()) - confidances.begin();
        my_dmps[idx]->get_dgain(dgain);

//        at_goal = my_dmps[idx]->get_force(cur_state, couple_term,
//                      (double)templ_counters[idx]/trajs[idx].points.size() * taus[idx], force_dmp);
//        cur_t = (double)(templ_counters[idx]+1)/trajs[idx].points.size() * taus[idx];

        // 找到最近的点，以此作为当前时刻的变量
        timer_count = find_local_nearest(cur_state,templ_counters[idx],trajs[idx],timer_count);
        cur_t = (double)(timer_count+1)/trajs[idx].points.size() * taus[idx];
        ROS_INFO("cur_t: %f",cur_t);
        at_goal = my_dmps[idx]->get_force(cur_state, couple_term,cur_t, force_dmp);

        my_data<<force_human[0]<<" "<<force_human[1]<<" ";
        my_data<<force_dmp[0]<<" "<<force_dmp[1]<<" ";

        // 置信度计算与仲裁
        c_h = confi_h(force_human);
        sort(confidances.begin(),confidances.end());
        c_r = confidances[2] - confidances[1];
        alpha = arbitration_fun(c_h,c_r);
        my_data<<c_h<<" "<<c_r<<" "<<alpha<<" ";
        // 更新状态变量
        for (int i = 0; i < DIM; ++i) {
            force_robot[i] = force_dmp[i] * M_D[i];
            force_total[i] = alpha * force_robot[i] + (1-alpha) * force_human[i];
            C_D[i] = M_D[i] * dgain[i] / taus[idx];
            xdd[i] = (force_total[i]-C_D[i]*plan_state.velocities[i]) / M_D[i];
            plan_state.velocities[i] = plan_state.velocities[i] + xdd[i] * DT;
            plan_state.positions[i] = plan_state.positions[i] + plan_state.velocities[i] * DT;
            targetFrame.p.data[i] = plan_state.positions[i];
        }
        my_data<<force_total[0]<<" "<<force_total[1]<<" "<<endl;
        myController.moveToFrame(targetFrame,DT, false); //机器人运动
        DVector tmp = {targetFrame.p.x(),targetFrame.p.y()};

        //可视化
        marker_line(tmp, marker, counter++ ,'k');
        if(flag){
            dmp_path_online.publish(marker);
        }else{
            flag = true;
        }

        rate.sleep();
    }
    my_data.close();

    return 0;
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
    p.z = 1.3;
    marker.scale.x = 0.003;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    if(color=='g'){
        marker.color.g = 1.0;
    }else if(color=='r'){
        marker.color.r = 1.0;
    }else if(color=='b'){
        marker.color.b = 1.0;
    }else{
//        marker.color.r = 1.0;
//        marker.color.g = 1.0;
//        marker.color.b = 1.0;
    }
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    marker.points.push_back(p);
}


void Visualize(ros::Publisher &pub, double* pos)
{
    points.header.stamp = ros::Time::now();
    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.pose.position.x = pos[0];
    this_pose_stamped.pose.position.y = pos[1];
    this_pose_stamped.pose.position.z = pos[2];

    points.poses.push_back(this_pose_stamped);
    pub.publish(points);
}

// force_human callback
void force_humanCB(const std_msgs::Float64MultiArray::ConstPtr& force_){
    for (int i = 0; i < DIM; ++i) {
        force_human[i] = force_->data[i];
    }
}

bool find_path(const DMPTraj& path , double& length,
               int& counter, const double& max_len,
               vector<DVector>& templ_vec) {
    size_t dims = path.points[counter].positions.size();
    DVector cur_pos(dims,0);
    DVector next_pos(dims,0);
    cur_pos = path.points[counter].positions;
    while (length <= max_len) {
        if(counter>=path.points.size()-1){
            return true;
        }
        next_pos = path.points[++counter].positions;
        length += dtw::distance(cur_pos,next_pos);
        templ_vec.push_back(next_pos);
        cur_pos = next_pos;
    }
    return false;
}

void gen_path(const vector<MyDmp*>& my_dmps , vector<DMPTraj>& trajs){
    string xml_path = "/home/jiyguo/ur_ws/src/dmp/data/";
    // Dmp library
    vector<string> xml_temps(N_LIBS);
    xml_temps[1] = xml_path + "dmp11.xml";
    xml_temps[0] = xml_path + "dmp01.xml";
    xml_temps[2] = xml_path + "dmp21.xml";
    double cur_t = 0;
    trajs.resize(N_LIBS);
    DVector total_len(N_LIBS,0);
    for (int i=0;i<N_LIBS;++i) {
        if(my_dmps[i]->Init(xml_temps[i]) ){
            my_dmps[i]->set_tau(taus[i]);
            if(!my_dmps[i]->get_plan(trajs[i],total_len[i],cur_t)){
                throw ;
            }
        }else{
            throw ;
        }
    }
}

// human confidence
double confi_h(const DVector& force_){
    double f = 0;
    for (int i = 0; i < force_.size(); ++i) {
        f += pow(force_[i],2);
    }
    f = sqrt(f);
    double lamda = 2* log(99)/F_M;
    double sigma = lamda * F_M / 2;
    return 1.0/(1+ exp(-lamda*f+sigma));
}

double arbitration_fun(const double& c_h,const double& c_r){
    double gamma = 1;
    double pesi = 0.01;
    double delta = 1;
    double para = -gamma* ( 2*(c_h+20*pesi) / (c_r+pesi) - delta );
    double alpha = 1-1/(1+ exp(para));
    return 1.5466*alpha;
}
