//
// Created by jiyguo on 2022/3/24.
//

#include "dtw.h"
#include "my_dmp.h"
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <fstream>
#include "tinyxml2.h"
#include "robotController/robotController.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;
using namespace tinyxml2;
using namespace myDmp;
using namespace dtw;

typedef vector<int> IVector;
typedef vector<bool> BVector;

const int HZ = 125;
const int N_LIBS=3;
const double DT = 1.0 / HZ;

// trajectory information
const DVector x_0 = {0.52 , -0.22};
const DVector goal = {0.44 , -0.38};
const DVector x_dot_0 = {0.0 , 0.0};
const DVector goal_thresh = {0.002 , 0.002};
const int N_BASES = 15;
DVector taus(N_LIBS,0);

dmp::DMPPoint cur_state;
// callback
void cur_stateCB(const dmp::DMPPoint& point){
    cur_state.positions = point.positions;
    cur_state.velocities = point.velocities;
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
    xml_temps[0] = xml_path + "dmp01.xml";
    xml_temps[1] = xml_path + "dmp11.xml";
    xml_temps[2] = xml_path + "dmp21.xml";
    double cur_t = 0;
    trajs.resize(N_LIBS);
    DVector total_len(N_LIBS,0);
    for (int i=0;i<N_LIBS;++i) {
        if(my_dmps[i]->Init(xml_temps[i]) ){
            my_dmps[i]->get_tau(taus[i]);
            if(!my_dmps[i]->get_plan(trajs[i],total_len[i],cur_t)){
                throw ;
            }
        }else{
            throw ;
        }
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "dmp_classify");
    ros::NodeHandle n;
    ros::Rate rate(HZ);
    // 发布各个dmp隶属度概率
    ros::Publisher probPub = n.advertise<geometry_msgs::PointStamped>("/dmp/probability",10, true);
    ros::Publisher forceDmpPub = n.advertise<std_msgs::Float64MultiArray>("/dmp/force_robot",10, true);
    geometry_msgs::PointStamped prob;
    std_msgs::Float64MultiArray force_msgs;
    // 订阅当前末端期望位置
    ros::Subscriber stateSub = n.subscribe<dmp::DMPPoint>("/shared_control/plan_state",10,cur_stateCB);

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
    // 初始化轨迹分类变量
    IVector templ_counters(N_LIBS,0);
    DVector templ_lens(N_LIBS,0);
    vector<DVector> templ_vecs(N_LIBS);
    BVector at_goals(N_LIBS, false);
    DVector confidances(N_LIBS);
    DVector dtw_dists(N_LIBS,0);
    DVector sig_vec;
    double length_sig = 0;
    // 前一时刻状态初始化为初始状态
    DMPPoint pre_state;
    cur_state.positions = x_0;cur_state.velocities = x_dot_0;
    DVector couple_term;
    vector<DVector> force_dmps(N_LIBS);
    // Online clasify
    while(ros::ok()){
        length_sig += dtw::distance(cur_state.positions, pre_state.positions);
        pre_state = cur_state;
        sig_vec = cur_state.positions;
        for (int i=0;i<N_LIBS;++i) {
            templ_vecs.clear();
            if(!at_goals[i] && find_path(trajs[i],templ_lens[i],templ_counters[i],length_sig,templ_vecs)){
                at_goals[i] = true;
            }
            my_dtws[i]->add_data( templ_vecs , sig_vec);
            my_dmps[i]->get_force(cur_state, couple_term,
                        (double)templ_counters[i]/trajs[i].points.size() * taus[i], force_dmps[i]);
            dtw_dists[i] = my_dtws[i]->get_distance();
        }
        confidances.clear();
        dtw::Confidances(dtw_dists, confidances);
        size_t biggest_idx = std::max_element(confidances.begin(), confidances.end()) - confidances.begin();
        // publish
        force_msgs.data = force_dmps[biggest_idx];
        forceDmpPub.publish(force_msgs);
        prob.header.stamp = ros::Time::now();
        prob.point.x = confidances[0];
        prob.point.y = confidances[1];
        prob.point.z = confidances[2];
        probPub.publish(prob);
        rate.sleep();
    }
    // Delete pointer
    for (int i = 0; i < N_LIBS; ++i) {
        delete my_dmps[i];
        delete my_dtws[i];
    }
    return 0;
}

