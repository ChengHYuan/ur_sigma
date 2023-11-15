//
// Created by jiyguo on 2022/3/22.
//

#include "dtw.h"
#include "my_dmp.h"
#include "ros/ros.h"
#include "dmp/dmp.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <fstream>

#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;
using namespace myDmp;
using namespace dtw;

const int HZ = 125;

nav_msgs::Path points;
geometry_msgs::PoseStamped this_pose_stamped;
ros::Publisher pathPub;
void Visualize(ros::NodeHandle &n,const DVector& pos)
{
    points.header.stamp = ros::Time::now();
    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.pose.position.x = pos[0];
    this_pose_stamped.pose.position.y = pos[1];
    this_pose_stamped.pose.position.z = 0;
    points.poses.push_back(this_pose_stamped);
    pathPub.publish(points);
}

bool find_path(const DMPTraj& path , double& length,
               int& counter, const double& max_len,
               vector<DVector>& templ_vec) {
    int dims = path.points[counter].positions.size();
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

int main(int argc, char **argv){
    ros::init(argc, argv, "dmp_test");
    ros::NodeHandle n;
    ros::Publisher probPub = n.advertise<geometry_msgs::PointStamped>("dmp/probability",10, true);

    pathPub = n.advertise<nav_msgs::Path>("xy/trajectory4",10, true);
    points.header.frame_id="world";

    this_pose_stamped.header.frame_id = "world";
    geometry_msgs::PointStamped prob;
    ros::Rate rate(HZ);

    ofstream my_data("/home/jiyguo/ur_ws/src/dmp/data/prob2.txt",ios::out);
    string xml_path = "/home/jiyguo/ur_ws/src/dmp/data/";
    // Dmp library
    int num_libs=3;
    vector<string> xml_temps(num_libs);
    xml_temps[0] = xml_path + "dmp01.xml";
    xml_temps[1] = xml_path + "dmp11.xml";
    xml_temps[2] = xml_path + "dmp21.xml";
    // Real time path which simulated by another DMP
    string xml_signal = xml_path + "dmp02.xml";;

    // trajectory information
    DVector x_0; x_0.push_back(0.52); x_0.push_back(-0.22);
    DVector goal; goal.push_back(0.44); goal.push_back(-0.38);
    DVector x_dot_0; x_dot_0.push_back(0.0); x_dot_0.push_back(0.0);
    DVector goal_thresh; goal_thresh.push_back(0.005); goal_thresh.push_back(0.005);
    double seg_length = 20;
    double dt = 1.0 / HZ;
    double cur_t = 0;
    int num_bases = 15;
    vector<DMPTraj> trajs(num_libs);
    DVector total_len(num_libs,0);
    vector<MyDmp*> my_dmps(num_libs);
    vector<Dtw*> my_dtws(num_libs);
    for (int i=0;i<num_libs;++i) {
        my_dmps[i] = new MyDmp( x_0, goal, x_dot_0, goal_thresh, num_bases,dt);
        my_dmps[i]->Init(xml_temps[i]);
        my_dmps[i]->get_plan(trajs[i],total_len[i],cur_t);
        my_dtws[i] = new Dtw(x_0);
    }
    MyDmp* dmp_signal = new MyDmp( x_0, goal, x_dot_0, goal_thresh, num_bases,dt);
    dmp_signal->Init(xml_signal);
    DMPPoint cur_state;cur_state.positions = x_0;cur_state.velocities = x_dot_0;
    DMPPoint plan_state;
    DVector couple_term(3,0);
    double run_time = 0;
    DMPTraj signal_traj;
    clock_t start,end;
    vector<int> templ_counters(num_libs,0);
    DVector templ_lens(num_libs,0);
    double length_sig = 0;
    vector<DVector> templ_vecs(num_libs);
    vector<bool> at_goals(num_libs, false);
    std::vector<vector<DVector>> temps(num_libs);
    DVector confidances(num_libs);
    DVector sig_vec;
    DVector dtw_dists(num_libs,0);
    while(ros::ok() && !dmp_signal->get_step(cur_state,couple_term, run_time, plan_state)){
        start = ros::Time::now().toNSec();
        signal_traj.points.push_back(cur_state);
        signal_traj.times.push_back(run_time);
        run_time += dt;
        length_sig += dtw::distance(cur_state.positions, plan_state.positions);
        cur_state = plan_state;
        sig_vec = plan_state.positions;
        for (int i=0;i<num_libs;++i) {
            templ_vecs.clear();
            if(!at_goals[i] && find_path(trajs[i],templ_lens[i],templ_counters[i],length_sig,templ_vecs)){
                at_goals[i] = true;
                cout<<"~~~~~at_goals~~~~~"<<at_goals[i]<<endl;
            }
            my_dtws[i]->add_data( templ_vecs , sig_vec);
            dtw_dists[i] = my_dtws[i]->get_distance();
        }
//        cout<<dtw_dists[0]<<" "<<dtw_dists[1]<<" "<<dtw_dists[2]<<" "<<endl;
        confidances.clear();
        dtw::Confidances(dtw_dists, confidances);

//        cout<<"~~~~~end~~~~~"<<endl;


        prob.header.stamp = ros::Time::now();
        prob.point.x = confidances[0];
        prob.point.y = confidances[1];
        prob.point.z = confidances[2];
        my_data<<run_time<<" "<<prob.point.x<<" "<<prob.point.y<<" "<<prob.point.z<<endl;
        probPub.publish(prob);
//        cout<<templ_counters[0]<<" "<<templ_counters[1]<<" "<<templ_counters[2]<<" "<<endl;
//        cout<<cur_state.positions[0]<<" , "<<cur_state.positions[1]<<endl;
        rate.sleep();
        end = ros::Time::now().toNSec();

        double endtime=(double)(end-start)/1e6;
        string info  = "Total time:" + to_string(endtime) + "ms";
        ROS_INFO("%s", info.c_str());
    }

    my_data.close();
    return 0;
}