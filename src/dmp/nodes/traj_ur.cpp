//
// Created by jiyguo on 2022/4/6.
//

#include <fstream>
#include "string"
#include "sstream"
#include "robotController/robotController.h"

const bool IF_SIM = false;
using namespace std;

typedef vector<double> DVector;

// 宏定义
const int HZ = 125;                 // 频率
const int DIM=2;                    // dimensiona
const double DT = 1.0 / HZ;         // dt
const DVector x_0 = {0.52 , -0.22};

int main(int argc, char **argv){
    ros::init(argc, argv, "traj_ur");
    ros::NodeHandle n;
    ros::Rate rate(HZ);

    string file_name = "/home/jiyguo/ur_ws/src/data/sc/video_test_r.txt";
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
    targetFrame.p.x(x_0[0] );
    targetFrame.p.y(x_0[1] );
    targetFrame.p.z(1.1 );
    myController.moveToFrame(myController.homeJoint,targetFrame,5,true);
    sleep(2);
    string tmp;
    stringstream ss_tmp;
    string single_s;
    while(ros::ok() && getline(my_data,tmp)){
        ss_tmp.str(tmp);
        for (int i = 0; i < 3; ++i) {
            ss_tmp>>single_s;
        }
        single_s.clear();
        ss_tmp>>single_s;
        targetFrame.p.x(stod(single_s));
        single_s.clear();
        ss_tmp>>single_s;
        targetFrame.p.y(stod(single_s));
        tmp.clear();
        ss_tmp.clear();
        single_s.clear();
        myController.moveToFrame(targetFrame,DT, false);
        rate.sleep();
    }
    my_data.close();

    return 0;
}

