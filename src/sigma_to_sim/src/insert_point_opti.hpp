#include <iostream>
#include <string>
#include <vector>
#include "printer.hpp"
#include <fstream>

using namespace std;
class InvertOpti {
public:
    SurgicalRobotUR* m_ur;
    vector<vector<double>> m_invert_points;

    

    //初始化
    InvertOpti();
    
    //读取文件并记录所有离散插入点的位置
    void read_insert_points(const std::string& filename);
    void printPoints();

    //根据插入点、工作空间计算几个指标

    //逆运动学计算
    void inverse_kin();


};

void InvertOpti::read_insert_points(const string& filename) {
    ifstream file(filename);
        if (!file) {
            cerr << "无法打开文件" << endl;
            return;
        }
        
        string line;
        while (getline(file, line)) {
            istringstream iss(line);
            vector<double> point;
            double num;
            while (iss >> num) {
                point.push_back(num);
            }
            m_invert_points.push_back(point);
        }
        file.close();
    
}

void InvertOpti::printPoints() {
        cout << "导入的点的内容：" << endl;
        for (const auto& point : m_invert_points) {
            for (int num : point) {
                cout << num << " ";
            }
            cout << endl;
        }
    }


    int numSegmentsX = length / segmentSize;
    int numSegmentsY = width / segmentSize;
    int numSegmentsZ = height / segmentSize;

    double startX = center[0] - (numSegmentsX * segmentSize) / 2.0;
    double startY = center[1] - (numSegmentsY * segmentSize) / 2.0;
    double startZ = center[2] - (numSegmentsZ * segmentSize) / 2.0;

    for (int i = 0; i < numSegmentsX; i++) {
        for (int j = 0; j < numSegmentsY; j++) {
            for (int k = 0; k < numSegmentsZ; k++) {
                vector<double> vertices(3);
                vertices[0] = startX + (i * segmentSize) + (segmentSize / 2.0);
                vertices[1] = startY + (j * segmentSize) + (segmentSize / 2.0);
                vertices[2] = startZ + (k * segmentSize) + (segmentSize / 2.0);
                boxVertices.push_back(vertices);
            }
        }
    }

}


    
    Eigen::MatrixXd j_RCM(3, 10), j_FULL(9, 10);
    
    Vector p_RCM;
    Eigen::MatrixXd error_RCM(3, 1), error_FULL(9, 1), null_motion(10, 1);
    null_motion << 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0;//零空间运动初始值
    
    Eigen::MatrixXd q_delta_DOUBLE_up(10, 1),q_delta_DOUBLE_down(10, 1),q_delta_DOUBLE(10,1),q_NULL(10,1);

    

    robot_fk_solver.JntToCart(q_origin, frame_t);
    robot_fk_solver.JntToCart(q_origin, frame_7, 6);
    robot_fk_solver.JntToCart(q_origin, frame_8, 7);
    // pri.print_frame(frame_t);
    // pri.print_frame(frame_9);
    
    Vector p_delta_ii;
    p_delta_ii = frame_8.p - frame_7.p;
    // cout << "p_i-p_i+1: " << p_delta_ii.data[0] << " " << p_delta_ii.data[1] << " " << p_delta_ii.data[2] << endl;
    //获取 第8个关节 第九个关节 雅克比
    jacSolver.JntToJac(q_origin, jac_7, 6);
    jacSolver.JntToJac(q_origin, jac_8, 7);
    jacSolver.JntToJac(q_origin, jaco_t);
    
            reach_param_l += get_reach_param_l();
            orient_param_l += get_orient_param_l();
            reach_param_r += get_reach_param_r();
            orient_param_r += get_orient_param_r();
            joint_lim_param_l += get_joint_lim_param(joints_now);
            joint_lim_param_r += get_joint_lim_param(joints_now);
            //各个单个指标求和
        }
        reach_param_l = reach_param_l / num_space;
        orient_param_l = orient_param_l / num_space;
        reach_param_r = reach_param_r / num_space;
        orient_param_r = orient_param_r / num_space;
        joint_lim_param_l += get_joint_lim_param(joints_now);


    

    }