#include <iostream>
#include <string>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include "surgical_robot_ur.hpp"
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "surgical_robot_pub_sub.hpp"
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

    //根据工作空间大小分割出一系列点（腹腔镜手术训练场景）
    vector<vector<double>> getBoxVertices(const vector<double>& center, double length, double width, double height, double segmentSize);

    //根据插入点、工作空间计算几个指标

    //逆运动学计算
    void inverse_kin();


    //指标计算（根据当前关节角计算）
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

vector<vector<double>> InvertOpti::getBoxVertices(const vector<double>& center, double length, double width, double height, double segmentSize) {
    vector<vector<double>> boxVertices;

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

    return boxVertices;
}

void InvertOpti::inverse_kin(Frame& frame_in, JntArray& q_in) {
    lamda = get_lamda(q_in);
    while (step < 100) {
        q_in = invOneStep(frame_in, q_in, stop_flag, lamda, over1_flag);

        if (stop_flag) {
            break;
        }
        step++;
        if (step == 100) {
            ROS_INFO("iterate step is out of range");
        }

    }

}

double get_lamda(JntArray& q_origin) {
    double lamda;
    Vector p_delta_ii;
    //获取 第8个关节 第九个关节 坐标系
    robot_fk_solver.JntToCart(q_origin, frame_7, 6);
    robot_fk_solver.JntToCart(q_origin, frame_8, 7);
    //根据坐标系原点差值、RCM点位置 计算 λ 
    p_delta_ii = frame_7.p - frame_8.p;
    
    // cout << "p_i-p_i+1: " << p_delta_ii.data[0] << " " << p_delta_ii.data[1] << " " << p_delta_ii.data[2] << endl;
    
    Vector p_delta_ic(frame_7.p.x() - p_c(0), frame_7.p.y() - p_c(1), frame_7.p.z() - p_c(2));
    lamda = p_delta_ic.Norm() / p_delta_ii.Norm();
    // cout << "lamda: " << lamda << endl;

    return lamda;
}

JntArray invOneStep(Frame& frame_t_d, JntArray& q_origin, bool& stop_flag, double& lamda,bool &over1_flag) {
    //设置第二道线，当可操作性值过小时，直接让角度归零
    bool over2_flag = false;
    
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
    
    //计算 RCM处 的雅可比矩阵
    jac_7_double = jac_7.data;
    
    jac_8_double = jac_8.data;

    

    j_RCM.block(0, 0, 3, 9) = (jac_7_double + lamda * (jac_8_double - jac_7_double)).block(0, 0, 3, 9);
    j_RCM.block(0, 9, 3, 1) << p_delta_ii.x(), p_delta_ii.y(), p_delta_ii.z();
    // ROS_INFO("flag!");

    
    //计算总的雅可比矩阵
    j_FULL.block(0, 0, 6, 9) = jaco_t.data;
    j_FULL.block(0, 9, 6, 1) << 0, 0, 0, 0, 0, 0;
    j_FULL.block(6, 0, 3, 10) = j_RCM;
    

    
    //检测约束雅克比可操作性是否超出范围 over_flag将变为true，此时认为本次迭代失败
    double manipula = sqrt((j_FULL * j_FULL.transpose()).determinant());
    // cout << "----------manipula: " << manipula << "----------" << endl;
    //设置第一道线，当可操作性小于一个值时，不发送角度
    if (manipula < 0.0005 && manipula > 0.0001) {
        over1_flag = true;
        stop_flag = true;
        ROS_WARN("robot is near to singular pose, manipula is : %f", manipula);
    }
    
    //设置第二道线，过小时直接回到初始值
    if (manipula < 0.00005) {
        over2_flag = true;
        ROS_WARN("robot is over singular pose, manipula is : %f", manipula);
    }

    over1_flag = false;

    // cout <<"j_FULL: "<< j_FULL << endl;

    p_RCM = frame_7.p + (frame_8.p - frame_7.p) * lamda;

    

    // cout << "p_RCM: " << p_RCM.data[0] << " " << p_RCM.data[1] << " " << p_RCM.data[2] << endl;
    error_RCM << p_c(0) - p_RCM.data[0], p_c(1) - p_RCM.data[1], p_c(2) - p_RCM.data[2];

    // cout << "error_RCM: " << error_RCM << endl;

    // pri.print_frame(frame_t);
    // pri.print_frame(frame_t_d);

    error_delta_twist = diff(frame_t, frame_t_d);
    Twist_to_Eigen(error_delta_twist, error_delta);

    //计算总误差
    error_FULL.block(0, 0, 6, 1) = error_delta;
    error_FULL.block(6, 0, 3, 1) = error_RCM;

    
    // cout << "error_full: " << error_FULL << endl;

    double error_norm = error_FULL.norm();
    
    // cout << "error_norm:" << error_norm << endl;
    
    if (error_norm < min_error) {//满足收敛条件
        stop_flag = true;
        // save_file(p_c, manipula);
    }
        


    
    // cout << "jac_FULL" <<j_FULL<< endl;

    // q_delta_DOUBLE = pseudoInverse(j_FULL) * error_FULL;
    // cout << null_space_get(j_FULL) * null_motion << endl;
    // cout << "-------------------------" << endl;

    //计算最小奇异值
    // double min_singular = get_min_sigular(j_FULL);
    // // // ROS_INFO("min singular: %f", min_singular);
    // Eigen::MatrixXd q_all(10, 1);
    // q_all.block(0, 0, 9, 1) = q_origin.data;
    // q_all(9) = lamda;
    
    // cout << "singular_axoid" << singular_avoid(q_all, min_singular, joints_limits_max) << endl;

    q_NULL = null_space_get(j_FULL) * null_motion;
    q_delta_DOUBLE = pseudoInverse(j_FULL) * error_FULL;
    
    limit_joint_5(q_origin, q_delta_DOUBLE, q_NULL);
    
    


    q_delta_DOUBLE =q_delta_DOUBLE+q_NULL ;//原始零空间无角度
    // q_delta_DOUBLE_up = pseudoInverse(j_FULL) * error_FULL + null_space_get(j_FULL) * null_motion;
    // q_delta_DOUBLE_down = pseudoInverse(j_FULL) * error_FULL - null_space_get(j_FULL) * null_motion;
    // double mani_up = get_fake_manipulate(q_delta_DOUBLE_up);
    // double mani_down = get_fake_manipulate(q_delta_DOUBLE_down);

    // cout << "fake manipu up: " << mani_up << " fake mani down: " << mani_down << endl;


    // if (mani_up >= mani_down) {
    //     q_delta_DOUBLE = q_delta_DOUBLE_up;
    // }
    // else {
    //     q_delta_DOUBLE = q_delta_DOUBLE_down;
    // }

    


    

    // pri.print_joints(q_origin);
    // q_delta_DOUBLE = pseudoInverse(j_FULL) * error_FULL + null_space_get(j_FULL) * singular_avoid(q_all, min_singular);
    
    // cout << pseudoInverse(j_FULL) << endl;
    // cout << "--------------------------" << endl;
    set_k_error();
    // cout << "lamda_delta: " << q_delta_DOUBLE(9) << endl;
    q_delta_DOUBLE = k_error.asDiagonal() * q_delta_DOUBLE;

    q_delta.data = q_delta_DOUBLE.block(0, 0, 9, 1);

    //求逆解，更新角度与 λ
    lamda = lamda + q_delta_DOUBLE(9, 0);
    // cout << "lamda: " << lamda << endl;
    
    Add(q_origin, q_delta, q_answer);
    
    // limit_joints(q_answer);
    
    

    // get_jac_R_O(jaco_t.data, j_RCM);

    // pri.print_frame(frame_t);


    // pri.print_joints(q_answer);
    // cout << "lamda: " << lamda << endl;

    //当可操作性过小时，返回迭代前的角度
    if (over2_flag) {
        return q_origin;
    }
    else {
        return  q_answer;
    }

}


