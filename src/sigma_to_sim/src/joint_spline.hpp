#include <vector>
#include <deque>
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointPositionVelocity.h"
#include <math.h>
#include "std_msgs/Float64MultiArray.h"
using namespace std;

class IIwaSpline {
public:
    int SIZE = 5;//传入点数
    int EXPEND_EACH = 4;//两点间分段数量
    int EXPEND_POINT = SIZE + (EXPEND_EACH - 1) * (SIZE - 1);//扩张后点的数量
    double hz;//点的原始发布频率
    double duration;//时间间隔
    double duration_expend;//拓展后的时间间隔
    double JV_LIMIT = 0.7;
    vector<vector<double>> joint_positions;
    vector<vector<double>> joint_velocity = vector<vector<double>>(SIZE, vector<double>(7,0));
    vector<vector<double>> joint_acceleration = vector<vector<double>>(SIZE, vector<double>(7,0));
    vector<double> joint_velocity_now = vector<double>(7, 0);
    vector<double> joint_acceleration_now=vector<double>(7,0);

    deque<vector<double>> joint_positions_spline;
    deque<vector<double>> joint_velocity_spline;
    deque<iiwa_msgs::JointPositionVelocity> iiwa_position_velocity;
    deque<iiwa_msgs::JointPosition> iiwa_position;
    
    vector<vector<double>>a_n = vector<vector<double>>(SIZE, vector<double>(SIZE));
    iiwa_msgs::JointPositionVelocity jpv;
    iiwa_msgs::JointPosition jp;

    
    bool is_position_full = false;//判断是否存满
    
    //五次多项式系数 SIZE个点、7个关节
    vector<vector<double>> a0 = vector<vector<double>>(SIZE-1, vector<double>(7, 0));
    vector<vector<double>> a1 = vector<vector<double>>(SIZE-1, vector<double>(7, 0));
    vector<vector<double>> a2 = vector<vector<double>>(SIZE-1, vector<double>(7, 0));
    vector<vector<double>> a3 = vector<vector<double>>(SIZE-1, vector<double>(7, 0));
    vector<vector<double>> a4 = vector<vector<double>>(SIZE-1, vector<double>(7, 0));
    vector<vector<double>> a5 = vector<vector<double>>(SIZE-1, vector<double>(7, 0));

    
public:
    IIwaSpline() {
        joint_positions;
        hz = 200;
        duration = 1.0 / hz;
        duration_expend = duration / EXPEND_EACH;
    }
    void get_joints(const iiwa_msgs::JointPosition& iiwa_joint);//存储并更新关节角
    void update_joint(vector<double>&new_joint);//更新角度
    void check_velocity();//通过关节位置计算关节的速度
    void check_acceleration();//通过关节位置计算关节的加速度
    void get_a_n();//获取五次插值的
    void spline_generate();//生成样条点
    iiwa_msgs::JointPositionVelocity transfer_pv(vector<double>& position, vector<double>& velocity);//将计算出的关节位置和速度转化为iiwa的消息
    iiwa_msgs::JointPosition transfer_p(vector<double>& position);//将计算出的关节位置转化为iiwa的消息
    iiwa_msgs::JointPositionVelocity get_iiwa_position_velocity();//获取队列头的点
    iiwa_msgs::JointPosition get_iiwa_position();
    std_msgs::Float64MultiArray get_iiwa_fri_position();
    void iiwa_joint_spline(const iiwa_msgs::JointPosition& iiwa_joint);//主要函数
    void print_info(vector<vector<double>>& obj);
    bool is_joint_over_limit(iiwa_msgs::JointPositionVelocity& jpv);
};

//存储并更新关节角
void IIwaSpline::get_joints(const iiwa_msgs::JointPosition& iiwa_joint) {
    vector<double> new_joint(7);
    new_joint[0] = iiwa_joint.position.a1;
    new_joint[1] = iiwa_joint.position.a2;
    new_joint[2] = iiwa_joint.position.a3;
    new_joint[3] = iiwa_joint.position.a4;
    new_joint[4] = iiwa_joint.position.a5;
    new_joint[5] = iiwa_joint.position.a6;
    new_joint[6] = iiwa_joint.position.a7;

    if (joint_positions.size() < SIZE) {
        //新的在后，旧的在前
        joint_positions.push_back(new_joint);
    }
    else {
        is_position_full = true;
        update_joint(new_joint);
    }
    // print_info(joint_positions);
}

//更新角度
void IIwaSpline::update_joint(vector<double>&new_joint) {
    joint_positions[0] = joint_positions[1];
    joint_positions[1] = joint_positions[2];
    joint_positions[2] = joint_positions[3];
    joint_positions[3] = joint_positions[4];
    joint_positions[4] = new_joint;
    joint_velocity_now = joint_velocity[1];
    joint_acceleration_now = joint_acceleration[1];
}

//获取节点速度，要求：速度同正负则取均值，速度不同正负取0
void IIwaSpline::check_velocity() {
    vector<double>v1(7);
    vector<double>v2(7);
    // cout << "duration: " << duration << " duration_exp" << duration_expend << endl;
    for (int i = 0;i < SIZE - 1;++i) {
        if (i == 0) {
            joint_velocity[0] = joint_velocity_now;
        }
        else {
            for (int j = 0;j < 7;++j) {
                v1[j] = (joint_positions[i][j] - joint_positions[i - 1][j]) / duration;
                v2[j] = (joint_positions[i + 1][j] - joint_positions[i][j]) / duration;
                // cout << "v1: " << v1[0] << " " << v1[1] << " " << v1[2] << " " << v1[3] << " " << v1[4] << " " << v1[5] << " " << v1[6] << " " << endl;
                // cout << "v2: " << v2[0] << " " << v2[1] << " " << v2[2] << " " << v2[3] << " " << v2[4] << " " << v2[5] << " " << v2[6] << " " << endl;
                if ((v1[j] >= 0 && v2[j] >= 0) || (v1[j] <= 0 && v2[j] <= 0)) {
                    joint_velocity[i][j] = (v1[j] + v2[j]) / 2;
                }
                else {
                    joint_velocity[i][j] = 0;
                }
            }
        }

    }
    joint_velocity[SIZE - 1] = vector<double>(7, 0);
    // print_info(joint_velocity);
}

void IIwaSpline::check_acceleration() {
    vector<double>accel1(7);
    vector<double>accel2(7);
    // cout << "duration: " << duration << " duration_exp" << duration_expend << endl;
    for (int i = 0;i < SIZE - 2;++i) {
        if (i == 0) {
            joint_acceleration[0] = joint_acceleration_now;
        }
        else {
            for (int j = 0;j < 7;++j) {
                accel1[j] = (joint_velocity[i][j] - joint_velocity[i - 1][j]) / duration;
                accel2[j] = (joint_velocity[i + 1][j] - joint_velocity[i][j]) / duration;
                if ((accel1[j] >= 0 && accel2[j] >= 0) || (accel1[j] <= 0 && accel2[j] <= 0)) {
                    joint_acceleration[i][j] = (accel1[j] + accel2[j]) / 2;
                }
                else {
                    joint_acceleration[i][j] = 0;
                }
            }
        }

    }
    joint_acceleration[SIZE - 1] = vector<double>(7, 0);
    print_info(joint_acceleration);
}

//获取五次插值系数
void IIwaSpline::get_a_n() {
    // 每两点之间的关节角度 ： theta(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    for (int i = 0;i < SIZE - 2;++i) {
        for (int j = 0;j < 7;++j) {
            a0[i][j] = joint_positions[i][j];
            a1[i][j] = joint_velocity[i][j];
            a2[i][j] = joint_acceleration[i][j] / 2;
            a3[i][j] = (20 * joint_positions[i + 1][j] - 20 * joint_positions[i][j] - (8 * joint_velocity[i + 1][j] + 12 * joint_velocity[i][j]) * duration - (3 * joint_acceleration[i][j] - joint_acceleration[i + 1][j]) * duration * duration) / (2 * pow(duration, 3));
            a4[i][j] = (30 * joint_positions[i][j] - 30 * joint_positions[i + 1][j] + (14 * joint_velocity[i + 1][j] + 16 * joint_velocity[i][j]) * duration + (3 * joint_acceleration[i][j] - 2 * joint_acceleration[i + 1][j]) * duration * duration) / (2 * pow(duration, 4));
            a5[i][j] = (12 * joint_positions[i + 1][j] - 12 * joint_positions[i][j] - (6 * joint_velocity[i + 1][j] + 6 * joint_velocity[i][j]) * duration - (joint_acceleration[i][j] - joint_acceleration[i + 1][j]) * duration * duration) / (2 * pow(duration, 5));
        }
    }
    // print_info(a3);
}

//生成样条点
void IIwaSpline::spline_generate() {
    //为空时需要存储所有的点，满的时候只需要存储最后一段的点
    if (joint_positions_spline.empty()) {
        for (int i = 0;i < SIZE - 2;++i) {//对于一个大时间段遍历
            for (int j = 0;j < EXPEND_EACH;++j) {//对于每一段中的小段时间遍历
                double t = j * duration_expend;
                vector<double> position(7);
                vector<double> velocity(7);
                for (int k = 0;k < 7;++k) {//对7个关节遍历
                    position[k] = a0[i][k] + a1[i][k] * t + a2[i][k] * pow(t, 2) + a3[i][k] * pow(t, 3) + a4[i][k] * pow(t, 4) + a5[i][k] * pow(t, 5);
                    velocity[k] = a1[i][k] + 2 * a2[i][k] * t + 3 * a3[i][k] * pow(t, 2) + 4 * a4[i][k] * pow(t, 3) + 5 * a5[i][k] * pow(t, 4);
                }
                joint_positions_spline.push_back(position);
                joint_velocity_spline.push_back(velocity);
                iiwa_position_velocity.push_back(transfer_pv(position, velocity));
                iiwa_position.push_back(transfer_p(position));
            }
        }
    }
    else {
        int i = SIZE - 3;
        // for (int j = 0;j < EXPEND_EACH;++j) {
        //     //当有新点过来时，弹出最后的一串点，并且替换成新点
        //     joint_positions_spline.pop_back();
        //     joint_velocity_spline.pop_back();
        //     joint_position_velocity.pop_back();
        // }
        for (int j = 0;j < EXPEND_EACH;++j) {//对于每一段中的小段时间遍历
                double t = j * duration_expend;
                vector<double> position(7);
                vector<double> velocity(7);
                for (int k = 0;k < 7;++k) {//对7个关节遍历
                    position[k] = a0[i][k] + a1[i][k] * t + a2[i][k] * pow(t, 2) + a3[i][k] * pow(t, 3) + a4[i][k] * pow(t, 4) + a5[i][k] * pow(t, 5);
                    velocity[k] = a1[i][k] + 2 * a2[i][k] * t + 3 * a3[i][k] * pow(t, 2) + 4 * a4[i][k] * pow(t, 3) + 5 * a5[i][k] * pow(t, 4);
                }
                joint_positions_spline.push_back(position);
                joint_velocity_spline.push_back(velocity);
                iiwa_position_velocity.push_back(transfer_pv(position, velocity));
                iiwa_position.push_back(transfer_p(position));
        }
    }

}

iiwa_msgs::JointPositionVelocity IIwaSpline::transfer_pv(vector<double>& position, vector<double>& velocity) {
    iiwa_msgs::JointPositionVelocity j_p_v;
    
    j_p_v.position.a1 = position[0];
    j_p_v.position.a2 = position[1];
    j_p_v.position.a3 = position[2];
    j_p_v.position.a4 = position[3];
    j_p_v.position.a5 = position[4];
    j_p_v.position.a6 = position[5];
    j_p_v.position.a7 = position[6];
    
    j_p_v.velocity.a1 = velocity[0];
    j_p_v.velocity.a2 = velocity[1];
    j_p_v.velocity.a3 = velocity[2];
    j_p_v.velocity.a4 = velocity[3];
    j_p_v.velocity.a5 = velocity[4];
    j_p_v.velocity.a6 = velocity[5];
    j_p_v.velocity.a7 = velocity[6];
    return j_p_v;
}

iiwa_msgs::JointPosition IIwaSpline::transfer_p(vector<double>& position) {
    iiwa_msgs::JointPosition j_p;
    j_p.position.a1 = position[0];
    j_p.position.a2 = position[1];
    j_p.position.a3 = position[2];
    j_p.position.a4 = position[3];
    j_p.position.a5 = position[4];
    j_p.position.a6 = position[5];
    j_p.position.a7 = position[6];
    return j_p;
}

//获取需要发布的关节角度和速度
iiwa_msgs::JointPositionVelocity IIwaSpline::get_iiwa_position_velocity() {
    iiwa_msgs::JointPositionVelocity answer=iiwa_position_velocity.front();
    joint_positions_spline.pop_front();
    joint_velocity_spline.pop_front();
    iiwa_position_velocity.pop_front();
    iiwa_position.pop_front();
    return answer;
}

//获取需要发布的关节角度
iiwa_msgs::JointPosition IIwaSpline::get_iiwa_position() {
    iiwa_msgs::JointPosition answer=iiwa_position.front();
    joint_positions_spline.pop_front();
    joint_velocity_spline.pop_front();
    iiwa_position_velocity.pop_front();
    iiwa_position.pop_front();
    return answer;
}

//获取需要发布的关节角度
std_msgs::Float64MultiArray IIwaSpline::get_iiwa_fri_position() {
    std_msgs::Float64MultiArray answer;
    answer.data = joint_positions_spline.front();
    
    joint_positions_spline.pop_front();
    joint_velocity_spline.pop_front();
    iiwa_position_velocity.pop_front();
    iiwa_position.pop_front();
    return answer;
}


//打印某元素信息
void IIwaSpline::print_info(vector<vector<double>>& obj) {
    for (int i = 0;i < obj.size();++i) {
        for (int j = 0;j < obj[0].size();++j) {
            cout << obj[i][j] << " ";
        }
        cout << endl;
    }
    cout << "-----------------------" << endl;
}

//主要函数
void IIwaSpline::iiwa_joint_spline(const iiwa_msgs::JointPosition& iiwa_joint) {
    get_joints(iiwa_joint);
    if (is_position_full) {
        check_velocity();
        // check_acceleration();
        get_a_n();
        spline_generate();
    }
    else {
        cout << "----------the points are not enough------" << endl;
    }
}

bool IIwaSpline::is_joint_over_limit(iiwa_msgs::JointPositionVelocity& jpv) {
    bool is_over_limit = false;
    vector<double> jv(7);
    jv[0] = jpv.velocity.a1;
    jv[1] = jpv.velocity.a2;
    jv[2] = jpv.velocity.a3;
    jv[3] = jpv.velocity.a4;
    jv[4] = jpv.velocity.a5;
    jv[5] = jpv.velocity.a6;
    jv[6] = jpv.velocity.a7;
    for (int i = 0;i < 7;++i) {
        if (abs(jv[i]) > JV_LIMIT) {
            is_over_limit = true;
            if (jv[i] > 0.0) {
                jv[i] = JV_LIMIT;
            }
            else {
                jv[i] = -JV_LIMIT;
            }
        }
    }
    jpv.velocity.a1 = jv[0];
    jpv.velocity.a2 = jv[1];
    jpv.velocity.a3 = jv[2];
    jpv.velocity.a4 = jv[3];
    jpv.velocity.a5 = jv[4];
    jpv.velocity.a6 = jv[5];
    jpv.velocity.a7 = jv[6];
    return is_over_limit;
}

