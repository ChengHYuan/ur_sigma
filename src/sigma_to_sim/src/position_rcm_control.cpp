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

SurgicalRobotUR robot;
//ik_solver_test(robot);
Chain robot_chain = robot.ur10e_chain;
ChainFkSolverPos_recursive robot_fk_solver = *robot.fwdkin;
Eigen::Vector3d p_c(0.702, 0, 0.217);//RCM点

Jacobian jaco_t(9);

vector<double> joints_limits_max = { M_PI,   M_PI,    M_PI,    M_PI,    M_PI,    M_PI,    M_PI+M_PI/2, M_PI*80/180-M_PI/2, M_PI*80/180,1.0 };
vector<double> joints_limits_min = { -M_PI,   -M_PI,    -M_PI,    -M_PI,    -M_PI,    -M_PI,    -M_PI+M_PI/2, M_PI*80/180-M_PI/2, M_PI*80/180 ,0.0};
Eigen::Matrix3d pcpr_cross;
Eigen::Vector3d p_r, pcpr_differ;
Frame frame_in;
Twist error_delta_twist;
Eigen::MatrixXd error_rcm(2, 1);
ChainJntToJacSolver jacSolver(robot_chain);
JntArray q_delta(9), q_answer(9), q_in(9), v_out(9);

JntArray q_init(9);

double min_error = 1e-5;
Eigen::Matrix<double, 6, 1> error_delta;

Printer pri;
Frame frame_7, frame_8,frame_t;
Jacobian jac_7(9), jac_8(9);
Eigen::MatrixXd jac_7_double(6, 9), jac_8_double(6, 9);

double itr_size = 100;//设置迭代次数

//误差增益矩阵
Eigen::VectorXd k_error(10);

//设置误差增益矩阵
void set_k_error() {
    // k_error << 10, 10, 10, 10, 10, 10, 10, 10, 10;
    k_error << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,1.0;
    k_error =k_error*1.0;
}


//求矩阵的秩
int get_matrix_rank(Eigen::MatrixXd &A) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
    // std::cout<<"A :\n"<<A<<std::endl;
    // std::cout << "rank:\n" << svd.rank() << std::endl;
    // std::cout << "svd value:\n" << A.eigenvalues() << endl;
    return svd.rank();
}

//求矩阵伪逆
template<typename _Matrix_Type_> 
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon()) 
{  
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);  
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);  
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint(); 
}

//文件存储（RCM点坐标与可操作性）
void save_file(Eigen::Vector3d &rcm,double manipula) {
    ofstream ofs;
    ofs.open("/home/chy/rcm_point_file/text.txt", ios::out);//以写的方式打开文件
    ofs << rcm.x() << " " << rcm.y()<<" " << rcm.z()<<" " <<manipula<< endl;
    ofs.close();
}


//计算雅可比矩阵零空间矩阵
Eigen::Matrix<double, 10, 10>null_space_get(Eigen::MatrixXd& j_full) {
    Eigen::MatrixXd null_matrix(10, 10);
    null_matrix.setIdentity();
    // cout << j_full << endl;
    // cout << pseudoInverse(j_full) << endl;
    null_matrix = null_matrix - pseudoInverse(j_full) * j_full;
    get_matrix_rank(null_matrix);
    return null_matrix;
}

//计算带约束的可操作性
double rcm_manipula(Eigen::Matrix<double, 11, 11> jac_full) {
    return sqrt((jac_full * jac_full.transpose()).determinant());
}

//输入俯仰角和偏转角，计算根据当前的RCM点计算新的RCM点,返回末端位姿
Frame get_new_rcm(Vector& end_position, Eigen::Vector3d& rcm_point, double& pitch, double& yaw) {
    double r = sqrt((rcm_point[0] - end_position(0)) * (rcm_point[0] - end_position(0)) +
        (rcm_point[1] - end_position(1)) * (rcm_point[1] - end_position(1)) +
        (rcm_point[2] - end_position(2)) * (rcm_point[2] - end_position(2)));
    p_c[0] = end_position(0) + r * cos(pitch) * cos(yaw);
    p_c[1] = end_position(1) + r * cos(pitch) * sin(yaw);
    p_c[2] = end_position(2) + r * sin(pitch);
    Frame answer(end_position);
    answer.M.DoRotY(M_PI / 2);
    answer.M.DoRotX(yaw);
    answer.M.DoRotY(pitch);
    return answer;
}

//末端点圆周轨迹测试
Frame circle_traj(Vector endPosi,int step) {
    double ratio = 0.02;
    Vector target_posi(endPosi.x() + ratio * sin(double(step) / 1000 * 2 * M_PI),
                       endPosi.y() + ratio * cos(double(step) / 1000 * 2 * M_PI),
        endPosi.z());
    Frame answer(target_posi);
    // pri.print_frame(answer);
    answer.M.DoRotY(M_PI / 2);
    return answer;
}
//方轨迹测试
Frame square_traj(Vector endPosi,int step) {
    double side = 0.12;
    int index_size = 3000;
    double delta_x = 0.0, delta_y = 0.0;
    int step_left = step % index_size;
    double plus_value = 4 * side / index_size;
    
    if (step_left < index_size / 4) {
        delta_x =- side / 2;
        delta_y =- side / 2+step_left*plus_value;
    }
    else if((index_size / 4<=step_left )&& (step_left<index_size / 2) ){
        delta_x = -side / 2+(step_left-index_size / 4)*plus_value;
        delta_y = side / 2;
    }
    else if ((index_size / 2<=step_left )&& (step_left<3*index_size / 4)) {
        delta_x = side / 2;
        delta_y = side / 2-(step_left-index_size / 2)*plus_value;
    }
    else {
        delta_x = side / 2-(step_left-3*index_size / 4)*plus_value;
        delta_y = -side / 2;
    }

    Vector target_posi(endPosi.x() + delta_x,
                       endPosi.y() + delta_y,endPosi.z()-0.02);

    Frame answer(target_posi);
    // pri.print_frame(answer);
    answer.M.DoRotY(M_PI / 2);
    // pri.print_frame(answer);
    return answer;
}

//速度转化为矩阵
void Twist_to_Eigen(Twist& t, Eigen::Matrix<double, 6, 1>& m) {
    m(0) = t.vel.data[0];
    m(1) = t.vel.data[1];
    m(2) = t.vel.data[2];
    m(3) = t.rot.data[0];
    m(4) = t.rot.data[1];
    m(5) = t.rot.data[2];
}


//求RCM矩阵伪逆(行满秩,右伪逆)
Eigen::Matrix<double, 11, 3> pseudoInverse_RCM(Eigen::Matrix<double, 3, 11>& A) {
    return A.transpose() * (A * A.transpose()).inverse();
}

//打印可操作性(未考虑约束)
void print_manipula_all(JntArray& joints) {
    vector<double> manipula = {0,0};
    Eigen::MatrixXd jac_double_v(3, 10), jac_double_w(3, 10), jac_dot_jac_v(3, 3), jac_dot_jac_w(3, 3);
    Jacobian local_jacobian(10);
    jacSolver.JntToJac(joints, local_jacobian);
    // cout << "cols: " << local_jacobian.data.cols() << " rows: " << local_jacobian.data.rows() << endl;

    jac_double_v = local_jacobian.data.block(0, 0, 3, 10);
    jac_double_w = local_jacobian.data.block(3, 0, 3, 10);
    
    // cout << "local jacobian: "<<local_jacobian.data << endl;
    jac_dot_jac_v = jac_double_v * jac_double_v.transpose();
    jac_dot_jac_w = jac_double_w * jac_double_w.transpose();
    double manipula_v = sqrt(jac_dot_jac_v.determinant());
    double manipula_w = sqrt(jac_dot_jac_w.determinant());
    manipula[0] = manipula_v;
    manipula[1] = manipula_w;
    ROS_INFO("\nmanipulability of linear now: %f \nmanipulability of rotate now: %f\n--------------------------", manipula_v, manipula_w);

}


//计算reach可操作性和orient可操作性
void get_jac_R_O(Eigen::Matrix<double, 6, 10> jac_TIP, Eigen::Matrix<double, 3, 11> jac_RCM) {
    Eigen::MatrixXd ident(11, 11), jac_double_v(3, 11), jac_double_w(3, 11), jac_R(3, 11), jac_O(3, 11);
    ident.setIdentity();
    jac_double_v.block(0, 0, 3, 10) = jac_TIP.block(0, 0, 3, 10);
    jac_double_v.block(0, 10, 3, 1) << 0, 0, 0;
    jac_double_w.block(0, 0, 3, 10) = jac_TIP.block(3, 0, 3, 10);
    jac_double_w.block(0, 10, 3, 1) << 0, 0, 0;

    // cout << ident - pseudoInverse(jac_double_w) * jac_double_w << endl;
    // cout << pseudoInverse_RCM(jac_RCM) << endl;


    //计算可达可操作性与方向可操作性
    jac_R = jac_double_v * (ident - pseudoInverse(jac_double_w) * jac_double_w) * (ident - pseudoInverse_RCM(jac_RCM) * jac_RCM);
    jac_O = jac_double_w * (ident - pseudoInverse(jac_double_v) * jac_double_v) * (ident - pseudoInverse_RCM(jac_RCM) * jac_RCM);
    // cout << (jac_R * jac_R.transpose()).inverse() << endl;
    //计算可操作性系数
    double k_r = sqrt((jac_R * jac_R.transpose()).trace() * (jac_R * jac_R.transpose()).inverse().trace());
    double k_o = sqrt((jac_O * jac_O.transpose()).trace() * (jac_O * jac_O.transpose()).inverse().trace());

    cout << "k_r: " << 1 / k_r << " k_o: " << 1 / k_o << endl;
    
    cout << "------------------------" << endl;

}

void ik_solver_test(SurgicalRobotUR& robot) {
    ChainIkSolverPos_LMA robot_ik_solver(robot.ur10e_chain);
    JntArray q_out(robot.ur10e_chain.getNrOfJoints());
    Vector end_position(0.6, 0, 0.5);
    Frame end_pose(end_position);
    end_pose.M.DoRotX(M_PI);
    end_pose.M.DoRotZ(M_PI);
    //cout << end_pose.M.data[8] << endl;
    // cout << end_pose.p.x() << endl;
    //CartToJnt返回值: E_DEGRADED         = +1,
    //                E_NOERROR          =  0,
    //                E_NO_CONVERGE      = -1,
    //                E_UNDEFINED        = -2,
    //                E_NOT_UP_TO_DATE = -3,
    //                E_SIZE_MISMATCH = -4,
    //                E_MAX_ITERATIONS_EXCEEDED = -5,
    //                E_OUT_OF_RANGE = -6,
    //                E_NOT_IMPLEMENTED = -7,
    //                E_SVD_FAILED = -8
    
    int is_ok = robot_ik_solver.CartToJnt(robot.q, end_pose, q_out);
    if (is_ok < 0) {                                        
        ROS_INFO("failed to get ik solver");
    }
    else {
        cout << q_out.data << endl;
        // Frame end_calculate;
        // robot.fwdkin->JntToCart(q_out, end_calculate, -1);
        // cout << end_calculate.p.y() << endl;
    }
}

double get_min_sigular(const Eigen::MatrixXd& matrix) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double smallestSingularValue = svd.singularValues().tail(1)(0);
    return smallestSingularValue;
}

//奇异值避免
Eigen::MatrixXd singular_avoid(Eigen::MatrixXd& q, double& delta) {
    int num = 10;
    Eigen::MatrixXd q_avoid(num,1);
    
    //计算deltaH
    vector<double> H_gred(num);
    for (int i = 0;i < H_gred.size();++i) {
        H_gred[i] = ((joints_limits_max[i] - joints_limits_min[i]) * (joints_limits_max[i] - joints_limits_min[i])
            * (2 * q(i) - joints_limits_max[i] - joints_limits_min[i])) / (num * (joints_limits_max[i] - q(i))
                * (joints_limits_max[i] - q(i)) * (q(i) - joints_limits_min[i]) * (q(i) - joints_limits_min[i]));
    }

    cout << "H:" << H_gred[0]<<" "<<H_gred[1] <<" "<<H_gred[2]<<" "<<H_gred[3]<< H_gred[4]<<" "<<H_gred[5] <<" "<<H_gred[6]<<" "<<H_gred[7]<< endl;
    //计算系数K，与可操作性有关
    double delta_0 = 0.03;
    double K = 0.0;
    double K_m = 0.01;
    if (delta >= 0 && delta < delta_0) {
        K = 0.0;
    }
    else if (delta >= delta_0 && delta < 2 * delta_0) {
        K=K_m* (sin(M_PI * delta / delta_0 + M_PI / 2) + 1.0) / 2.0;
    }
    else if (delta >= 2 * delta_0) {
        K=K_m;
    }
    // cout << "K:" <<K<< endl;
    
    for (int j = 0;j < q_avoid.size();++j) {
        q_avoid(j) = K * H_gred[j];
    }
    
    return q_avoid;

}


//每时每刻将关节角的输出值限制在-pi到+pi之间
void limit_joints(JntArray &q) {
    double temp;
    for (int i = 0;i < q.data.size();++i) {
        if (q.data(i) > M_PI || q.data(i) < -M_PI) {
            temp = fmod(q.data(i), 2 * M_PI);
            if (temp >= M_PI) {
                q.data(i) = temp - 2 * M_PI;
                // cout << "flag 2" << endl;
            }
            else if (temp < -M_PI) {
                q.data(i) = temp + 2 * M_PI;
                // cout << "flag 1" << endl;
            }
            else {
                q.data(i) = temp;
            }
            // cout << "data(" << i << "): " << q.data(i) << endl;
        }
    }
}

double get_fake_manipulate(const Eigen::MatrixXd& q_fake) {
    double lamda = q_fake(9);
    JntArray q_origin(9);
    q_origin.data = q_fake.block(0, 0, 9, 1);

    Frame frame_7_fake, frame_8_fake,frame_t_fake;
    Eigen::MatrixXd j_RCM(3, 10), j_FULL(9, 10);
    Jacobian jac_7_fake(9), jac_8_fake(9),jac_t_fake(9);
    robot_fk_solver.JntToCart(q_origin, frame_t_fake);
    robot_fk_solver.JntToCart(q_origin, frame_7_fake, 6);
    robot_fk_solver.JntToCart(q_origin, frame_8_fake, 7);
    // pri.print_frame(frame_t);
    // pri.print_frame(frame_9);
    
    Vector p_delta_ii;
    p_delta_ii = frame_8.p - frame_7.p;
    // cout << "p_i-p_i+1: " << p_delta_ii.data[0] << " " << p_delta_ii.data[1] << " " << p_delta_ii.data[2] << endl;
    //获取 第8个关节 第九个关节 雅克比
    jacSolver.JntToJac(q_origin, jac_7_fake, 6);
    jacSolver.JntToJac(q_origin, jac_8_fake, 7);
    jacSolver.JntToJac(q_origin, jac_t_fake);


    j_RCM.block(0, 0, 3, 9) = (jac_7_fake.data + lamda * (jac_8_fake.data - jac_7_fake.data)).block(0, 0, 3, 9);
    j_RCM.block(0, 9, 3, 1) << p_delta_ii.x(), p_delta_ii.y(), p_delta_ii.z();
    // ROS_INFO("flag!");
    
    //计算总的雅可比矩阵
    j_FULL.block(0, 0, 6, 9) = jac_t_fake.data;
    j_FULL.block(0, 9, 6, 1) << 0, 0, 0, 0, 0, 0;
    j_FULL.block(6, 0, 3, 10) = j_RCM;

    double manipula = sqrt((j_FULL * j_FULL.transpose()).determinant());

    return manipula;

}

//约束第五个关节，当超出时在零空间施加反向角度进行约束
void limit_joint_5(JntArray& q_origin, Eigen::MatrixXd& q_delta, Eigen::MatrixXd& q_NULL) {
    double bottom__lim = -120 * M_PI / 180;
    double top_lim = 0.0;
    // cout << q_NULL << endl;
    // cout << "-------------------" << endl;
    if (q_NULL(4) < 0) {
        q_NULL = -q_NULL;
    }
    if (q_origin.data(4) + q_delta(4) > top_lim) {
        
        q_NULL = -q_NULL;
    }
    else if(q_delta(4)<bottom__lim) {
        q_NULL = q_NULL;
    }
    else {
        q_NULL = q_NULL *0.0;
    }
}

//单步逆运动学 新 6自由度机器人
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


int main(int argc, char* argv[]){

    ros::init(argc, argv, "robot_rcm_control");
    
    int step = 0;
    double flag = 1e-8;
    
    Vector end_position(0.702, 0, 0.08);
    
    Frame end_pose(end_position);
    RobotSubPub sub_pub;
    sub_pub.sub_pub_init();
    
    end_pose.M.DoRotY(M_PI / 2);
    // pri.print_frame(end_pose);

    
    frame_in = end_pose;
    // pri.print_frame(frame_in);
    bool stop_flag = false;
    bool over1_flag = false;
    double lamda;

    // Frame frame_local;
    
    ros::Rate rate(200);
    int itr_step = 0;

    q_init.data << 0.344386781162184, -1.5510568513695429, -2.245217419290604, 0.654681617069999, 1.2918835865389384, 0,0.06515547633171082+M_PI/2, -M_PI/2, 0.0;


    //每次运行时，初始化轨迹规划到默认位置
    // while ((!sub_pub.is_vrep_get || !sub_pub.is_real_get) && ros::ok()) {
    //     ros::spinOnce();
    //     if (!sub_pub.is_real_get) {
    //         ROS_INFO("Waiting for ur real msg........");
    //     }
    //     if (!sub_pub.is_vrep_get) {
    //         ROS_INFO("Waiting for vrep start........");
    //     }
    //     ros::Duration(0.10).sleep();
    // }
    // ROS_INFO("real robot get !!!");
    // int count_range = 2000;
    // for (int count = 0;count < count_range;++count) {
        
    //     q_in = sub_pub.joint_planning(sub_pub.joints_now_real, q_init, count, count_range);
        
    //     sub_pub.pub_of_all(q_in);
        
    //     if (count % 100 == 0) {
    //         ROS_INFO("------%d msg sent!!-------", count);
    //     }
    //     // pri.print_joints(q_in);
    //     rate.sleep();
    // }
    // sub_pub.joints_pubed = q_in;
    // ROS_INFO("Joints Planning Success!!");
    // ROS_INFO("Teleoperation Start!!");

    for (int t = 0;t < 10;++t) {
        ros::spinOnce();
        // ROS_INFO("msgs getting.....");
        ros::Duration(0.05).sleep();
    }
    
    Eigen::Vector3d p_c_saved = p_c;
    while (ros::ok())
    {
        double start = ros::Time::now().toSec();
        frame_in = circle_traj(end_position, itr_step);
        // frame_in = square_traj(end_position, itr_step);
        // frame_in = sub_pub.command_frame;

        q_in = sub_pub.joints_now;
        
        // pri.print_joints(q_in);
        // pri.print_frame(frame_in);

        // robot.fwdkin->JntToCart(q_in, frame_local);
        
        //循环计算不同RCM点的可操作性
        // if (itr_step < 250) {
        //     for (int i = 0;i < itr_size;++i) {
                
        //         double yaw = 2 * double(itr_step) * M_PI / 250;
        //         double pitch = double(i) * M_PI / 500;

        //         cout <<"yaw: " <<yaw <<" "<<"pitch: "<< pitch << endl;
        //         get_new_rcm(end_position, p_c_saved, pitch, yaw);

        lamda = get_lamda(q_in);
        // cout << "lamda: " << lamda << endl;
        // lamda = 0;
                /////////////////////////////////迭代法，发送位置////////////////////////////////////////
                
                while (step < 100) {
                    q_in = invOneStep(frame_in, q_in, stop_flag, lamda, over1_flag);
                    
                    if (stop_flag) {
                        // cout <<"----------step: " << step <<"----------"<< endl;
                        // cout <<"stop_flag"<< stop_flag << endl;
                        break;
                    }
                    step++;
                    if (step == 100) {
                        ROS_INFO("iterate step is out of range");
                    }

                }
        //     }
        // }
        // else {
        //     ROS_INFO("RCM points is enough!!!");
        // }

        // cout << "lamda: " << lamda << endl;

        // robot.fwdkin->JntToCart(q_in, robot.pose_end);

        // pri.print_joints(sub_pub.joints_now);
        // pri.print_joints(q_in);
        // print_manipula_all(q_in);

        // sub_pub.get_joints_velocity(q_in);
        // pri.print_joints(sub_pub.joints_velocity);

        if (!over1_flag) {
                    // cout << "flag1...." << endl;
                    // q_in = sub_pub.keep_joints_save(q_in);
            // cout << "flag2...." << endl;
            sub_pub.pub_of_all(q_in);
            // cout << "flag3...." << endl;
            // sub_pub.pub_of_velocity(q_in);
        }
        
        over1_flag = false;
        stop_flag = false;
        step = 0;
        itr_step++;


        ros::spinOnce();
        double endd = ros::Time::now().toSec();
        // cout << "duration: " << (endd - start) * 1000 << " ms" << endl;
        rate.sleep();

    }


    //用于计算当RCM变化时最佳角度
    // while (ros::ok())
    // {
    //     //循环计算不同RCM点的可操作性
    //     if (itr_step < 250) {
    //         for (int i = 0;i < itr_size;++i) {
    //             // q_in = sub_pub.joints_now;
    //             double yaw = 2 * double(itr_step) * M_PI / 250;
    //             double pitch = double(i) * M_PI / 500;

    //             // cout <<"yaw: " <<yaw <<" "<<"pitch: "<< pitch << endl;
    //             frame_in = get_new_rcm(end_position, p_c_saved, pitch, yaw);

    //             lamda = get_lamda(q_in);
    //             // lamda = 0;
    //             /////////////////////////////////迭代法，发送位置////////////////////////////////////////
    //             while (step < 100) {
    //                 q_in = invOneStep(frame_in, q_in, stop_flag, lamda, over1_flag);
    //                 if (stop_flag) {
    //                     // cout <<"----------step: " << step <<"----------"<< endl;
    //                     // cout <<"stop_flag"<< stop_flag << endl;
    //                     break;
    //                 }
    //                 step++;
    //                 if (step == 100) {
    //                     ROS_INFO("iterate step is out of range");
    //                 }

    //             }
    //             sub_pub.pub_of_all(q_in);
    //             ros::spinOnce();
    //             rate.sleep();
    //         }
    //     }
    //     else {
    //         ROS_INFO("RCM points is enough!!!");
    //     }

    //     cout << itr_step << endl;

    //     // pri.print_joints(q_in);
    //     // print_manipula_all(q_in);
    // }
    return 0;
}


