#include <iostream>
#include <string>
#include <vector>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/CartesianPose.h>
#include <geometry_msgs/PoseStamped.h>
#include "surgical_robot.hpp"
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "printer.hpp"
#include "surgical_robot_pub_sub.hpp"

using namespace std;

SurgicalRobot robot;
//ik_solver_test(robot);
Chain robot_chain = robot.kuka_chain;
ChainFkSolverPos_recursive robot_fk_solver = *robot.fwdkin;
Eigen::Vector3d p_c(0.702, 0, 0.217);
Frame frame_r;
Jacobian jaco_r(10),jaco_t(10);
Eigen::Vector3d r_a;
Eigen::Vector3d r_o;
Eigen::MatrixXd B_c(3, 2), j_before(3, 6), j_RCM(2, 10), j_r(6, 10), j_t(6, 10), j_task(8, 10);
Eigen::MatrixXd j_task_inv(8, 10), j_task_mid(8, 8), j_null(10, 10), error_full(8, 1), q_delta_double(10, 1);
Eigen::Matrix3d pcpr_cross;
Eigen::Vector3d p_r, pcpr_differ;
Frame frame_t, frame_in;
Twist error_delta_twist;
Eigen::MatrixXd error_rcm(2, 1);
ChainJntToJacSolver jacSolver(robot_chain);
JntArray q_delta(10), q_answer(10), q_in(10), q_in_temp,v_out(10);
double min_error = 1e-5;
Eigen::Matrix<double, 6, 1> error_delta;
JntArray q_init(10);
Printer pri;
Frame frame_8, frame_9;
Jacobian jac_8(10), jac_9(10);
Eigen::MatrixXd jac_8_double(6, 10), jac_9_double(6, 10);

//误差增益矩阵
Eigen::VectorXd k_error(9);

//设置误差增益矩阵
void set_k_error() {
    // k_error << 10, 10, 10, 10, 10, 10, 10, 10, 10;
    k_error << 1, 1, 1, 1, 1, 1, 1, 1, 1;
    k_error =k_error*5;
}



//圆周轨迹测试
Frame circle_traj(Vector endPosi,int step) {
    double ratio = 0.02;
    Vector target_posi(endPosi.x() + ratio * sin(double(step) / 100 * 2 * M_PI),
                       endPosi.y() + ratio * cos(double(step) / 100 * 2 * M_PI),
        endPosi.z());
    Frame answer(target_posi);
    pri.print_frame(answer);
    answer.M.DoRotY(M_PI / 2);
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

//求矩阵伪逆
template<typename _Matrix_Type_> 
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon()) 
{  
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);  
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);  
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint(); 
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

void ik_solver_test(SurgicalRobot& robot) {
    ChainIkSolverPos_LMA robot_ik_solver(robot.kuka_chain);
    JntArray q_out(robot.kuka_chain.getNrOfJoints());
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

//单步逆运动学 新
JntArray invOneStep(Frame& frame_t_d, JntArray& q_origin, bool& stop_flag, double& lamda,bool &over1_flag) {
    //设置第二道线，当可操作性值过小时，直接让角度归零
    bool over2_flag = false;
    Eigen::MatrixXd j_RCM(3, 11), j_FULL(9, 11);
    Vector p_delta_ii;
    Vector p_RCM;
    Eigen::MatrixXd error_RCM(3, 1), error_FULL(9, 1);
    Eigen::MatrixXd q_delta_DOUBLE(11, 1);
    robot_fk_solver.JntToCart(q_origin, frame_t);
    robot_fk_solver.JntToCart(q_origin, frame_8, 7);
    robot_fk_solver.JntToCart(q_origin, frame_9, 8);
    // pri.print_frame(frame_8);
    // pri.print_frame(frame_9);
   

    p_delta_ii = frame_9.p - frame_8.p;
    // cout << "p_i-p_i+1: " << p_delta_ii.data[0] << " " << p_delta_ii.data[1] << " " << p_delta_ii.data[2] << endl;
    //获取 第8个关节 第九个关节 雅克比
    jacSolver.JntToJac(q_origin, jac_8, 7);
    jacSolver.JntToJac(q_origin, jac_9, 8);
    jacSolver.JntToJac(q_origin, jaco_t);
    

    //计算 RCM处 的雅可比矩阵
    jac_8_double = jac_8.data;
    jac_9_double = jac_9.data;
    j_RCM.block(0, 0, 3, 10) = (jac_8_double + lamda * (jac_9_double - jac_8_double)).block(0, 0, 3, 10);
    j_RCM.block(0, 10, 3, 1) << p_delta_ii.x(), p_delta_ii.y(), p_delta_ii.z();

    

    //计算总的雅可比矩阵
    j_FULL.block(0, 0, 6, 10) = jaco_t.data;
    j_FULL.block(0, 10, 6, 1) << 0, 0, 0, 0, 0, 0;
    j_FULL.block(6, 0, 3, 11) = j_RCM;

    //检测约束雅克比可操作性是否超出范围 over_flag将变为true，此时认为本次迭代失败
    double manipula = sqrt((j_FULL * j_FULL.transpose()).determinant());
    // cout << "----------" << manipula << "----------" << endl;
    //设置第一道线，当可操作性小于一个值时，不发送角度
    if (manipula < 0.001 && manipula > 0.0001) {
        over1_flag = true;
        stop_flag = true;
        ROS_INFO("robot is near to singular pose, manipula is : %f", manipula);
    }
    //设置第二道线，过小时直接回到初始值
    if (manipula < 0.0001) {
        over2_flag = true;
        ROS_INFO("robot is over singular pose, manipula is : %f", manipula);
    }

    // cout <<"j_FULL: "<< j_FULL << endl;

    
    p_RCM = frame_8.p + (frame_9.p - frame_8.p) * lamda;
    
    // cout << "p_RCM: " << p_RCM.data[0] << " " << p_RCM.data[1] << " " << p_RCM.data[2] << endl;

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
    
    if (error_norm < min_error) stop_flag = true;

    // cout << "jac_FULL" <<j_FULL<< endl;


    q_delta_DOUBLE = pseudoInverse(j_FULL) * error_FULL;
    // cout << pseudoInverse(j_FULL) << endl;
    // cout << "--------------------------" << endl;

    // cout << "lamda_delta: " << q_delta_DOUBLE(10) << endl;

    q_delta.data = q_delta_DOUBLE.block(0, 0, 10, 1);

    //求逆解，更新角度与 λ 
    lamda = lamda + q_delta_DOUBLE(10, 0);

    Add(q_origin, q_delta, q_answer);
    
    limit_joints(q_answer);
    
    

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

//单步逆运动学，与上面相同，但是发送速度
JntArray invVelocity(Frame& frame_t_d, JntArray& q_origin, bool& stop_flag, double& lamda,bool &over1_flag) {
    //设置第二道线，当可操作性值过小时，直接让角度归零
    bool over2_flag = false;
    Eigen::MatrixXd j_RCM(3, 11), j_FULL(9, 11);
    Vector p_delta_ii;
    Vector p_RCM;
    Eigen::MatrixXd error_RCM(3, 1), error_FULL(9, 1);
    Eigen::MatrixXd q_delta_DOUBLE(11, 1);
    robot_fk_solver.JntToCart(q_origin, frame_t);
    robot_fk_solver.JntToCart(q_origin, frame_8, 7);
    robot_fk_solver.JntToCart(q_origin, frame_9, 8);
    // pri.print_frame(frame_8);
    // pri.print_frame(frame_9);
   

    p_delta_ii = frame_9.p - frame_8.p;
    // cout << "p_i-p_i+1: " << p_delta_ii.data[0] << " " << p_delta_ii.data[1] << " " << p_delta_ii.data[2] << endl;
    //获取 第8个关节 第九个关节 雅克比
    jacSolver.JntToJac(q_origin, jac_8, 7);
    jacSolver.JntToJac(q_origin, jac_9, 8);
    jacSolver.JntToJac(q_origin, jaco_t);
    

    //计算 RCM处 的雅可比矩阵
    jac_8_double = jac_8.data;
    jac_9_double = jac_9.data;
    j_RCM.block(0, 0, 3, 10) = (jac_8_double + lamda * (jac_9_double - jac_8_double)).block(0, 0, 3, 10);
    j_RCM.block(0, 10, 3, 1) << p_delta_ii.x(), p_delta_ii.y(), p_delta_ii.z();

    

    //计算总的雅可比矩阵
    j_FULL.block(0, 0, 6, 10) = jaco_t.data;
    j_FULL.block(0, 10, 6, 1) << 0, 0, 0, 0, 0, 0;
    j_FULL.block(6, 0, 3, 11) = j_RCM;

    //检测约束雅克比可操作性是否超出范围 over_flag将变为true，此时认为本次迭代失败
    double manipula = sqrt((j_FULL * j_FULL.transpose()).determinant());
    // cout << "----------" << manipula << "----------" << endl;
    //设置第一道线，当可操作性小于一个值时，不发送角度
    // if (manipula < 0.001 && manipula > 0.0001) {
    //     over1_flag = true;
    //     stop_flag = true;
    //     ROS_INFO("robot is near to singular pose, manipula is : %f", manipula);
    // }
    // //设置第二道线，过小时直接回到初始值
    // if (manipula < 0.0001) {
    //     over2_flag = true;
    //     ROS_INFO("robot is over singular pose, manipula is : %f", manipula);
    // }

    // cout <<"j_FULL: "<< j_FULL << endl;

    
    p_RCM = frame_8.p + (frame_9.p - frame_8.p) * lamda;
    
    // cout << "p_RCM: " << p_RCM.data[0] << " " << p_RCM.data[1] << " " << p_RCM.data[2] << endl;

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
    
    if (error_norm < min_error) stop_flag = true;

    // cout << "jac_FULL" <<j_FULL<< endl;
    
    set_k_error();
    
    q_delta_DOUBLE = pseudoInverse(j_FULL) * (k_error.asDiagonal() * error_FULL);
    // cout << pseudoInverse(j_FULL) << endl;
    // cout << "--------------------------" << endl;

    // cout << "lamda_delta: " << q_delta_DOUBLE(10) << endl;

    q_delta.data = q_delta_DOUBLE.block(0, 0, 10, 1);

    //求逆解，更新角度与 λ 
    lamda = lamda + q_delta_DOUBLE(10, 0);

    // Add(q_origin, q_delta, q_answer);
    
    // limit_joints(q_answer);
    // get_jac_R_O(jaco_t.data, j_RCM);

    // pri.print_frame(frame_t);


    // pri.print_joints(q_answer);
    // cout << "lamda: " << lamda << endl;

    //当可操作性过小时，返回迭代前的角度
    // if (over2_flag) {
    //     return q_origin;
    // }
    // else {
    //     return  q_answer;
    // }
    return q_delta;

}

double get_lamda(JntArray& q_origin) {
    double lamda;
    Vector p_delta_ii;
    //获取 第8个关节 第九个关节 坐标系
    robot_fk_solver.JntToCart(q_origin, frame_8, 7);
    robot_fk_solver.JntToCart(q_origin, frame_9, 8);
    //根据坐标系原点差值、RCM点位置 计算 λ 
    p_delta_ii = frame_8.p - frame_9.p;
    
    // cout << "p_i-p_i+1: " << p_delta_ii.data[0] << " " << p_delta_ii.data[1] << " " << p_delta_ii.data[2] << endl;
    
    Vector p_delta_ic(frame_8.p.x() - p_c(0), frame_8.p.y() - p_c(1), frame_8.p.z() - p_c(2));
    lamda = p_delta_ic.Norm() / p_delta_ii.Norm();
    // cout << "lamda: " << lamda << endl;

    return lamda;
}

int main(int argc, char* argv[]){

    ros::init(argc, argv, "robot_rcm_control");
    int step = 0;
    double flag = 1e-8;
    Vector end_position(0.702, 0, 0.10);
    Frame end_pose(end_position);
    RobotSubPub sub_pub;
    sub_pub.sub_pub_init();
    end_pose.M.DoRotY(M_PI / 2);
    // pri.print_frame(end_pose);
    // q_init.data << 0, M_PI / 6, 0, -M_PI / 6, 0, 0, -M_PI / 2, M_PI / 2, -M_PI / 2, 0;
    // q_init.data << -1.87021e-16, 0.722614, 0, -0.523599, 0, 0.438865, -1.5708, 1.5708, -1.68508, -3.92534e-17;
    // q_init.data << 0, 0.523599, 0, -0.523599, 0, 0, -1.5708, 1.5708, -1.5708, 0;
    q_init.data << 0, 0.523599, 0, -0.523599, 0, 0, -1.5708, 1.5708, -1.5708, 0;
    
    q_in = q_init;
    
    frame_in = end_pose;
    pri.print_frame(frame_in);
    bool stop_flag = false;
    bool over1_flag = false;
    double lamda;

    // Frame frame_local;
    
    ros::Rate rate(200);
    // int itr_step = 0;

    while (ros::ok())
    {

        // frame_in = circle_traj(end_position, itr_step);
        frame_in = sub_pub.command_frame;
        q_in = sub_pub.joints_now;

        // pri.print_frame(frame_in);

        // robot.fwdkin->JntToCart(q_in, frame_local);

        lamda = get_lamda(q_in);
        // lamda = 0;
        
        double start = ros::Time::now().toSec();

        
        /////////////////////////////////迭代法，发送位置////////////////////////////////////////
        /**
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
        **/
        ////////////////////////////////////////////////////////////////////////////////////


        ///////////////////////////////发送速度////////////////////////////////////
        v_out = invVelocity(frame_in, q_in, stop_flag, lamda, over1_flag);
        //////////////////////////////////////////////////////////////////////////

        
        pri.print_joints(v_out);

        step++;
        
        
        
        // cout << "lamda: " << lamda << endl;

        // robot.fwdkin->JntToCart(q_in, robot.pose_end);


        // pri.print_joints(q_in);
        // print_manipula_all(q_in);
        if (!over1_flag) {
            sub_pub.pub_of_all_2(v_out);
        }
        
        over1_flag = false;
        stop_flag = false;
        step = 0;
        // itr_step++;


        ros::spinOnce();
        double endd = ros::Time::now().toSec();
        // cout << "duration: " << (endd - start) * 1000 << " ms" << endl;
        rate.sleep();
        

    }

    return 0;
}


