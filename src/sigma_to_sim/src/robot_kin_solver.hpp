#include <iostream>
#include <string>
#include <vector>
#include "surgical_robot_ur.hpp"
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "surgical_robot_pub_sub.hpp"
// #include "printer.hpp"
#include <fstream>

using namespace std;
class SurRobotKinUR {
public:
    SurgicalRobotUR* m_R;//机器人
    Chain robot_chain;//机器人链
    KDL::Frame base;//末端位姿
    ChainJntToJacSolver *jacSolver;
    ChainFkSolverPos_recursive *robot_fk_solver;


    double k_r;//可操作性指标，包括位置和姿态
    double k_o;

    Eigen::MatrixXd j_RCM, j_TIP;

    //逆运动学全局参数
    double min_error = 1e-5;
    Eigen::VectorXd k_error;


public:

    SurRobotKinUR();

    //速度转化为矩阵
    void Twist_to_Eigen(Twist& t, Eigen::Matrix<double, 6, 1>& m);
    //逆运动学计算
    void inverse_kin(Frame& frame_in, JntArray& q_in, Vector &p_c);
    JntArray invOneStep(Frame& frame_t_d, JntArray& q_origin, Vector& p_c, bool& stop_flag, double& lamda, bool& over1_flag);

    double get_lamda(JntArray & q_origin, Vector &p_c);
    Eigen::Matrix<double, 10, 10>null_space_get(Eigen::MatrixXd& j_full);
    
    template<typename _Matrix_Type_>
    _Matrix_Type_ pseudoInverse(const _Matrix_Type_& a, double epsilon = std::numeric_limits<double>::epsilon());
    Eigen::Matrix<double, 10, 3> pseudoInverse_RCM(Eigen::MatrixXd& A);
    void limit_joint_5(JntArray& q_origin, Eigen::MatrixXd& q_delta, Eigen::MatrixXd& q_NULL);
    void set_k_error();
    int get_matrix_rank(Eigen::MatrixXd& A);

    //计算可操作性指标
    vector<double> get_jac_R_O();

};

////////////////////////////////
/*                           ///
                             ///
构造函数   
                             ///
*/                           ///
////////////////////////////////
SurRobotKinUR::SurRobotKinUR() {
    m_R = new SurgicalRobotUR();
    robot_chain = m_R->ur10e_chain;

    jacSolver=new ChainJntToJacSolver(robot_chain);

    robot_fk_solver =new ChainFkSolverPos_recursive(*m_R->fwdkin);


    Eigen::MatrixXd j_RCM(3, 10), j_TIP(6, 9);

    //逆运动学参数
    k_error.resize(10);
}

////////////////////////////////
/*                           ///
                             ///
这一部分是关于运动学计算    ///
                             ///
*/                           ///
////////////////////////////////

void SurRobotKinUR::inverse_kin(Frame& frame_in, JntArray& q_in, Vector& p_c) {
    int step = 0;
    bool stop_flag;
    bool over1_flag;
    double lamda = get_lamda(q_in, p_c);
    while (step < 100) {
        q_in = invOneStep(frame_in, q_in,p_c,stop_flag, lamda, over1_flag);

        if (stop_flag) {
            break;
        }
        step++;
        if (step == 100) {
            ROS_INFO("iterate step is out of range");
        }
    }

    get_jac_R_O();

}

//速度转化为矩阵
void SurRobotKinUR::Twist_to_Eigen(Twist& t, Eigen::Matrix<double, 6, 1>& m) {
    m(0) = t.vel.data[0];
    m(1) = t.vel.data[1];
    m(2) = t.vel.data[2];
    m(3) = t.rot.data[0];
    m(4) = t.rot.data[1];
    m(5) = t.rot.data[2];
}

double SurRobotKinUR::get_lamda(JntArray& q_origin, Vector& p_c) {
    double lamda;
    Vector p_delta_ii;
    Frame frame_7;
    Frame frame_8;
    //获取 第8个关节 第九个关节 坐标系
    robot_fk_solver->JntToCart(q_origin, frame_7, 6);
    robot_fk_solver->JntToCart(q_origin, frame_8, 7);
    //根据坐标系原点差值、RCM点位置 计算 λ 
    p_delta_ii = frame_7.p - frame_8.p;

    // cout << "p_i-p_i+1: " << p_delta_ii.data[0] << " " << p_delta_ii.data[1] << " " << p_delta_ii.data[2] << endl;

    Vector p_delta_ic(frame_7.p.x() - p_c(0), frame_7.p.y() - p_c(1), frame_7.p.z() - p_c(2));
    lamda = p_delta_ic.Norm() / p_delta_ii.Norm();
    // cout << "lamda: " << lamda << endl;

    return lamda;
}
//求矩阵的秩
int SurRobotKinUR::get_matrix_rank(Eigen::MatrixXd &A) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
    // std::cout<<"A :\n"<<A<<std::endl;
    // std::cout << "rank:\n" << svd.rank() << std::endl;
    // std::cout << "svd value:\n" << A.eigenvalues() << endl;
    return svd.rank();
}

//计算雅可比矩阵零空间矩阵
Eigen::Matrix<double, 10, 10>SurRobotKinUR::null_space_get(Eigen::MatrixXd& j_full) {
    Eigen::MatrixXd null_matrix(10, 10);
    null_matrix.setIdentity();
    // cout << j_full << endl;
    // cout << pseudoInverse(j_full) << endl;
    null_matrix = null_matrix - pseudoInverse(j_full) * j_full;
    get_matrix_rank(null_matrix);
    return null_matrix;
}

//求矩阵伪逆
// template<typename _Matrix_Type_>
// _Matrix_Type_ SurRobotKinUR::pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon()) 
// {  
//     Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);  
//     double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);  
//     return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint(); 
// }
template<typename _Matrix_Type_>
_Matrix_Type_ SurRobotKinUR::pseudoInverse(const _Matrix_Type_ &a, double epsilon) 
{  
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);  
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);  
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint(); 
}


//求RCM矩阵伪逆(行满秩,右伪逆)
Eigen::Matrix<double, 10, 3> SurRobotKinUR::pseudoInverse_RCM(Eigen::MatrixXd& A) {
    return A.transpose() * (A * A.transpose()).inverse();
}

//约束第五个关节，当超出时在零空间施加反向角度进行约束
void SurRobotKinUR::limit_joint_5(JntArray& q_origin, Eigen::MatrixXd& q_delta, Eigen::MatrixXd& q_NULL) {
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

//设置误差增益矩阵
void SurRobotKinUR::set_k_error() {
    // k_error << 10, 10, 10, 10, 10, 10, 10, 10, 10;
    k_error << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,1.0;
    k_error =k_error*1.0;
}

//逆运动学主函数
JntArray SurRobotKinUR::invOneStep(Frame& frame_t_d, JntArray& q_origin, Vector& p_c, bool& stop_flag, double& lamda, bool& over1_flag) {

    Frame frame_t;
    Frame frame_7;
    Frame frame_8;
    Jacobian jac_7;
    Jacobian jac_8;
    Jacobian jac_t;
    Eigen::MatrixXd jac_7_double(6, 9), jac_8_double(6, 9);
    Twist error_delta_twist;
    Eigen::Matrix<double, 6, 1> error_delta;
    JntArray q_delta(9), q_answer(9);

    Eigen::MatrixXd j_FULL(9, 10);
    
    Vector p_RCM;
    Eigen::MatrixXd error_RCM(3, 1), error_FULL(9, 1), null_motion(10, 1);
    null_motion << 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0;//零空间运动初始值
    
    Eigen::MatrixXd q_delta_DOUBLE_up(10, 1),q_delta_DOUBLE_down(10, 1),q_delta_DOUBLE(10,1),q_NULL(10,1);

    robot_fk_solver->JntToCart(q_origin, frame_t);
    robot_fk_solver->JntToCart(q_origin, frame_7, 6);
    robot_fk_solver->JntToCart(q_origin, frame_8, 7);
    // pri.print_frame(frame_t);
    // pri.print_frame(frame_9);
    
    Vector p_delta_ii;
    p_delta_ii = frame_8.p - frame_7.p;
    // cout << "p_i-p_i+1: " << p_delta_ii.data[0] << " " << p_delta_ii.data[1] << " " << p_delta_ii.data[2] << endl;
    //获取 第8个关节 第九个关节 雅克比
    jacSolver->JntToJac(q_origin, jac_7, 6);
    jacSolver->JntToJac(q_origin, jac_8, 7);
    jacSolver->JntToJac(q_origin, jac_t);
    
    //计算 RCM处 的雅可比矩阵
    jac_7_double = jac_7.data;
    
    jac_8_double = jac_8.data;

    

    j_RCM.block(0, 0, 3, 9) = (jac_7_double + lamda * (jac_8_double - jac_7_double)).block(0, 0, 3, 9);
    j_RCM.block(0, 9, 3, 1) << p_delta_ii.x(), p_delta_ii.y(), p_delta_ii.z();
    // ROS_INFO("flag!");

    
    //计算总的雅可比矩阵
    j_FULL.block(0, 0, 6, 9) = jac_t.data;
    j_FULL.block(0, 9, 6, 1) << 0, 0, 0, 0, 0, 0;
    j_FULL.block(6, 0, 3, 10) = j_RCM;

    j_TIP = jac_t.data;


    
    //检测约束雅克比可操作性是否超出范围 over_flag将变为true，此时认为本次迭代失败
    double manipula = sqrt((j_FULL * j_FULL.transpose()).determinant());
    // cout << "----------manipula: " << manipula << "----------" << endl;
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
    
    return  q_answer;

    
}

////////////////////////////////
/*                           ///
                             ///
这一部分是关于可操作性计算    ///
                             ///
*/                           ///
////////////////////////////////

//计算reach可操作性和orient可操作性
vector<double> SurRobotKinUR::get_jac_R_O() {
    
    Eigen::MatrixXd ident(10, 10), jac_double_v(3, 10), jac_double_w(3, 10), jac_R(3, 10), jac_O(3, 10);
    ident.setIdentity();
    jac_double_v.block(0, 0, 3, 9) = j_TIP.block(0, 0, 3, 9);
    jac_double_v.block(0, 9, 3, 1) << 0, 0, 0;
    jac_double_w.block(0, 0, 3, 9) = j_TIP.block(3, 0, 3, 9);
    jac_double_w.block(0, 9, 3, 1) << 0, 0, 0;

    // cout << ident - pseudoInverse(jac_double_w) * jac_double_w << endl;
    // cout << pseudoInverse_RCM(jac_RCM) << endl;


    //计算可达可操作性与方向可操作性
    jac_R = jac_double_v * (ident - pseudoInverse(jac_double_w) * jac_double_w) * (ident - pseudoInverse_RCM(j_RCM) * j_RCM);
    jac_O = jac_double_w * (ident - pseudoInverse(jac_double_v) * jac_double_v) * (ident - pseudoInverse_RCM(j_RCM) * j_RCM);
    // cout << (jac_R * jac_R.transpose()).inverse() << endl;
    //计算可操作性系数
    double k_r = sqrt((jac_R * jac_R.transpose()).trace() * (jac_R * jac_R.transpose()).inverse().trace());
    double k_o = sqrt((jac_O * jac_O.transpose()).trace() * (jac_O * jac_O.transpose()).inverse().trace());

    // cout << "k_r: " << 1 / k_r << " k_o: " << 1 / k_o << endl;
    
    // cout << "------------------------" << endl;

}
