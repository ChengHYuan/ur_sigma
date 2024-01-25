#include <iostream>
#include <string>
#include <vector>
#include "printer.hpp"
#include <fstream>
#include <random>
#include <algorithm>
#include "robot_kin_solver.hpp"



using namespace std;
class InsertOpti {
public:
    SurRobotKinUR* m_RL;//机器人左手
    SurRobotKinUR* m_RR;//机器人右手
    SurRobotKinUR* m_RE;//机器人腹腔镜
    
    vector<vector<double>> m_invert_points;//插入点阵列
    vector<Frame>m_space_points;//空间点阵列

    KDL::Frame baseL;
    KDL::Frame baseR;
    KDL::Frame baseE;
    

    int num_space;//参与遍历的空间中点的数量

    int num_joint;
    int SIZE;

    vector<double> joint_limit;

    vector<double> reach_params;
    vector<double> orient_params;
    vector<double> joint_lim_params;

    
    //初始化
    InsertOpti();
    
    //读取文件并记录所有离散插入点的位置
    void read_insert_points(const std::string& filename);
    void read_space_pose(const std::string& filename);
    void printPoints(vector<vector<double>> &vec);

    //根据插入点、工作空间计算几个指标
    double get_reach_param();//位置灵巧性l
    double get_orient_param();//姿态灵巧性l
    double get_joint_lim_param(JntArray& joints);//关节角限制参数
    double get_symmetry_param(vector<double>rcm_l, vector<double>rcm_r, vector<double>rcm_e);//对称性参数
    double get_distances_param(vector<double>rcm_l, vector<double>rcm_r, vector<double>rcm_e);//插入点距离参数

    double get_Unit(const vector<int> & individual);//综合参数
    void get_all_param_independent();//实验之前计算所有的与插入点无关的参数

    //评估参数的优劣,返回带权重的参数
    double judge_reach_param(double &reach);
    double judge_orient_param(double &orient);
    double judge_joint_lim_param(double &lim);
    double judge_symmetry_param(double &symmetry);
    double judge_distances_param(double &dis);
    

};

////////////////////////////////
/*                           ///
                             ///
构造函数   
                             ///
*/                           ///
////////////////////////////////
InsertOpti::InsertOpti() {
    m_RL = new SurRobotKinUR();
    m_RR = new SurRobotKinUR();
    m_RE = new SurRobotKinUR();


    read_insert_points("data1.txt");
    read_space_pose("data2.txt");

    num_joint = 9;
    
    SIZE = m_invert_points.size();
    
    reach_params.resize(SIZE);
    orient_params.resize(SIZE);
    joint_lim_params.resize(SIZE);

    Eigen::MatrixXd j_RCM(3, 10), j_TIP(6, 9);
    joint_limit = { 360,   360,    360,    360,    360,    360,    360, 80, 80 };
}

//读取插入点
void InsertOpti::read_insert_points(const string& filename) {
    ifstream file(filename);
        if (!file) {
            cerr << "无法打开文件" << endl;
            return;
        }
        
        string line;
        while (getline(file, line)) {
            istringstream iss(line);
            vector<double> point;
            string value;
            while (iss >> value) {
                if (isdigit(value[0])) {
                    double num = stod(value);
                    point.push_back(num);
                }
            }
            m_invert_points.push_back(point);
        }
        file.close();
    
}

//读取空间点
void InsertOpti::read_space_pose(const string& filename) {
    ifstream file(filename);
        if (!file) {
            cerr << "无法打开文件" << endl;
            return;
        }
        
        string line;
        while (getline(file, line)) {
            istringstream iss(line);
            vector<double> point;
            string value;
            while (iss >> value) {
                if (isdigit(value[0])) {
                    double num = stod(value);
                    point.push_back(num);
                }
            }

            Vector position(point[0], point[1], point[2]);
            Rotation orientation = Rotation::Quaternion(point[6], point[3], point[4], point[5]);
            Frame frame(orientation, position);
            m_space_points.push_back(frame);
        }
        file.close();
}

//打印插入点
void InsertOpti::printPoints(vector<vector<double>> &vec) {
        cout << "导入的点的内容：" << endl;
        for (const auto& point : vec) {
            for (int num : point) {
                cout << num << " ";
            }
            cout << endl;
        }
    }


//计算单次空间点的关节角参数
double InsertOpti::get_joint_lim_param(JntArray& joints) {
    double f_theta;
    double lim_param = 0;
    for (int i = 0;i < num_joint;++i) {
        f_theta = (joints.data(i)) * (joints.data(i)) / (joint_limit[i] * joint_limit[i]);
        lim_param += f_theta;
    }
    return lim_param / num_joint;
}

//计算对称参数
double InsertOpti::get_symmetry_param(vector<double>rcm_l, vector<double>rcm_r, vector<double>rcm_e) {//越小越好
    double el = sqrt((rcm_l[0] - rcm_e[0]) * (rcm_l[0] - rcm_e[0]) + (rcm_l[1] - rcm_e[1]) * (rcm_l[1] - rcm_e[1]) + (rcm_l[2] - rcm_e[2]) * (rcm_l[2] - rcm_e[2]));
    double er = sqrt((rcm_r[0] - rcm_e[0]) * (rcm_r[0] - rcm_e[0]) + (rcm_r[1] - rcm_e[1]) * (rcm_r[1] - rcm_e[1]) + (rcm_r[2] - rcm_e[2]) * (rcm_r[2] - rcm_e[2]));
    return (el - er) * (el - er);
}
//计算插入点距离参数（不能太近）
double InsertOpti::get_distances_param(vector<double>rcm_l, vector<double>rcm_r, vector<double>rcm_e) {
    double min_distance=0.03;
    double el = sqrt((rcm_l[0] - rcm_e[0]) * (rcm_l[0] - rcm_e[0]) + (rcm_l[1] - rcm_e[1]) * (rcm_l[1] - rcm_e[1]) + (rcm_l[2] - rcm_e[2]) * (rcm_l[2] - rcm_e[2]));
    double er = sqrt((rcm_r[0] - rcm_e[0]) * (rcm_r[0] - rcm_e[0]) + (rcm_r[1] - rcm_e[1]) * (rcm_r[1] - rcm_e[1]) + (rcm_r[2] - rcm_e[2]) * (rcm_r[2] - rcm_e[2]));
    double lr = sqrt((rcm_r[0] - rcm_l[0]) * (rcm_r[0] - rcm_l[0]) + (rcm_r[1] - rcm_l[1]) * (rcm_r[1] - rcm_l[1]) + (rcm_r[2] - rcm_l[2]) * (rcm_r[2] - rcm_l[2]));
    return 2*min_distance / (el)+2*min_distance / (er)+2*min_distance / (lr);
}


//可以利用左右手等价的原理，在开始遗传算法之前就把所有的与空间相关的参数全部计算完
double InsertOpti::get_reach_param() {
    
}

double InsertOpti::get_orient_param() {
    
}

//评估参数的优劣,返回带权重的参数
double InsertOpti::judge_reach_param(double& reach) {
    double result = 0.0;
    double value = reach;
    if (value <= 10.0) {
        result = value * 1.0; // 理想
    } else if (value <= 100.0) {
        result = value * 10.0; // 可接受
    } else {
        result = value * 100.0; // 不可接受
    }

    return result;
}

double InsertOpti::judge_orient_param(double& orient) {
    double result = 0.0;
    double value = orient;
    if (value <= 10.0) {
        result = value * 1.0; // 理想
    } else if (value <= 100.0) {
        result = value * 10.0; // 可接受
    } else {
        result = value * 100.0; // 不可接受
    }

    return result;
}

double InsertOpti::judge_joint_lim_param(double& lim) {
    double result = 0.0;
    double value = lim;
    if (value <= 10.0) {
        result = value * 1.0; // 理想
    } else if (value <= 100.0) {
        result = value * 10.0; // 可接受
    } else {
        result = value * 100.0; // 不可接受
    }

    return result;
}

double InsertOpti::judge_symmetry_param(double& symmetry) {
    double result = 0.0;
    double value = symmetry;
    if (value <= 10.0) {
        result = value * 1.0; // 理想
    } else if (value <= 100.0) {
        result = value * 10.0; // 可接受
    } else {
        result = value * 100.0; // 不可接受
    }

    return result;
}

double InsertOpti::judge_distances_param(double& dis) {
    double result = 0.0;
    double value = dis;
    if (value <= 10.0) {
        result = value * 1.0; // 理想
    } else if (value <= 100.0) {
        result = value * 10.0; // 可接受
    } else {
        result = value * 100.0; // 不可接受
    }

    return result;
}

void InsertOpti::get_all_param_independent() {
    JntArray joints_now(9);
    //遍历所有插入点

    for (int i = 0;i < m_invert_points.size(); ++i) {
        Vector p_rcm(m_invert_points[i][0], m_invert_points[i][1], m_invert_points[i][2]);
        double reach_param = 0.0;
        double orient_param = 0.0;
        double joint_lim_param = 0.0;
        //遍历所有空间点
        for (int j = 0;j < m_space_points.size();++j) {
            //计算各个指标

            Frame frame_now = m_space_points[j];
            m_RL->inverse_kin(frame_now, joints_now, p_rcm);

            reach_param += get_reach_param();
            orient_param += get_orient_param();
            joint_lim_param += get_joint_lim_param(joints_now);
            //各个单个指标求和
        }
        reach_param = reach_param / num_space;
        orient_param = orient_param / num_space;
        joint_lim_param = joint_lim_param / num_space;

        reach_params[i] = reach_param;
        orient_params[i] = orient_param;
        joint_lim_params[i] = joint_lim_param;
    }

}

double InsertOpti::get_Unit(const vector<int> & individual) {

    //根据索引获得插入点坐标
    vector<double> rcm_l = m_invert_points[individual[0]];
    vector<double> rcm_r = m_invert_points[individual[1]];
    vector<double> rcm_e = m_invert_points[individual[2]];


    //计算当前插入点下的各个参数值，并进行接收
    double RL = reach_params[individual[0]];
    double OL = orient_params[individual[0]];
    double RR = reach_params[individual[1]];
    double OR = orient_params[individual[0]];
    double JntLimL = joint_lim_params[individual[0]];
    double JntLimR = joint_lim_params[individual[1]];
    double Sym = get_symmetry_param(rcm_l, rcm_r, rcm_e);
    double Dis = get_distances_param(rcm_l,rcm_r,rcm_e);
    //给权重，这个权重基于三个档位：理想、可接受、不可接受
    RL = judge_reach_param(RL);
    OL = judge_orient_param(OL);
    RR = judge_reach_param(RR);
    OR = judge_orient_param(OR);
    JntLimL = judge_joint_lim_param(JntLimL);
    JntLimR = judge_joint_lim_param(JntLimR);
    Sym = judge_symmetry_param(Sym);
    Dis = judge_distances_param(Dis);


}

