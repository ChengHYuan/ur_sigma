#include <iostream>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <vector>
#include <kdl/chainfksolver.hpp>

using namespace std;
using namespace KDL;

class Printer {
public:
    //打印雅可比矩阵
    void print_jac(Jacobian jac) {
        for (int i = 0;i < jac.data.rows();++i) {
            for (int j = 0;j < jac.data.cols();++j) {
                cout << jac.data(i, j) << " ";
            }
            cout << endl;
        }
        cout << "---------------------------------------------" << endl;
    }
    //打印关节角
    void print_joints(JntArray joints) {
        for (int i = 0;i < joints.data.size();++i) {
            cout << joints.data(i) << " ";
        }
        cout << endl;
        cout << "---------------------------------------------" << endl;
    }
    //打印坐标系
    void print_frame(Frame frame) {
        cout << "pose: " << endl;
        cout << frame.M.data[0] << " " << frame.M.data[1] << " " << frame.M.data[2] << " " << frame.p[0] << " " << endl;
        cout << frame.M.data[3] << " " << frame.M.data[4] << " " << frame.M.data[5] << " " << frame.p[1] << " " << endl;
        cout << frame.M.data[6] << " " << frame.M.data[7] << " " << frame.M.data[8] << " " << frame.p[2] << " " << endl;
        cout << "-----------------------------------" << endl;
    }
    //打印所有配置
    void print_config(vector<pair<vector<bool>, double>> config) {
        for (int i = 0;i < config.size();++i) {
            for (int j = 0;j < config[i].first.size();++j) {
                cout << config[i].first[j] << " ";
            }
            cout << "manipula:" << config[i].second << " " << endl;
        }
        cout << "---------------------------------" << endl;
    }
};




