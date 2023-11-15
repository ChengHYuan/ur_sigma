//
// Created by jiyguo on 2022/3/23.
//

#ifndef SRC_MY_DMP_H
#define SRC_MY_DMP_H

#include "dmp/DMPData.h"
#include "dmp/DMPTraj.h"
#include "dmp/radial_approx.h"
#include "tinyxml2.h"

using namespace dmp;
using namespace tinyxml2;
using std::string;

namespace myDmp{

typedef std::vector<double> DVector;
typedef std::vector<DMPData> DmpVector;
double distance(const DVector& x , const DVector& y);

class MyDmp {
private:
    int _dims;
    double _tau{};
    double _dt;
    int _num_bases{};
    double _base_width;
    DmpVector _dmp_list;
    std::vector<FunctionApprox*> _f;
    DVector _x_0;
    DVector _x_dot_0;
    DVector _goal;
    DVector _goal_thresh;
    double _seg_length;
    double CalcPhase(double curr_time) const;
    bool LearnFromDemo(const DMPTraj &demo,
                       const DVector &k_gains,
                       const DVector &d_gains,
                       const int &num_bases);
    bool LoadFromParam(const string& dmp_file);

public:
    explicit MyDmp(const DVector& x_0,const DVector& goal,
                   const DVector& x_dot_0, const DVector& goal_thresh, int num_bases = 15,
                   const double& dt = 0.01,const double& tau = 10,
                   const double& seg_length = 20 );
    ~MyDmp();

    bool Init(const DMPTraj &demo,
              const DVector &k_gains,
              const DVector &d_gains,
              const int &num_bases);
    bool Init(const string& dmp_file);

    void set_tau(const double& tau){_tau = tau;}
    void set_dt(const double& dt){_dt = dt;}
    void set_goal(const DVector& goal){_goal = goal;}

    void get_tau(double& tau) const{tau = _tau;}
    void get_dgain(DVector& dgain) const{
        dgain.resize(_dims);
        for (int i = 0; i < _dims; ++i) {
            dgain[i] = _dmp_list[i].d_gain;
        }
    }

    bool get_plan(DMPTraj &plan,double& total_len,const double& cur_t=0) const;

    bool get_step(const DMPPoint& cur_state,const DVector& couple_term,
                 const double& cur_t, DMPPoint &plan) const;
    bool get_step(const DMPPoint& cur_state, const double& cur_t, DMPPoint &plan) const;

    bool get_force(const DMPPoint& cur_state,const DVector& couple_term,
                   const double& cur_t, DVector& force_dmp) const;

    bool get_force(const DMPPoint& cur_state, const double& cur_t, DVector& force_dmp) const;
};


}


#endif //SRC_MY_DMP_H
