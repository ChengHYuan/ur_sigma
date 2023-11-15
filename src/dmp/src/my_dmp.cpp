//
// Created by jiyguo on 2022/3/23.
//

#include "my_dmp.h"

namespace myDmp {
    const int MAX_PLAN_LENGTH = 1000;
    const double alpha = -log(0.01); //Ensures 99% phase convergence at t=tau

    MyDmp::MyDmp(const DVector& x_0 , const DVector& goal,
                 const DVector& x_dot_0 , const DVector& goal_thresh,
                 const int num_bases, const double& dt ,const double& tau ,
                 const double& seg_length ){
        this->_x_0 = x_0;
        this->_goal = goal;
        this->_x_dot_0 = x_dot_0;
        this->_goal_thresh = goal_thresh;
        this->_dt = dt;
        this->_seg_length = seg_length;
        this->_dims = x_0.size();
        this->_num_bases = num_bases;
        this->_tau = tau;
        _dmp_list.resize(_dims);
        _f.resize(_dims);
        _base_width = pow(_num_bases,1.5)*alpha;
        for (int i = 0; i < _dims; ++i) {
            _f[i] = new RadialApprox(_num_bases, _base_width, alpha);
        }
    }

    bool MyDmp::Init(const DMPTraj &demo,
              const DVector &k_gains,
              const DVector &d_gains,
              const int &num_bases){
        return LearnFromDemo(demo,k_gains,d_gains,num_bases);
    }
    bool MyDmp::Init(const string& dmp_file){
        return LoadFromParam(dmp_file);
    }
    /**
     * @brief Given a single demo trajectory, produces a multi-dim DMP
     * @param[in] demo An n-dim demonstration trajectory
     * @param[in] k_gains A proportional gain for each demo dimension
     * @param[in] d_gains A vector of differential gains for each demo dimension
     * @param[in] n_bases The number of basis functions to use for the fxn approx (i.e. the order of the Fourier series)
     */
    bool MyDmp::LearnFromDemo(const DMPTraj &demo,
                              const DVector &k_gains,
                              const DVector &d_gains,
                              const int &num_bases)
    {
        //Determine traj length and dim
        size_t n_pts = demo.points.size();
        if(n_pts < 1){
            ROS_ERROR("Empty trajectory passed to learn_dmp_from_demo service!");
            return false;
        }
        double tau = demo.times[n_pts-1];
        _num_bases = num_bases;

        double *x_demo = new double[n_pts];
        double *v_demo = new double[n_pts];
        double *v_dot_demo = new double[n_pts];
        double *f_domain = new double[n_pts];
        double *f_targets = new double[n_pts];

        double base_width = pow(_num_bases,1.5)*alpha;
        //Compute the DMP weights for each DOF separately
        for(int d=0; d<_dims; d++){
            double curr_k = k_gains[d];
            double curr_d = d_gains[d];
            double x_0 = demo.points[0].positions[d];
            double goal = demo.points[n_pts-1].positions[d];
            x_demo[0] = demo.points[0].positions[d];
            v_demo[0] = 0;
            v_dot_demo[0] = 0;

            //Calculate the demonstration v and v dot by assuming constant acceleration over a time period
            for(int i=1; i<n_pts; i++){
                x_demo[i] = demo.points[i].positions[d];
                double dx = x_demo[i] - x_demo[i-1];
                double dt = demo.times[i] - demo.times[i-1];
                v_demo[i] = dx/dt;
                v_dot_demo[i] = (v_demo[i] - v_demo[i-1]) / dt;
            }

            //Calculate the target pairs so we can solve for the weights
            for(int i=0; i<n_pts; i++){
                double phase = CalcPhase(demo.times[i]);
                f_domain[i] = demo.times[i]/tau;  //Scaled time is cleaner than phase for spacing reasons
                f_targets[i] = ((tau*tau*v_dot_demo[i] + curr_d*tau*v_demo[i]) / curr_k) - (goal-x_demo[i]) + ((goal-x_0)*phase);
                f_targets[i] /= phase; // Do this instead of having fxn approx scale its output based on phase
            }

            //Solve for weights
            _f[d]->leastSquaresWeights(f_domain, f_targets, n_pts);

            //Create the DMP structures
            DMPData *curr_dmp = new DMPData();
            curr_dmp->weights = _f[d]->getWeights();
            curr_dmp->k_gain = curr_k;
            curr_dmp->d_gain = curr_d;
            for(int i=0; i<n_pts; i++){
                curr_dmp->f_domain.push_back(f_domain[i]);
                curr_dmp->f_targets.push_back(f_targets[i]);
            }
            _dmp_list[d] = *curr_dmp;
            _tau = tau;
        }

        delete[] x_demo;
        delete[] v_demo;
        delete[] v_dot_demo;
        delete[] f_domain;
        delete[] f_targets;
        return true;
    }

    /**
 * @brief 从xml文件中导入dmp参数
 * @param[in] dmp_file 文件路径
 */
    // 从xml文件中导入dmp参数
    bool MyDmp::LoadFromParam(const std::string& dmp_file){
        XMLDocument doc;
        XMLError state = doc.LoadFile(dmp_file.c_str());
        if (state!=tinyxml2::XML_SUCCESS)
        {
            throw ;
        }
        XMLElement* xml_root = doc.RootElement();
        string dim_str = xml_root->FirstAttribute()->Value();
        XMLElement* tau_element=xml_root->FirstChildElement("tau");
        string tau_str = tau_element->GetText();
        _tau = stof(tau_str);
        XMLElement *dmp_element = xml_root->FirstChildElement("dmp");
        int idx = 0;
        while (dmp_element)
        {
            XMLElement *kgain_element = dmp_element->FirstChildElement("kgain");
            string kgain_str = kgain_element->GetText();
            _dmp_list[idx].k_gain = stof(kgain_str);
            XMLElement *dgain_element = dmp_element->FirstChildElement("dgain");
            string dgain_str = dgain_element->GetText();
            _dmp_list[idx].d_gain = stof(dgain_str);
            XMLElement *weights_element = dmp_element->FirstChildElement("weights");
            string nums_weight_str = weights_element->FirstAttribute()->Value();
            DVector weights(stoi(nums_weight_str));
            XMLElement *w_element = weights_element->FirstChildElement("w");
            for (int i = 0; i < weights.size(); ++i) {
                string w = w_element->GetText();
                weights[i] = stof(w);
                w_element=w_element->NextSiblingElement();
            }
            _dmp_list[idx].weights = weights;
            _f[idx]->setWeights(weights);
            dmp_element = dmp_element->NextSiblingElement();
            idx++;
        }
        return true;
    }


    /**
     * @brief Use the current active multi-dim DMP to create a plan starting from x_0 toward a goal
     * @param[in] cur_t The current time resolution of the plan
     * @param[out] plan An n-dim plan starting from x_0
     */
    bool MyDmp::get_plan(DMPTraj &plan,double& total_len,const double& cur_t) const{
        plan.points.clear();
        plan.times.clear();
        bool at_goal = false;
        int n_pts = 0;
        DVector *x_vecs, *x_dot_vecs;
        DVector t_vec;
        x_vecs = new DVector[_dims];
        x_dot_vecs = new DVector[_dims];
        double t = 0;
        double f_eval;
        //Plan for at least tau seconds.  After that, plan until goal_thresh is satisfied.
        //Cut off if plan exceeds MAX_PLAN_LENGTH seconds, in case of overshoot / oscillation
        //Only plan for seg_length seconds if specified
        bool seg_end = false;
        while(((t+cur_t) < _tau || (!at_goal && t<MAX_PLAN_LENGTH)) && !seg_end){
            //Check if we've planned to the segment end yet
            if(_seg_length > 0){
                if (t > _seg_length) seg_end = true;
            }
            //Plan in each dimension
            for(int i=0; i<_dims; i++){
                double x,v;
                if(n_pts==0){
                    x = _x_0[i];
                    v = _x_dot_0[i];
                }
                else{
                    x = x_vecs[i][n_pts-1];
                    v = x_dot_vecs[i][n_pts-1] * _tau;
//                    v = x_dot_vecs[i][n_pts-1] ;
                }
                //Compute the phase and the log of the phase to assist with some numerical issues
                //Then, evaluate the function approximator at the log of the phase
                double s = CalcPhase(t+cur_t+_dt);
                double log_s = (t+cur_t)/_tau;
                if(log_s >= 1.0){
                    f_eval = 0;
                }
                else{
                    f_eval = _f[i]->evalAt(log_s) * s;
                }
                //Update v dot and x dot based on DMP differential equations
                double v_dot = (_dmp_list[i].k_gain*((_goal[i]-x) - (_goal[i]-_x_0[i])*s + f_eval) - _dmp_list[i].d_gain*v) / _tau;
                double x_dot = v/_tau;
//                double x_dot = v;

                //Update state variables
                v += v_dot * _dt;
                x += x_dot * _dt;
                //Add current state to the plan
                x_vecs[i].push_back(x);
                x_dot_vecs[i].push_back(v/_tau);
//                x_dot_vecs[i].push_back(v);
            }
            t += _dt;
            t_vec.push_back(t);
            n_pts++;
            //If plan is at least minimum length, check to see if we are close enough to goal
            if((t+cur_t) >= _tau){
                at_goal = true;
                for(int i=0; i<_dims; i++){
                    if(_goal_thresh[i] > 0){
                        if(fabs(x_vecs[i][n_pts-1] - _goal[i]) > _goal_thresh[i])
                            at_goal = false;
                    }
                }
            }
        }
        //Create a plan from the generated trajectories
        plan.points.resize(n_pts);
        for(int j=0; j<n_pts; j++){
            plan.points[j].positions.resize(_dims);
            plan.points[j].velocities.resize(_dims);
        }
        for(int i=0; i<_dims; i++){
            for(int j=0; j<n_pts; j++){
                if (j!=0){
                    total_len += distance(plan.points[j].positions , plan.points[j-1].positions);
                }
                plan.points[j].positions[i] = x_vecs[i][j];
                plan.points[j].velocities[i] = x_dot_vecs[i][j];
            }
        }
        plan.times = t_vec;
        //Clean up
        delete[] x_vecs;
        delete[] x_dot_vecs;
        return at_goal;
    }

    /**
     * @brief 单步规划
     * @param[in] cur_state The current state of robot ee
     * @param[in] couple_term couple term from GMM workspace constraint
     * @param[in] cur_t The current execution time
     * @param[out] plan Next step planned
     */
    bool MyDmp::get_step(const DMPPoint &cur_state, const DVector &couple_term, const double& cur_t, DMPPoint &plan) const{
        plan.positions.clear();
        plan.velocities.clear();
        bool at_goal = false;
        double f_eval;
        //Plan in each dimension
        double v_dot;
        double x_dot;
        double x,v;

        Eigen::Vector3d couple_term_vec;
        couple_term_vec << couple_term[0],couple_term[1],0;
        Eigen::Vector3d v_vec;
        v_vec << cur_state.velocities[0],cur_state.velocities[1],0;
        bool if_couple = true;
        if(couple_term_vec.dot(v_vec) > 0){
            if_couple = false;
        }

        for(int i=0; i<_dims; i++){
            x = cur_state.positions[i];
            v = cur_state.velocities[i] * _tau;
            //Compute the phase and the log of the phase to assist with some numerical issues
            //Then, evaluate the function approximator at the log of the phase
            double s = CalcPhase(cur_t);
            double log_s = cur_t/_tau;
            if(log_s >= 1.0){
                f_eval = 0;
            }
            else{
                f_eval = _f[i]->evalAt(log_s) * s;
            }

            //Update v dot and x dot based on DMP differential equations
            v_dot = (_dmp_list[i].k_gain*((_goal[i]-x) - (_goal[i]-_x_0[i])*s + f_eval) - _dmp_list[i].d_gain*v) / _tau ;
//                std::cout<<"v_dots: "<<v_dot<<std::endl;

            if(if_couple){
                v_dot += couple_term[i];
            }

            x_dot = v/_tau ;

            //Update state variables
            v += v_dot * _dt;
            x += x_dot * _dt;

            plan.positions.push_back(x);
            plan.velocities.push_back(v/_tau);
        }


        //If plan is at least minimum length, check to see if we are close enough to goal
        if(cur_t >= _tau){
            at_goal = true;
            for(int i=0; i<_dims; i++){
                if(_goal_thresh[i] > 0){
                    if(fabs(cur_state.positions[i] - _goal[i]) > _goal_thresh[i])
                        at_goal = false;
                }
            }
        }
        return at_goal;
    }


    bool MyDmp::get_step(const DMPPoint &cur_state, const double &cur_t, DMPPoint &plan) const{
        DVector couple_term(3,0);
        return get_step(cur_state,couple_term,cur_t,plan);
    }

    /**
     * @brief 公式5-18，以为当前dmp为机器人的输入时，以admittance control的表达形式推导出来的force，
     * @param[in] cur_state The current state of robot ee
     * @param[in] couple_term couple term from GMM workspace constraint
     * @param[in] cur_t The current execution time
     * @param[out] force_dmp force项
     */
    bool MyDmp::get_force(const DMPPoint &cur_state, const DVector &couple_term,
                          const double& cur_t, DVector& force_dmp) const{
//        std::cout<<"cur_t: "<<cur_t<<std::endl;
        if(cur_state.positions.size()!=_dims){
            throw ;
        }
        force_dmp.resize(_dims);
        bool at_goal = true;
        double f_eval;
        double x;
        //Compute the phase and the log of the phase to assist with some numerical issues
        //Then, evaluate the function approximator at the log of the phase
        double s = CalcPhase(cur_t);
        double log_s = cur_t/_tau;
//        std::cout<<"s: "<<s<<std::endl;
//        std::cout<<"log_s: "<<log_s<<std::endl;

        for(int i=0; i<_dims; i++){
            x = cur_state.positions[i];
            if(log_s >= 1.0){
                f_eval = 0;
            }
            else{
                f_eval = _f[i]->evalAt(log_s) * s;
            }
//            std::cout<<"f_eval: "<<f_eval<<std::endl;

//            ROS_INFO("~~~~~~~ BEFOR force_dmp ~~~~~~~");
            // Get force term in admittance control model
            force_dmp[i] = _dmp_list[i].k_gain*((_goal[i]-x) - (_goal[i]-_x_0[i])*s + f_eval) ;
            // If angle between direction of velocity and couple term
            Eigen::Vector3d v_vec;
            v_vec << cur_state.velocities[0],cur_state.velocities[1],0;
            Eigen::Vector3d couple_term_vec;
            for (int j = 0; j < couple_term.size(); ++j) {
                couple_term_vec(j) = couple_term[j];
            }
            if( couple_term_vec.dot(v_vec) > 0 ){
                force_dmp[i] += couple_term[i];
            }
            force_dmp[i] /= pow(_tau,2);
//            std::cout<<"force_dmp[i]: "<<force_dmp[i]<<std::endl;
//            ROS_INFO("~~~~~~~ AFTER force_dmp ~~~~~~~");
        }

        //If plan is at least minimum length, check to see if we are close enough to goal

        for(int i=0; i<_dims; i++){
            if(_goal_thresh[i] > 0){
                if(fabs(cur_state.positions[i] - _goal[i]) > _goal_thresh[i])
                    at_goal = false;
            }
        }
//        ROS_INFO("~~~~~~~ AFTER at_goal ~~~~~~~");
        return at_goal;
    }

    bool MyDmp::get_force(const DMPPoint &cur_state, const double &cur_t, DVector &force_dmp) const{
        DVector couple_term(3,0);
        return get_force(cur_state,couple_term,cur_t,force_dmp);
    };


    /**
     * @brief Calculate an exp-decaying 1 to 0 phase based on time and the time scaling constant tau
     * @param[in] curr_time The current time in seconds from the start of DMP execution / trajectory
     * @param[in] tau The DMP time scaling constant
     * @return A zero to one phase
     */
    double MyDmp::CalcPhase(double curr_time) const{
        return exp(-(alpha/_tau)*curr_time);
    }


    /**
     * @brief Delete pointer
     */
    MyDmp::~MyDmp(){
        for(int i=0; i<_dims; i++){
            delete _f[i];
        }
    }

    // 计算欧式距离
    double distance(const DVector& x , const DVector& y){
        double ans = 0;
        for (int i = 0; i < x.size(); ++i) {
            ans += pow(x[i] - y[i],2);
        }
        return std::sqrt(ans);
    }
}