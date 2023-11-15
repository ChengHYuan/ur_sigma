#include "dmp/dmp.h"
using namespace dmp;

std::vector<DMPData> active_dmp_list;
dmp::FunctionApprox **f;
int dims;
bool lfdCallback(LearnDMPFromDemo::Request  &req,
			     LearnDMPFromDemo::Response &res )
{
	learnFromDemo(req.demo, req.k_gains, req.d_gains, req.num_bases, res.dmp_list);
	res.tau = req.demo.times[req.demo.times.size()-1];
	return true;
}

bool planCallback(GetDMPPlan::Request  &req,
			      GetDMPPlan::Response &res )
{
	generatePlan(active_dmp_list, req.x_0, req.x_dot_0, req.t_0, req.goal, req.goal_thresh,
			     req.seg_length, req.tau, req.dt, req.integrate_iter, res.plan, res.at_goal);
	return true;
}

bool planStepCallback(GetDMPStepPlan::Request  &req,
                      GetDMPStepPlan::Response &res )
{

    bool atGoal;
    std::string info = std::to_string(req.current.positions.size());
    ROS_INFO("%s", info.c_str());
    DMPPoint plan;
    generateStep(active_dmp_list, req.x_0, req.t_0, req.goal, req.goal_thresh, req.tau, req.dt,
                 req.integrate_iter, f, req.current, plan, atGoal);


    res.plan = plan;
    info = std::to_string(res.plan.positions.size());
    ROS_INFO("%s", info.c_str());
    res.at_goal = atGoal;
    return true;
}


bool activeCallback(SetActiveDMP::Request &req,
					SetActiveDMP::Response &res )
{
	active_dmp_list = req.dmp_list;
	res.success = true;
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dmp_server");
	ros::NodeHandle n;


	ros::ServiceServer service1 = n.advertiseService("learn_dmp_from_demo", lfdCallback);
	ros::ServiceServer service2 = n.advertiseService("get_dmp_plan", planCallback);
	ros::ServiceServer service3 = n.advertiseService("set_active_dmp", activeCallback);

    dims = active_dmp_list.size();
    // construct the force term function by dmp parameters
    f = new dmp::FunctionApprox*[dims];
    for(int i=0; i<dims; i++){
        double base_width = pow(active_dmp_list[i].weights.size(),1.5) * dmp::alpha;
        f[i] = new dmp::RadialApprox(active_dmp_list[i].weights, base_width, dmp::alpha);
    }


    dmp::GetDMPStepPlan srvPlan;
    const double X0 = 0.16264;
    const double Y0 = -0.1453;
    const double XG = 0.14541;
    const double YG = -0.34241;
    srvPlan.request.x_0.push_back(X0);srvPlan.request.x_0.push_back(Y0);
    srvPlan.request.t_0 = 0;
    srvPlan.request.goal.push_back(XG);srvPlan.request.goal.push_back(YG);
    srvPlan.request.goal_thresh.push_back(0.002);srvPlan.request.goal_thresh.push_back(0.002);
    srvPlan.request.seg_length = 20;
    srvPlan.request.tau = 5.0;
    srvPlan.request.dt = 0.008;
    srvPlan.request.integrate_iter = 1;
    srvPlan.request.current.positions = srvPlan.request.x_0;
    srvPlan.request.current.velocities.push_back(0);srvPlan.request.current.velocities.push_back(0);


    std::string info = std::to_string(srvPlan.request.current.positions.size());
    ROS_INFO("%s", info.c_str());
    bool atGoal;
    DMPPoint plan;
    generateStep(active_dmp_list, srvPlan.request.x_0, srvPlan.request.t_0, srvPlan.request.goal, srvPlan.request.goal_thresh, srvPlan.request.tau, srvPlan.request.dt,
                 srvPlan.request.integrate_iter, f, srvPlan.request.current, plan, atGoal);


    srvPlan.response.plan = plan;

    info = std::to_string(srvPlan.response.plan.positions.size());
    ROS_INFO("%s", info.c_str());
    srvPlan.response.at_goal = atGoal;


    ros::ServiceServer service4 = n.advertiseService("get_dmp_step_plan", planStepCallback);

    ROS_INFO("DMP services now ready");
	ros::spin();

	return 0;
}
