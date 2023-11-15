//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2016, Nearlab
    Refer to Readme.txt for full license text

    \author    <http://nearlab.polimi.it/>
    \author    Nima Enayati
    \version   2.0
*/
//==============================================================================

#include "SigmaDevice.hpp"
#include <sensor_msgs/Joy.h>

// int SigmaDevice::id = -1;

//-----------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------
SigmaDevice::SigmaDevice(ros::NodeHandle n, const std::string ns,int i)
        : new_wrench_msg(false)
{
    id = i;
    ROS_INFO("-------id %d flag------", id);

    // setup the publishers and the subscriber
    pub_pose = n.advertise<geometry_msgs::PoseStamped>(ns+"/pose",1, 0);
    pub_twist = n.advertise<geometry_msgs::TwistStamped>(ns+"/twist", 1, 0);
    pub_gripper = n.advertise<std_msgs::Float32>(ns+"/gripper_angle", 1, 0);
    pub_buttons = n.advertise <sensor_msgs::Joy> (ns+"/buttons", 1, 0);
    pub_force_msg = n.advertise <geometry_msgs::Vector3Stamped> (ns+"/force_msg", 1, 0);
    pub_orientation = n.advertise<geometry_msgs::Vector3Stamped>(ns+"/orientation",1,0);

    std::string wrench_topic("/sigma/force_feedback");
    n.getParam("wrench_topic", wrench_topic);
    sub_wrench	= n.subscribe(wrench_topic, 1, &SigmaDevice::WrenchCallback,
                                this);

    std::string position_topic("/sigma/position");
    n.getParam("position_topic",position_topic);
    sub_position = n.subscribe(position_topic,1, &SigmaDevice::PositionCallback,this);

    // params
    n.param<bool>("enable_gripper_button", enable_gripper_button, 0);
    n.param<bool>("lock_orientation", lock_orient, 0);

    // calibrate the devices
    if(CalibrateDevice() == -1)
        ros::shutdown();

    buttons_msg.buttons.push_back(0);
    buttons_msg.buttons.push_back(0);
}


//------------------------------------------------------------------------------
// WrenchCallback
void SigmaDevice::WrenchCallback(
        const geometry_msgs::WrenchStampedConstPtr &msg) {
    //newDataDirect = true;
    wrench.wrench = msg->wrench;
    new_wrench_msg = true;
}
//PositionCallback
void SigmaDevice::PositionCallback( const geometry_msgs::PoseStampedConstPtr &msg){
     pose.pose = msg->pose;
}
   

//------------------------------------------------------------------------------
// CalibrateDevice
int SigmaDevice::CalibrateDevice() {

    ROS_INFO("Calibrating device %i ...", id);

    // open device
    if (drdOpenID ((char)id) < 0) {
        ROS_ERROR("No device %i found. dhd says: %s", id, dhdErrorGetLastStr());
        dhdSleep (2.0);
        drdClose ((char)id);
        return -1;
    }

    //Calibrate the device if it is not already calibrated;
    if(drdIsInitialized((char)id)){
        ROS_INFO("Device %i is already calibrated.", id);
    }
    else if(drdAutoInit((char)id)<0) {
        ROS_ERROR("Initialization of device %i failed. dhd says: (%s)", id,
                  dhdErrorGetLastStr ());
        dhdSleep(2.0);
    }

    // // center of workspace
    //	 double nullPose[DHD_MAX_DOF] = { 0.0, 0.0, 0.0, //base  (translations)
    //	                                  0.0, 0.0, 0.0, //wrist (rotations)
    //	                                  0.0 };         //gripper
    // //move to center
    // drdMoveTo (nullPose);

    // stop regulation (and leave force enabled)
    drdStop(true, (char)id);

    // enable force
    dhdEnableForce (DHD_ON, (char)id);

//    dhdSetGravityCompensation(DHD_ON, (char)id);
    dhdSleep (0.2);
    //Enable the gripper button
    if(enable_gripper_button)
        dhdEmulateButton(DHD_ON, (char)id);

    ROS_INFO("Device %i ready.", id);
    return 0;
}

int SigmaDevice::ReadMeasurementsFromDevice() {
    // ROS_INFO("-------id %d flag------", id);

    // -------------------------------
    // Pose
    double p[3];
    double orient_m[3][3];
    //Reading the data from the device
    dhdGetPositionAndOrientationFrame(&p[0], &p[1], &p[2], orient_m, (char)id);

    // convert to pose message
    KDL::Rotation rot;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) { rot(r, c) = orient_m[r][c]; }
    }
    tf::poseKDLToMsg(KDL::Frame(rot, KDL::Vector(p[0],p[1],p[2])),
                     pose_msg.pose);
    // stamp the msg
    pose_msg.header.stamp = ros::Time::now();

    // -----------------------------
    // Twist
    double v[6];

    dhdGetLinearVelocity(&v[0], &v[1], &v[2], (char)id);
    dhdGetAngularVelocityRad(&v[3], &v[4], &v[5], (char)id);
    // convert to twist message
    twist_msg.twist.linear.x = v[0];
    twist_msg.twist.linear.y = v[1];
    twist_msg.twist.linear.z = v[2];
    twist_msg.twist.angular.x = v[3];
    twist_msg.twist.angular.y = v[4];
    twist_msg.twist.angular.z = v[5];
    // stamp the msg
    twist_msg.header.stamp = ros::Time::now();

    // ------------------------------
    // gripper
    double temp;
    dhdGetGripperAngleRad(&temp,(char)id);
    gripper_angle.data = (float)temp;
    // ------------------------------
    // buttons
    // saving the previous states of gripper button and pedal
    for (int i = 0; i < 2; ++i) {
        buttons_previous_state[i] = buttons_state[i];
        buttons_state[i] = dhdGetButton(i, (char)id);
    }
    
    //Wrench
    double f[3]={0., 0., 0.};

    force_msg.header.stamp = ros::Time::now();
  
    dhdGetForce(&f[0], &f[1], &f[2],(char)id);
    force_msg.vector.x = f[0];
    force_msg.vector.y = f[1];
    force_msg.vector.z = f[2];

    

    dhdGetOrientationRad(&locked_orient[0], &locked_orient[1],&locked_orient[2],(char)id);

    orientation_msg.header.stamp = ros::Time::now();
    // orientation_msg.vector = KDL::Vector(locked_orient[0], locked_orient[1], locked_orient[2]);
    orientation_msg.vector.x = locked_orient[0];
    orientation_msg.vector.y = locked_orient[1];
    orientation_msg.vector.z = locked_orient[2];


    return 0;
}

void SigmaDevice::PublishPoseTwistButtonPedal() {

    pub_pose.publish(pose_msg);
    pub_twist.publish(twist_msg);

    // Publish gripper angle
    pub_gripper.publish(gripper_angle);

    // publish orientation angle
    pub_orientation.publish(orientation_msg);
    pub_force_msg.publish(force_msg);

    // publish buttons when there is a change
    if((buttons_state[0] != buttons_previous_state[0]) ||
            (buttons_state[1] != buttons_previous_state[1]) ){

        // populate the message
        buttons_msg.buttons[0] = buttons_state[0];
        buttons_msg.buttons[1] = buttons_state[1];
        // publish it
        pub_buttons.publish(buttons_msg);
    }

    // ROS_INFO("-------id %d flag------", id);


}

void SigmaDevice::HandleWrench() {

    // should we use new_wrench_msg?
    // force_feedback is achieved if buttons state==1

    // dhdGetOrientationRad(&locked_orient[0], &locked_orient[1],&locked_orient[2]);
    
    if(buttons_state[0] == 1) {
	
        if (dhdSetForceAndTorqueAndGripperForce(0,
                                                0,
                                                0,
                                                -0.5*locked_orient[0],
                                                -0.5*locked_orient[1],
                                                -0.5*locked_orient[2],
                                                0.0, (char)id) < DHD_NO_ERROR){
            ROS_INFO("error: cannot set force (%s)\n", dhdErrorGetLastStr());
        }
	else{
        dhdGetOrientationRad(&locked_orient[0], &locked_orient[1],&locked_orient[2],(char)id);
	
	}
    }
    else if (lock_orient){
        drdRegulatePos  (false,(char)id);
        drdRegulateRot  (true,(char)id);
        drdRegulateGrip (false,(char)id);
        drdStart();
        drdMoveToRot (locked_orient[0], locked_orient[1],locked_orient[2],(char)id);
        drdStop(true,(char)id);
	
    }
    else{
        if (dhdSetForceAndTorqueAndGripperForce(.0, .0, .0, .0, .0, .0, 0.,
                                                (char)id) < DHD_NO_ERROR)
            ROS_INFO("error: cannot set force (%s)\n", dhdErrorGetLastStr());
        }
}


//void SigmaDevice::HandleWrenchAlways(double p){

    // give feedback always
    // dhdGetOrientationRad(&locked_orient[0], &locked_orient[1],&locked_orient[2]);
    //dhdSetForceAndTorqueAndGripperForce(wrench.wrench.force.x,
                                        //wrench.wrench.force.y,
                                        //wrench.wrench.force.z,
                                        //-p*locked_orient[0],
                                        //-p*locked_orient[1],
                                        //-p*locked_orient[2],
                                        //0.0, (char)id);

//}


void SigmaDevice::HandleWrenchAlways(double p){

    // give feedback always
    // dhdGetOrientationRad(&locked_orient[0], &locked_orient[1],&locked_orient[2]);
    // if (buttons_state[0] == 0)
    // {
    //     dhdSetForceAndTorqueAndGripperForce(0,
    //         0,
    //         0,
    //         -p * locked_orient[0],
    //         -p * locked_orient[1],
    //         -p * locked_orient[2],
    //         0.0, (char)id);
    // }
    // else if(buttons_state[0] == 1)
    // {
    //     dhdSetForceAndTorqueAndGripperForce(wrench.wrench.force.x,
    //         wrench.wrench.force.y,
    //         wrench.wrench.force.z,
    //         -p * locked_orient[0],
    //         -p * locked_orient[1],
    //         -p * locked_orient[2],
    //         0.0, (char)id);

    // }
    //修改后，没有按按钮也有力反馈
    dhdSetForceAndTorqueAndGripperForce(wrench.wrench.force.x,
            wrench.wrench.force.y,
            wrench.wrench.force.z,
            -p * locked_orient[0],
            -p * locked_orient[1],
            -p * locked_orient[2],
            0.0, (char)id);

}

void SigmaDevice::PositionControl(){
     
     //drdMoveToPos(pose.pose.position.x,
                  //pose.pose.position.y,
                  //pose.pose.position.z,
                  //0.0,(char)id);

     drdMoveToPos(0,
                  0,
                  0,
                  true,(char)id);
}









