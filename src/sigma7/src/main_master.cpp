//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2016, Nearlab
    Refer to Readme.txt for full license text

    \description: This ros node interface with a Force Dimension Sigma master
    device.

    The following topics are published to:
    - /pose  <PoseStamped> Cartesian pose of the end effector.
    - /buttons <Joy> Contains the states of two buttons of sigma. First is
    the gripper button emulation, and second is the foot pedal.
    - /twist  <TwistStamped> Cartesian velocities of the end-effector

    The following topic is subscribed to:
    - /force_feedback <WrenchStamped> Receives wrench informatino and when
    the foot pedal is pressed it sends the wrenches to the sigma device. Note
    that you can lock the orientation of the end-effector (useful in
    tele-operation) when the foot-pedal is released if you set the
    lock_orientation parameter as true.

    \author    <http://nearlab.polimi.it/>
    \author    Nima Enayati
    \version   2.0
 */
//==============================================================================

#include <sstream>
#include "SigmaDevice.hpp"

void CheckAvailableDevices(int &devs);

void StayDeviceZero(int &devs);

int id;

int main(int argc, char** argv) {

    ros::init(argc, argv, "sigma");
    ros::NodeHandle n(ros::this_node::getName());

    // Locking call looking for connected devices.
    // devs is the number of available devices
    int devs = 0;

    ROS_INFO("IDs:%d", dhdOpenID(0));
    ROS_INFO("IDs:%d", dhdOpenID(1));

    CheckAvailableDevices(devs);


    // declare device pointers
    std::vector<SigmaDevice*> sigma(devs+1);

    // Initialize devices
    for (int i=0; i<devs; i++){
        std::stringstream dev_names;
        dev_names << "sigma"<< i;
        sigma[i] = new SigmaDevice(n, dev_names.str(),i);
    }

    // get the frequency parameter
    double rate;
    n.param<double>("frequency", rate, 1000);
    ROS_INFO("Set frequency: %f", rate);
    ros::Rate loop_rate(rate);

    ROS_INFO("Initialization done.");
    

    // StayDeviceZero(devs);

    for (int i = 0;i < devs;++i) {
        StayDeviceZero(i);
    }


    while (ros::ok()) {

        // Reading Sigma measurements and setting the force
        for (int i = 0; i < devs; i++) {

            // std::cout << i << std::endl;

            sigma[i]->ReadMeasurementsFromDevice();

            sigma[i]->PublishPoseTwistButtonPedal();

            //Applying force feedback if the pedal is engaged
              //sigma[i]->HandleWrench();

            
            sigma[i]->HandleWrenchAlways(0.5);
            sigma[i]->PositionControl();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Ending Session...\n");
    for (int i=0; i<devs; i++){
        if (drdClose ((char)i) < 0)
            ROS_ERROR (" %s\n", dhdErrorGetLastStr ());
        else
            ROS_INFO("Closed device %i" ,i );
        delete sigma[i];
    }

    return 0;
}


void CheckAvailableDevices(int &devs) {

    while(ros::ok() && devs==0) {
        for (int i = 0; i < 2; i++) {
            if (drdOpenID((char) i) > -1)
                devs = i+1;
        }
        ros::Rate r(0.5);
        r.sleep();
        ROS_INFO("Looking for connected devices...");
    }

    ROS_INFO("Found %i Device(s)", devs);
}


void StayDeviceZero(int &i){

    double nullPose[DHD_MAX_DOF] = {
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0
    };

    id = i;

    ROS_INFO("Set Device to zero...\n");

    drdStart((char) id);
    drdMoveTo(nullPose, true,(char)id);
    dhdSleep(1.0);
    drdStop(true, (char) id);
    dhdEnableForce (DHD_ON, (char) id);
    dhdSleep(0.2);

    dhdEmulateButton(DHD_ON, (char)id);

    ROS_INFO("Device at zero pose");
}
