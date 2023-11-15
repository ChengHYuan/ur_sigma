/**
 * @file   EposProfileVelocityMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 22:42:41
 */

#include "epos_driver/control/EposProfileVelocityMode.hpp"

EposProfileVelocityMode::~EposProfileVelocityMode()
{}

void EposProfileVelocityMode::init(ros::NodeHandle &motor_nh, NodeHandle &node_handle)
{
    ControlModeBase::init(motor_nh, node_handle);
    if (m_use_ros_unit) {
        ros::NodeHandle encoder_nh(motor_nh, "encoder");
        const int type(encoder_nh.param("type", 0));

        if (type == 1 || type == 2) {
            const int resolution(encoder_nh.param("resolution", 0));
            const int gear_ratio(encoder_nh.param("gear_ratio", 0));
            ratio=gear_ratio;
            if (resolution == 0 || gear_ratio == 0) {
                throw EposException("Please set parameter 'resolution' and 'gear_ratio'");
            }
            const bool inverted_polarity(encoder_nh.param("inverted_poloarity", false));
        }
    }
    activate();
}

void EposProfileVelocityMode::activate()
{
    ROS_INFO("Activate ProfileVelocity Mode");
    VCS_NODE_COMMAND_NO_ARGS(ActivateProfileVelocityMode, m_epos_handle);
}

void EposProfileVelocityMode::read()
{}

void EposProfileVelocityMode::write(const double position, const double velocity, const double current)
{
    int target;

    if (m_use_ros_unit) {
        target = static_cast<int>(velocity*ratio* 60.);
    } else {
        target = static_cast<int>(velocity*ratio);
    }
    ROS_INFO("the target velocity is : %d",target);
    VCS_NODE_COMMAND(MoveWithVelocity, m_epos_handle, target);

}
