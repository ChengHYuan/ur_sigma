import rospy

from iiwa_msgs.srv import SetSmartServoJointSpeedLimits

def main():
    rospy.init_node('iiwa_velocity_rate')
    rospy.wait_for_service('/iiwa/configuration/setSmartServoLimits')
    try:
        velocity_rate= rospy.ServiceProxy("/iiwa/configuration/setSmartServoLimits",SetSmartServoJointSpeedLimits)
        joint_relative_velocity=1.0#设置关节加速度比例（0-1.0）
        joint_relative_acceleration=-1 #设置关节加速度比例（0-1.0）
        override_joint_acceleration=-1

        is_success=velocity_rate(joint_relative_velocity,joint_relative_acceleration,override_joint_acceleration)
        if is_success.success:
            print("success to change velocity rate, rate is: %f"%joint_relative_velocity)
        else:
            print("set velocity failed.error code is %s"%is_success.error)
    except rospy.ServiceException as e:
        print('Service call failed: %s'%e)

    return


if __name__ == '__main__':
    main()