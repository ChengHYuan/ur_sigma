import rospy
from iiwa_msgs.srv import ConfigureControlMode
from iiwa_msgs.msg import JointImpedanceControlMode
from iiwa_msgs.msg import CartesianImpedanceControlMode
from iiwa_msgs.msg import DesiredForceControlMode
from iiwa_msgs.msg import SinePatternControlMode
from iiwa_msgs.msg import CartesianControlModeLimits
from geometry_msgs.msg import PoseStamped
def main():
    rospy.init_node('iiwa_control')
    rospy.wait_for_service('/iiwa/configuration/ConfigureControlMode')
    try:
        change_mode = rospy.ServiceProxy('/iiwa/configuration/ConfigureControlMode',ConfigureControlMode)
        configureControlMode = ConfigureControlMode()
        cartesianImpedanceControlMode = CartesianImpedanceControlMode()
        limits = CartesianControlModeLimits()
        jointImpedanceControlMode = JointImpedanceControlMode()
        desiredForceControlMode = DesiredForceControlMode()
        sinePatternControlMode = SinePatternControlMode()

        configureControlMode.control_mode = 0
        configureControlMode.cartesian_impedance = cartesianImpedanceControlMode
        configureControlMode.limits = limits
        configureControlMode.joint_impedance = jointImpedanceControlMode
        configureControlMode.desired_force = desiredForceControlMode
        configureControlMode.sine_pattern =sinePatternControlMode

        change_mode(0,jointImpedanceControlMode,cartesianImpedanceControlMode,desiredForceControlMode,sinePatternControlMode,limits)
        print('success!')
        # return response.name
    except rospy.ServiceException as e:
        print('Service call failed: %s'%e)

if __name__ == '__main__':
    main()