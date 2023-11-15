
import rospy
from iiwa_msgs.srv import ConfigureControlMode
from iiwa_msgs.msg import JointImpedanceControlMode
from iiwa_msgs.msg import CartesianImpedanceControlMode
from iiwa_msgs.msg import DesiredForceControlMode
from iiwa_msgs.msg import SinePatternControlMode
from iiwa_msgs.msg import CartesianControlModeLimits
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

        cartesianImpedanceControlMode.cartesian_damping.x = 0.95
        cartesianImpedanceControlMode.cartesian_damping.y = 0.95
        cartesianImpedanceControlMode.cartesian_damping.z = 0.95
        cartesianImpedanceControlMode.cartesian_damping.a = 0.95
        cartesianImpedanceControlMode.cartesian_damping.b = 0.95
        cartesianImpedanceControlMode.cartesian_damping.c = 0.95

        cartesianImpedanceControlMode.cartesian_stiffness.x = 10
        cartesianImpedanceControlMode.cartesian_stiffness.y = 10
        cartesianImpedanceControlMode.cartesian_stiffness.z = 10
        cartesianImpedanceControlMode.cartesian_stiffness.a = 0.7
        cartesianImpedanceControlMode.cartesian_stiffness.b = 0.7
        cartesianImpedanceControlMode.cartesian_stiffness.c = 0.7
        # cartesianImpedanceControlMode.cartesian_damping = [0.5 ,0.5, 0.5, 0.5, 0.5, 0.5]
        # cartesianImpedanceControlMode.cartesian_stiffness = [50, 50, 50, 50, 50, 50]
        cartesianImpedanceControlMode.nullspace_stiffness =  100
        cartesianImpedanceControlMode.nullspace_damping = 0.7

        # limits.max_path_deviation = [1, 1, 1, 0.5, 0.5, 0.5]
        # limits.max_control_force = [20,20,20,0.5,0.5,0.5]
        # limits.max_control_force_stop = False
        # limits.max_cartesian_velocity = [20,20,20,0.2,0.2,0.2]

        configureControlMode.control_mode = 2
        configureControlMode.cartesian_impedance = cartesianImpedanceControlMode
        configureControlMode.limits = limits
        configureControlMode.joint_impedance = jointImpedanceControlMode
        configureControlMode.desired_force = desiredForceControlMode
        configureControlMode.sine_pattern =sinePatternControlMode

        change_mode(2,jointImpedanceControlMode,cartesianImpedanceControlMode,desiredForceControlMode,sinePatternControlMode,limits)
        # return response.name
    except rospy.ServiceException as e:
        print('Service call failed: %s'%e)

if __name__ == '__main__':
    main()
