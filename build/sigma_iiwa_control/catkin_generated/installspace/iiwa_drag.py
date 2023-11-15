import rospy
from iiwa_msgs.msg import JointPosition,JointVelocity

joint_flag = False

joint_pose = JointPosition()
joint_velocity = JointVelocity()

def joint_position_callback(msg):
    global joint_flag
    joint_flag = True
    global joint_pose

    joint_pose = msg

    return

def jointvelocity_callback(msg):
    global joint_flag
    joint_flag = True
    global joint_velocity
    joint_velocity = msg

    return

def main():
    global joint_flag
    joint_flag = True
    rospy.init_node('iiwa_control', anonymous=True)
    rospy.Subscriber('/iiwa/state/JointPosition', JointPosition, callback=joint_position_callback, queue_size=10)
    rospy.Subscriber('/iiwa/state/JointVelocity', JointVelocity, callback=jointvelocity_callback, queue_size=10)
    joint_pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
    rate = rospy.Rate(50)

    # print(joint_pose)

    while not rospy.is_shutdown():

        if joint_flag is True:

            joint_now = joint_pose

            v_7 = joint_velocity.velocity.a7 = 0

            if abs(v_7) < 0.1:
                joint_msg = JointPosition()

                joint_msg.position.a1 = joint_now.position.a1
                joint_msg.position.a2 = joint_now.position.a2
                joint_msg.position.a3 = joint_now.position.a3
                joint_msg.position.a4 = joint_now.position.a4
                joint_msg.position.a5 = joint_now.position.a5
                joint_msg.position.a6 = joint_now.position.a6
                joint_msg.position.a7 = joint_now.position.a7
                print(joint_msg)

                joint_pub.publish(joint_msg)
                rate.sleep()

    return


if __name__ == '__main__':
    main()
