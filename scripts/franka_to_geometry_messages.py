#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
from tf.transformations import quaternion_from_matrix

"""
Use this script to publish franka states vairables as geometry messages.
"""

class FrankaStatesConverter:
    def __init__(self):
        self.sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.convert_to_geometry_msg, queue_size=1)
        self.pub_eeff = rospy.Publisher("/franka_state_controller/O_T_EE", PoseStamped, queue_size=1)
        self.pub_eeff_flange = rospy.Publisher("/franka_state_controller/O_T_FL", PoseStamped, queue_size=1)

    @staticmethod
    def q_from_R(R):
        """ generates quaternion from 3x3 rotation matrix """
        _R = np.eye(4)
        _R[:3, :3] = R
        return quaternion_from_matrix(_R)

    def convert_to_geometry_msg(self, state_msg):
        """ publishes franka states as geometry msgs """
        
        
        # Tip of finger gripper
        O_T_EE = np.array(state_msg.O_T_EE).reshape(4, 4).T
        quat_ee = self.q_from_R(O_T_EE[:3, :3])        

        msg_o_t_ee = PoseStamped()
        msg_o_t_ee.header.stamp = state_msg.header.stamp
        msg_o_t_ee.header.frame_id = "panda_link0"
        msg_o_t_ee.pose.position.x = O_T_EE[0, 3]
        msg_o_t_ee.pose.position.y = O_T_EE[1, 3]
        msg_o_t_ee.pose.position.z = O_T_EE[2, 3]
        msg_o_t_ee.pose.orientation.x = quat_ee[0]
        msg_o_t_ee.pose.orientation.y = quat_ee[1]
        msg_o_t_ee.pose.orientation.z = quat_ee[2]
        msg_o_t_ee.pose.orientation.w = quat_ee[3]

        # Flange of robot
        F_T_EE_ = np.array(state_msg.F_T_EE).reshape(4, 4).T
        F_T_EE = np.asmatrix(O_T_EE) * np.linalg.inv(np.asmatrix(F_T_EE_))
        quat_fl = self.q_from_R(F_T_EE_[:3, :3])

        msg_o_t_fl = PoseStamped()
        msg_o_t_fl.header.stamp = state_msg.header.stamp
        msg_o_t_fl.header.frame_id = "panda_link0"
        msg_o_t_fl.pose.position.x = F_T_EE[0, 3]
        msg_o_t_fl.pose.position.y = F_T_EE[1, 3]
        msg_o_t_fl.pose.position.z = F_T_EE[2, 3]
        msg_o_t_fl.pose.orientation.x = quat_ee[0]
        msg_o_t_fl.pose.orientation.y = quat_ee[1]
        msg_o_t_fl.pose.orientation.z = quat_ee[2]
        msg_o_t_fl.pose.orientation.w = quat_ee[3]

        self.pub_eeff.publish(msg_o_t_ee)
        self.pub_eeff_flange.publish(msg_o_t_fl)


if __name__ == '__main__':
    try:
        rospy.init_node("franka_states_converter")
        FrankaStatesConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass