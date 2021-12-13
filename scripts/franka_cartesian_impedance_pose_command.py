#!/usr/bin/python
#  roslaunch table_setup cartesian_impedance_example_controller_no_marker.launch 
import rospy, yaml, numpy as np, os, json
from catkin.find_in_workspaces import find_in_workspaces
from IPython import embed
import itertools
from franka_msgs.msg import FrankaState
import tf.transformations
import tf
from geometry_msgs.msg import PoseStamped
from scipy import interpolate
from std_msgs.msg import String


class Mover(object):
    def __init__(self):
        self.franka_EE_trans = None
        self.franka_EE_quat = None
        self.O_F_ext_hat_K = None
        self.ext_force_ee = None
        self.ext_torque_ee = None
        rospy.Subscriber("/franka_state_controller/franka_states",
                         FrankaState, self.franka_callback)

        self.pose_pub = rospy.Publisher(
            "/cartesian_impedance_example_controller/desired_pose", PoseStamped, queue_size=10)
        # Sleep to wait for publisher ready
        rospy.sleep(1)

        while self.franka_EE_trans is None or self.franka_EE_quat is None:
            print("Wait ...")
            rospy.sleep(0.2)
        print("Ready")

    def franka_callback(self, data):
        # https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html
        q = tf.transformations.quaternion_from_matrix(
                np.transpose(np.reshape(data.O_T_EE, (4, 4))))
        self.franka_EE_quat = q / np.linalg.norm(q)
        ts = [data.O_T_EE[12], data.O_T_EE[13], data.O_T_EE[14]]
        self.franka_EE_trans = np.array(ts)
        # print("franka_EE_trans={}".format(self.franka_EE_trans))

        self.O_F_ext_hat_K = np.array(data.O_F_ext_hat_K)
        self.ext_force_ee = np.linalg.norm(self.O_F_ext_hat_K[:3], ord=2)
        self.ext_torque_ee = np.linalg.norm(self.O_F_ext_hat_K[3:], ord=2)
        # print("ext_force_ee={:.4f}, ext_torque_ee={:.4f}".format(
            # self.ext_force_ee, self.ext_torque_ee))



if __name__ == '__main__':
    np.random.seed(0)
    # https://stackoverflow.com/a/2891805
    np.set_printoptions(precision=5, suppress=True)

    rospy.init_node('test_franka_cartesian_impedance', anonymous=True)
    # Make sure sim time is working
    while not rospy.Time.now():
        pass
    x = Mover()

    marker_pose = PoseStamped()
    marker_pose.header.stamp = rospy.Time(0)

    marker_pose.pose.orientation.x = x.franka_EE_quat[0]
    marker_pose.pose.orientation.y = x.franka_EE_quat[1]
    marker_pose.pose.orientation.z = x.franka_EE_quat[2]
    marker_pose.pose.orientation.w = x.franka_EE_quat[3]

    marker_pose.pose.position.x = x.franka_EE_trans[0]
    marker_pose.pose.position.y = x.franka_EE_trans[1]
    marker_pose.pose.position.z = x.franka_EE_trans[2]+0.05

    x.pose_pub.publish(marker_pose)