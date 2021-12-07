#!/usr/bin/env python

from IPython import embed
import rospy
from gripper import GripperInterface


def main():
    rospy.init_node("test_franka_gripper", anonymous=True)
    rospy.set_param("/franka_gripper/robot_ip", "172.16.0.2")

    g = GripperInterface()
    g.calibrate()

    print("Close?")
    embed()
    g.grasp(width=0., force=50, speed = None, epsilon_inner = 0.005, epsilon_outer = 0.005,wait_for_result=False, cb=None)
    
    print("Open?")
    embed()
    g.open()

if __name__ == '__main__':
    main()
