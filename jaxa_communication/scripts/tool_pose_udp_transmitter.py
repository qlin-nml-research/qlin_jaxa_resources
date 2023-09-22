#!/usr/bin/python3
import math

import numpy as np
import rospy
import os, sys, time, datetime, traceback


import dqrobotics as dql
from sas_robot_kinematics import RobotKinematicsInterface
from sas_operator_side_receiver import OperatorSideReceiverInterface
import socket

timed_out = 1.0  # sec
rate_limit = 30


def tool_pose_transmitter_main(_name, _config):
    rospy.logwarn("[" + rospy.get_name() + "]::Running")
    # Initialize the OperatorSideReceiverInterface
    osri = OperatorSideReceiverInterface()

    # Each master manipulator will have a label assigned to it.
    # For example, 0_1 means computer 0, manipulator 1
    osri.add_manipulator_manager('0_1')
    osri.add_manipulator_manager('0_0')
    # If you want any information from that master, retrieve it first
    manipulator1 = osri.get_manipulator_manager_ptr('0_1')
    manipulator2 = osri.get_manipulator_manager_ptr('0_0')

    arm_interface_list = [RobotKinematicsInterface('/arm1_kinematics'),
                          RobotKinematicsInterface('/arm2_kinematics')]

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect((_config['remote_ip'], _config['port']))
        rospy.logwarn("CONNECTED TO {}:{}".format(_config['remote_ip'], _config['port']))
        try:
            updated_time = rospy.get_time()
            rate = rospy.Rate(rate_limit)

            # Wait for all interfaces to be enabled
            for arm_interface in arm_interface_list:
                while True:
                    if arm_interface.is_enabled():
                        break
                    rate.sleep()
            rospy.logwarn("Arm interface enabled")

            # Check and wait to see if that manipulator is enabled, or an exception will be thrown
            while not manipulator1.is_enabled():
                rate.sleep()
            while not manipulator2.is_enabled():
                rate.sleep()
            rospy.logwarn("manipulator enabled")

            # Read initial values of each interface
            arm_counter = 1
            for arm_interface in arm_interface_list:
                rospy.loginfo("****************************")
                rospy.loginfo("***Initial info for arm {}***".format(arm_counter))
                rospy.loginfo("****************************")
                rospy.loginfo(arm_interface.get_pose())
                rospy.loginfo(arm_interface.get_reference_frame())
                arm_counter = arm_counter + 1
            rospy.logwarn("[" + rospy.get_name() + "]::Entering loop")

            while True:

                arm1_pose = arm_interface_list[0].get_pose()
                arm1_t = 1000 * dql.translation(arm1_pose).vec3()  # tx ty tz, In millimeters
                arm1_r = dql.rotation(arm1_pose).vec4()  # rw rx ry rz
                arm1_g = manipulator1.get_gripper()  # gripper range [0,1]
                arm1_str = "{} {} {} {} {} {} {} {}".format(arm1_t[0], arm1_t[1], arm1_t[2], arm1_r[1], arm1_r[2],
                                                            arm1_r[3], arm1_r[0], arm1_g)  # tx ty tz rx ry rz rw g

                # rospy.loginfo("Arm 1:"+str(arm1_t))

                arm2_pose = arm_interface_list[1].get_pose()
                arm2_t = 1000 * dql.translation(arm2_pose).vec3()  # tx ty tz, In millimeters
                arm2_r = dql.rotation(arm2_pose).vec4()  # rw rx ry rz
                arm2_g = manipulator2.get_gripper()  # gripper range [0,1]
                arm2_str = "{} {} {} {} {} {} {} {}".format(arm2_t[0], arm2_t[1], arm2_t[2], arm2_r[1], arm2_r[2],
                                                            arm2_r[3], arm2_r[0], arm2_g)  # tx ty tz rx ry rz rw g
                # rospy.loginfo("Arm 2:"+str(arm2_t))

                # {} vec8
                # () vec4 P()
                # [] vec3
                # {(1,[0,0,0]),0,0,0,0}

                message = "{} {}".format(arm1_str, arm2_str)
                # print(message)

                s.sendall(bytes(message, 'utf-8'))

                rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("[" + rospy.get_name() + "]:::exit on keyboard interrupt")

        pass


if __name__ == '__main__':

    rospy.init_node("tool_pose_udp_transmitter_node",
                    disable_signals=True,
                    anonymous=False)

    name = rospy.get_name()
    params = rospy.get_param(name)

    config = {}

    try:
        config['port'] = params['port']
        config['remote_ip'] = params['remote_ip']
        config['arm1_manipulator_ns'] = params['arm1_manipulator_ns']
        config['arm2_manipulator_ns'] = params['arm2_manipulator_ns']
        config['arm1_kinematics_ns'] = params['arm1_kinematics_ns']
        config['arm2_kinematics_ns'] = params['arm2_kinematics_ns']
        """
            port: 20023
            remote_ip: 10.198.113.183
            arm1_manipulator_ns: 0_0
            arm1_kinematics_ns: /arm1_kinematics
            arm2_manipulator_ns: 0_1
            arm2_kinematics_ns: /arm2_kinematics
        """

        rospy.loginfo("[" + name + "]:: Parameter load OK.")
    except Exception as e:

        rospy.logerr(name + ": initialization ERROR on parameters")
        rospy.logerr(name + ": " + str(e))
        rospy.signal_shutdown(name + ": initialization ERROR on parameters")
        exit()

    namespace = rospy.get_namespace()

    tool_pose_transmitter_main(name, config)
