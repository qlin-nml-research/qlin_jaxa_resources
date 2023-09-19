#!/bin/python3
# Murilo Marques Marinho at the University of Tokyo.
# Author: Murilo M. Marinho, email: murilo@g.ecc.u-tokyo.ac.jp
import time
from dqrobotics import *  # Despite what PyCharm might say, this is very much necessary or DQs will not be recognized
import rospy
from sas_robot_kinematics import RobotKinematicsInterface
from sas_operator_side_receiver import OperatorSideReceiverInterface
import socket

rospy.init_node('example_for_saul', disable_signals=True)
print('example_for_saul')
try:

    # Initialize the OperatorSideReceiverInterface
    osri = OperatorSideReceiverInterface()

    # Each master manipulator will have a label assigned to it.
    # For example, 0_1 means computer 0, manipulator 1
    osri.add_manipulator_manager('0_0')
    osri.add_manipulator_manager('0_1')
    
    # If you want any information from that master, retrieve it first
    manipulator = osri.get_manipulator_manager_ptr('0_0')
    manipulator2 = osri.get_manipulator_manager_ptr('0_1')

    arm_interface_list = [RobotKinematicsInterface('/arm1_kinematics'),
                          RobotKinematicsInterface('/arm2_kinematics')]

    # Wait for all interfaces to be enabled
    all_interfaces_enabled = False
    while not all_interfaces_enabled:
        all_interfaces_enabled = True
        for arm_interface in arm_interface_list:
            if not arm_interface.is_enabled():
                all_interfaces_enabled = False
                break
        time.sleep(0.001)
        
    # Check and wait to see if that manipulator is enabled, or an exception will be thrown
    while not manipulator.is_enabled():
        time.sleep(0.1)

    # Read initial values of each interface
    arm_counter = 1
    for arm_interface in arm_interface_list:
        print("****************************")
        print("***Initial info for arm {}***".format(arm_counter))
        print("****************************")
        print(arm_interface.get_pose())
        print(arm_interface.get_reference_frame())
        arm_counter = arm_counter + 1
        
    # Print master interface information
    print(manipulator.get_sender_ip())
    print(manipulator.get_sender_port())
    print(manipulator.get_label())
    print(manipulator.is_enabled())
    print(manipulator.get_motion_scaling())
    print(manipulator.get_pose())
    print(manipulator.get_clutch())
    print(manipulator.is_clutch_pressed())
    print(manipulator.did_clutch_just_switch_on())
    print(manipulator.did_clutch_just_switch_off())
    print(manipulator.get_additional_key_states())
    print(manipulator.get_last_clutch_off_pose())
    print(manipulator.get_last_clutch_on_pose())
    print(manipulator.get_linear_velocity())
    print(manipulator.get_angular_velocity())
    print(manipulator.get_gripper())
    print(manipulator.get_last_clutch_off_gripper())
    print(manipulator.get_last_clutch_on_gripper())
    print(manipulator.get_button())

    # Move each arm on their end-effectors' reference frame
    # arm_counter = 1
    # for arm_interface in arm_interface_list:
    #    print("Moving arm {}...".format(arm_counter))
    #    x = arm_interface.get_pose()
    #    xd = x * (1 + 0.5 * E_ * i_ * 0.1)
    #    arm_interface.send_desired_pose(xd)
    #    arm_interface.send_desired_interpolator_speed(0.1)
    #    arm_counter = arm_counter + 1
    
    # Communication with Unreal Engine
    HOST = "10.198.113.183"
    PORT = 20023
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect((HOST, PORT))    
        print("CONNECTED TO {}:{}".format(HOST, PORT));
        while True:
                arm1_pose = arm_interface_list[0].get_pose()
                arm1_t = 1000 * translation(arm1_pose).vec3() # tx ty tz, In millimeters
                arm1_r = rotation(arm1_pose).vec4() # rw rx ry rz
                arm1_g = manipulator.get_gripper() # gripper range [0,1]
                arm1_str = "{} {} {} {} {} {} {} {}".format(arm1_t[0], arm1_t[1], arm1_t[2], arm1_r[1], arm1_r[2], arm1_r[3], arm1_r[0], arm1_g) # tx ty tz rx ry rz rw g
                
                arm2_pose = arm_interface_list[1].get_pose()
                arm2_t = 1000 * translation(arm2_pose).vec3() # tx ty tz, In millimeters
                arm2_r = rotation(arm2_pose).vec4() # rw rx ry rz
                arm2_g = manipulator2.get_gripper() # gripper range [0,1]
                arm2_str = "{} {} {} {} {} {} {} {}".format(arm2_t[0], arm2_t[1], arm2_t[2], arm2_r[1], arm2_r[2], arm2_r[3], arm2_r[0], arm2_g) # tx ty tz rx ry rz rw g
                
                #{} vec8
                #() vec4 P()
                #[] vec3 
                # {(1,[0,0,0]),0,0,0,0}
                
                message = "{} {}".format(arm1_str, arm2_str)
                #print(message)
                
                s.sendall(bytes(message, 'utf-8'))
                time.sleep(1.0/30.0)

except KeyboardInterrupt:
    print("Interrupted by user")
