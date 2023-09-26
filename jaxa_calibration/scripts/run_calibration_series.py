#!/bin/python3
import math
import os

import numpy as np
import rospy

import dqrobotics as dql
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from sas_robot_kinematics import RobotKinematicsInterface

import traceback

np.set_printoptions(formatter={'float': '{: 0.30f},'.format})


def closest_invariant_rotation_error_scalar(x, xd):
    err = closest_invariant_rotation_error(x, xd)
    return np.linalg.norm(err.vec4())


def closest_invariant_rotation_error(x, xd):
    er_plus = dql.conj(dql.rotation(x)) * dql.rotation(xd) - 1
    er_minus = dql.conj(dql.rotation(x)) * dql.rotation(xd) + 1

    if np.linalg.norm(er_plus.vec4()) < np.linalg.norm(er_minus.vec4()):
        return er_plus
    else:
        return er_minus


def translation_error_scalar(x, xd):
    err = dql.vec3(dql.translation(x) - dql.translation(xd))
    return np.linalg.norm(err)


def run_main(name, config):
    rate = rospy.Rate(config['update_rate'])
    arm_interface_list = [
        RobotKinematicsInterface(config['arm_1_ns']),
        RobotKinematicsInterface(config['arm_2_ns'])
    ]

    pose_list = config['pose_list']
    # print(pose_list)

    vrep = DQ_VrepInterface()
    if not vrep.connect(_config['vrep_ip'], _config['vrep_port'], 100, 10):
        rospy.logwarn("[" + name + "]::vrep connection failure")
        raise RuntimeError("Exit on vrep connection failure")

    # wait for arm to initialize
    all_interfaces_enabled = False
    while not all_interfaces_enabled:
        all_interfaces_enabled = True
        for arm_interface in arm_interface_list:
            if not arm_interface.is_enabled():
                all_interfaces_enabled = False
                break
        rate.sleep()
    rospy.loginfo("[" + name + "]" + ":: Arm kinematic initialized")

    # Read initial values of each interface
    xds = [None] * 2
    for ind, arm_interface in enumerate(arm_interface_list):
        rospy.loginfo("[" + name + "]:: ****************************")
        rospy.loginfo("[" + name + "]:: ***Initial info for arm {}***".format(ind + 1))
        rospy.loginfo("[" + name + "]:: ****************************")
        xds[ind] = arm_interface.get_pose()
        rospy.loginfo("[" + name + "]:: " + str(xds[ind]))
        arm_interface.send_desired_pose(xds[ind])
        rospy.loginfo("[" + name + "]:: " + str(arm_interface.get_reference_frame()))

    rospy.loginfo("[" + name + "]" + ":: Arm kinematic set")

    pose_log_file_h = open(config['pose_log_txt_path'], "w")
    rospy.loginfo("[" + name + "]" + ":: logfile opened")

    try:
        run_index = config['calibrating_arm_num']
        iter = 0
        pose_list_counter = 0
        while True:
            # set desire pose here
            xds[run_index] = pose_list[pose_list_counter]

            xs = [arm_interface.get_pose() for arm_interface in arm_interface_list]
            for ind, x in enumerate(xs):
                vrep.set_object_pose(config['vrep_x_names'][ind], x)

            for ind, xd in enumerate(xds):
                arm_interface_list[ind].send_desired_pose(xd)
                vrep.set_object_pose(config['vrep_xd_names'][ind], xd)

            err_ts = [translation_error_scalar(x, xd) for x, xd in zip(xs, xds)]
            err_rs = [closest_invariant_rotation_error_scalar(x, xd) for x, xd in zip(xs, xds)]
            if err_ts[run_index] < config['t_threshold'] and err_rs[run_index] < config['r_threshold']:
                # log current pose to file
                arm_interface_list[run_index].send_desired_pose(xs[run_index])  # stop robot from moving
                pose_str = ""
                for ele in xs[run_index].vec8():
                    pose_str += "{:.10f},".format(ele)
                desired_pose_str = ""
                for ele in pose_list[pose_list_counter].vec8():
                    desired_pose_str += "{:.10f},".format(ele)
                # counter, pose.vec8(),
                save_string = "{:d},".format(pose_list_counter) + pose_str + desired_pose_str + "\n"
                pose_log_file_h.writelines(save_string)

                rospy.logwarn("[" + name + "]:: arrived target position number {:d} "
                                           "Waiting for keyboard input to continue".format(pose_list_counter))
                _ = input()
                pose_list_counter += 1
                if pose_list_counter >= len(pose_list):
                    rospy.logwarn("[" + name + "]:: completed pose list setpoint")
                    break

            if iter % 30 == 0:
                rospy.loginfo("[" + name + "]:: current t_err:{:.5f}, r_err:{:.5f}".format(err_ts[run_index],
                                                                                           err_rs[run_index]))

            rate.sleep()
            iter += 1

    except KeyboardInterrupt:
        rospy.logwarn("[" + name + "]::Existing on keyboard interrupt")

    except Exception as e:
        traceback.print_exc()

    vrep.disconnect_all()
    pose_log_file_h.close()


if __name__ == '__main__':
    rospy.init_node("calibration_pose_series_runner", anonymous=True, disable_signals=True)

    _name = rospy.get_name()
    params = rospy.get_param(_name)

    _config = {}

    _pose_list = []

    try:
        _config['update_rate'] = params['update_rate']

        _config['arm_1_ns'] = params['arm_1_ns']
        _config['arm_2_ns'] = params['arm_2_ns']

        _config['vrep_port'] = params['vrep_port']
        _config['vrep_ip'] = params['vrep_ip']
        _config['vrep_x_names'] = params['vrep_x_names']
        _config['vrep_xd_names'] = params['vrep_xd_names']

        _config['t_threshold'] = params['t_threshold']
        _config['r_threshold'] = params['r_threshold']

        _config['calibrating_arm_num'] = params['calibrating_arm_num']
        _config['pose_log_txt_path'] = params['pose_log_txt_path']

        pose_gen_params = params['pose_generation_params']
        rotation_init = dql.DQ(pose_gen_params['r_init']).normalize()
        translation_init = dql.DQ(pose_gen_params['t_init'])

        rotation_list = [rotation_init]
        rot_delta = pose_gen_params["rot_d"]
        for axis_name, axis in zip(["i", "j", "k"], [dql.i_, dql.j_, dql.k_]):
            for sign in [-1, 1]:
                # print(pose_gen_params["rot_d" + axis_name], )
                if axis_name in rot_delta:
                    delta_r = math.cos(sign * math.radians(rot_delta[axis_name]) / 2) + \
                              axis * math.sin(sign * math.radians(rot_delta[axis_name]) / 2)
                    rotation_list.append(rotation_init * delta_r)

        translation_list = [translation_init]
        _config['pose_list'] = _pose_list
        t_offsets = pose_gen_params["t_offsets"]
        for offset in t_offsets:
            translation_list.append(translation_init+dql.DQ(offset))

        _pose_list = []
        for t in translation_list:
            for r in rotation_list:
                _pose_list.append(r + 0.5 * dql.E_ * t * r)

        _config['pose_list'] = _pose_list

        # clear parameter after loading
        rospy.delete_param(os.path.join(_name, 'pose_generation_params'))

        rospy.loginfo("[" + _name + "]:: Parameter load OK.")
    except Exception as e:

        rospy.logerr(_name + ": initialization ERROR on parameters")
        rospy.logerr(_name + ": " + str(e))
        rospy.signal_shutdown(_name + ": initialization ERROR on parameters")
        traceback.print_exc()
        exit()

    namespace = rospy.get_namespace()

    run_main(_name, _config)
