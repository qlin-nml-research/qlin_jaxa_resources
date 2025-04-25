#!/usr/bin/python3
"""
(C) Copyright Hung-Ching Lin

This file is part of jaxa_resources.

    qlin_jaxa_resources is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    qlin_jaxa_resources is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with qlin_jaxa_resources.  If not, see <http://www.gnu.org/licenses/>.

Author:
    Hung-Ching Lin (qlin1806@g.ecc.u-tokyo.ac.jp)

Contributors (aside from author):
"""
import math
from typing import List, Union

import numpy as np
import rospy
import os, sys, time, datetime, traceback

from PyQt5.QtCore import QByteArray, qChecksum, QDataStream, QIODevice, QBuffer
from PyQt5.QtNetwork import QUdpSocket, QHostAddress
from sas_robot_kinematics import RobotKinematicsInterface
from sas_robot_driver import RobotDriverInterface
import dqrobotics as dql

from collections import deque

MAX_RATE = 1000


class TransmitDataStruct:
    _fields = [('interface_id', 'uint', 1), ('desired_pose', 'double', 8),
               ('gripper', 'double', 1)]

    def get_fields(self):
        return self._fields

    def __init__(self):
        for name_, dtype_, len_ in self._fields:
            if len_ is None:
                setattr(self, name_, np.zeros(0))
            else:
                setattr(self, name_, np.zeros(len_))


def read_byte_array(data_buffer: QByteArray, expected_len) -> List[TransmitDataStruct]:
    ret = [TransmitDataStruct() for _ in range(expected_len)]
    crc16 = qChecksum(data_buffer[:-2])
    data_stream = QDataStream(data_buffer, QIODevice.ReadOnly)
    data_stream.setVersion(18)
    data_stream.setByteOrder(QDataStream.BigEndian)
    for i in range(expected_len):
        for key_, dtype_, length_ in ret[i].get_fields():
            if length_ is None:
                length = data_stream.readUInt32()
            else:
                length = length_
            data = np.zeros([length])
            for ind in range(length):
                if dtype_ == 'int':
                    data[ind] = data_stream.readInt64()
                elif dtype_ == 'uint':
                    data[ind] = data_stream.readUInt64()
                elif dtype_ == 'double':
                    data[ind] = data_stream.readDouble()
                elif type == 'bool':
                    data[ind] = data_stream.readUInt8()
                else:
                    raise RuntimeError("unrecognized type")
            setattr(ret[i], key_, data)
    crc_from_sender = data_stream.readUInt16()
    if crc_from_sender != crc16:
        rospy.loginfo("[" + rospy.get_name() + "]:::CRC Checksum Failled")
        return None
    return ret


def linear_map(x, out_min, out_max, flip=False):
    in_min, in_max = (0, 1) if not flip else (1, 0)
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class TeleopKinematicsReceiver:
    def __init__(self, _name, _config):
        self.name = _name
        self.config = _config

        self.number_of_robots = len(_config['robot_kinematic_interfaces_ns'])
        self.kinematics_interfaces = [RobotKinematicsInterface(ns) for ns in _config['robot_kinematic_interfaces_ns']]
        self.gripper_drivers = [RobotDriverInterface(ns) for ns in _config['gripper_drivers_ns']]
        self.flip_gripper = _config["flip_gripper"]

        # wait for all interface enabled
        # while not all([x.is_enabled() for x in self.gripper_drivers]):
        #     rospy.loginfo(f"[{self.name}]::Waiting for gripper_drivers interfaces to be enabled")
        #     rospy.sleep(1)
        while not all([x.is_enabled() for x in self.kinematics_interfaces]):
            rospy.loginfo(f"[{self.name}]::Waiting for kinematics interfaces to be enabled")
            rospy.sleep(1)
        rospy.logwarn(f"[{self.name}]::All interfaces enabled")

        self.remote_side_open_port = _config['remote_side_open_port']

    def get_number_of_robots(self):
        return self.number_of_robots

    def spin(self):
        rate = rospy.Rate(MAX_RATE)
        udp_recv_socket = QUdpSocket()
        recv_addr = QHostAddress.AnyIPv4
        udp_recv_port = int(self.remote_side_open_port)
        try:
            if udp_recv_socket.bind(recv_addr, udp_recv_port):
                rospy.loginfo(
                    "[" + rospy.get_name() + "]:::Listing on UDP Port: "
                    + str(udp_recv_port) + " IP: " + str(recv_addr)
                    + " setup success"
                )
            else:
                rospy.logerr("[" + rospy.get_name() + "]:::Listing on UDP Port: "
                             + str(udp_recv_port) + " setup failed")
                raise RuntimeError("UDP cannot open")
            udp_send_sock = QUdpSocket()
            s_time = rospy.Time.now().to_nsec()
            counter = 0
            rospy.logwarn("[" + self.name + "]:: Start Listening for data")
            while True:
                if udp_recv_socket.hasPendingDatagrams():
                    datagram, host, port = udp_recv_socket.readDatagram(udp_recv_socket.pendingDatagramSize())
                    rospy.logwarn(
                        "[" + rospy.get_name() + "]:::Receive teleop control info from ip: "
                        + str(host.toString()) + " PORT: " + str(port)
                    )
                    break

            while True:
                if udp_recv_socket.hasPendingDatagrams():
                    datagram, host, port = udp_recv_socket.readDatagram(udp_recv_socket.pendingDatagramSize())
                    data_buffer = QByteArray(datagram)
                    ret = read_byte_array(data_buffer, self.get_number_of_robots())
                    if ret is not None:
                        for ind, data in enumerate(ret):
                            self.kinematics_interfaces[ind].send_desired_pose(dql.DQ(data.desired_pose).normalize())
                            self.kinematics_interfaces[ind].send_desired_interpolator_speed(0)
                            if self.gripper_drivers[ind].is_enabled():
                                gripper_limit = self.gripper_drivers[ind].get_joint_limits()
                            else:
                                gripper_limit = [[0], [1]]
                            gripper_val = [linear_map(data.gripper[0], gripper_limit[0][0], gripper_limit[1][0], self.flip_gripper)]
                            self.gripper_drivers[ind].send_target_joint_positions([gripper_val])

                rate.sleep()

        except KeyboardInterrupt:
            rospy.loginfo("[" + self.name + "]:: Exit on keyboard interrupt")


if __name__ == '__main__':

    rospy.init_node("delay_teleop_kinematics_receiver_node",
                    disable_signals=True,
                    anonymous=False)

    name = rospy.get_name()
    params = rospy.get_param(name)

    config = {}

    try:
        config['robot_kinematic_interfaces_ns'] = params['robot_kinematic_interfaces_ns']
        config['gripper_drivers_ns'] = params['gripper_drivers_ns']
        config["flip_gripper"] = params["flip_gripper"]
        config['remote_side_open_port'] = params['remote_side_open_port']

        rospy.loginfo("[" + name + "]:: Parameter load OK.")
    except Exception as e:

        rospy.logerr(name + ": initialization ERROR on parameters")
        rospy.logerr(name + ": " + str(e))
        rospy.signal_shutdown(name + ": initialization ERROR on parameters")
        exit()

    namespace = rospy.get_namespace()

    main_h = TeleopKinematicsReceiver(name, config)

    main_h.spin()
