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
from typing import List

import numpy as np
import rospy
import os, sys, time, datetime, traceback

from PyQt5.QtCore import QByteArray, qChecksum, QDataStream, QIODevice, QBuffer
from PyQt5.QtNetwork import QUdpSocket, QHostAddress
from sas_robot_kinematics import RobotKinematicsProvider
from sas_operator_side_receiver import OperatorSideReceiverInterface, OperatorSideMasterReceiverManipulatorManager
import dqrobotics as dql

from collections import deque


class TransmitDataStruct:
    _fields = [('interface_id', 'uint', 1), ('desired_pose', 'double', 8),
               ('gripper', 'double', 1)]

    def __init__(self):
        for name_, dtype_, len_ in self._fields:
            if len_ is None:
                setattr(self, name_, np.zeros(0))
            else:
                setattr(self, name_, np.zeros(len_))


def send_packet(udp_send_sock: QUdpSocket, out_datas: List[TransmitDataStruct], udp_send_addr: QHostAddress, port: int):
    out_array = QByteArray()
    out_buffer = QBuffer(out_array)
    out_buffer.open(QIODevice.WriteOnly)
    out_stream = QDataStream(out_buffer)
    out_stream.setVersion(18)
    out_stream.setByteOrder(QDataStream.BigEndian)
    for out_data in out_datas:
        for key_, dtype_, length_ in out_data._fields:
            seg = getattr(out_data, key_)
            if length_ is None:
                out_stream.writeUInt32(len(seg))
                _length = len(seg)
            else:
                _length = length_

            if dtype_ == 'int':
                data = seg.astype(np.int64)
                for ind in range(_length):
                    out_stream.writeInt64(data[ind])
            elif dtype_ == 'uint':
                data = seg.astype(np.uint64)
                for ind in range(_length):
                    out_stream.writeUInt64(int(data[ind]))
            elif dtype_ == 'double':
                data = seg.astype(np.float64)
                for ind in range(_length):
                    out_stream.writeDouble(data[ind])
            elif type == 'bool':
                data = seg.astype(np.uint8)
                for ind in range(_length):
                    out_stream.writeUInt8(data[ind])
            else:
                raise RuntimeError("unrecognized type")
    out_stream.writeUInt16(qChecksum(out_array))
    return udp_send_sock.writeDatagram(out_array, udp_send_addr, port)


class TeleopKinematicsTransmitter:
    def __init__(self, _name, _config):
        self.name = _name
        self.config = _config

        self.number_of_robots = len(_config['robot_kinematic_interfaces_ns'])
        self.kinematics_providers = [RobotKinematicsProvider(ns) for ns in _config['robot_kinematic_interfaces_ns']]
        self.orsi = OperatorSideReceiverInterface()
        for ns in _config['manipulator_interfaces_ns']:
            self.orsi.add_manipulator_manager(ns)
        self.manipulator_managers = [self.orsi.get_manipulator_manager_ptr(ns) for ns in
                                     _config['manipulator_interfaces_ns']]

        # wait for all interface enabled
        enabled_check_list = self.manipulator_managers + self.kinematics_providers
        while not all([x.is_enabled() for x in enabled_check_list]):
            rospy.loginfo(f"[{self.name}]::Waiting for all interfaces to be enabled")
            rospy.sleep(1)
        rospy.logwarn(f"[{self.name}]::All interfaces enabled")

        self.sender_socket = QUdpSocket()

    def get_number_of_robots(self):
        return self.number_of_robots

    def get_interface_data(self, ind):
        return (self.kinematics_providers[ind].get_desired_pose(),
                self.manipulator_managers[ind].get_gripper())

    def spin(self):
        try:
            data_structs = [TransmitDataStruct() for _ in range(self.get_number_of_robots())]
            rate = rospy.Rate(self.config["transmit_rate"])
            udp_send_sock = QUdpSocket()
            s_time = rospy.Time.now().to_nsec()
            counter = 0
            rospy.logwarn("[" + self.name + "]:: Start transmitting data")
            while True:
                time_now = rospy.Time.now().to_nsec() - s_time

                interface_data = [self.get_interface_data(ind) for ind in range(self.get_number_of_robots())]
                for ind, (data_struct, data) in enumerate(zip(data_structs, interface_data)):
                    data_struct.interface_id[0] = ind
                    data_struct.desired_pose[:] = data[0].vec8()
                    data_struct.gripper[0] = data[1]

                send_packet(udp_send_sock, data_structs, QHostAddress(self.config['remote_side_ip']),
                            int(self.config['remote_side_port']))
                rate.sleep()

        except KeyboardInterrupt:
            rospy.loginfo("[" + self.name + "]:: Exit on keyboard interrupt")


if __name__ == '__main__':

    rospy.init_node("delay_teleop_kinematics_transmitter_node",
                    disable_signals=True,
                    anonymous=False)

    name = rospy.get_name()
    params = rospy.get_param(name)

    config = {}

    try:
        config['robot_kinematic_interfaces_ns'] = params['robot_kinematic_interfaces_ns']
        config['manipulator_interfaces_ns'] = params['manipulator_interfaces_ns']
        config['remote_side_ip'] = params['remote_side_ip']
        config['remote_side_port'] = params['remote_side_port']
        config["transmit_rate"] = params["transmit_rate"]

        rospy.loginfo("[" + name + "]:: Parameter load OK.")
    except Exception as e:

        rospy.logerr(name + ": initialization ERROR on parameters")
        rospy.logerr(name + ": " + str(e))
        rospy.signal_shutdown(name + ": initialization ERROR on parameters")
        exit()

    namespace = rospy.get_namespace()

    main_h = TeleopKinematicsTransmitter(name, config)

    main_h.spin()
