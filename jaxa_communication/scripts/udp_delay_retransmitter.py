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
import numpy as np
import rospy
import os, sys, time, datetime, traceback

from PyQt5.QtCore import QByteArray, qChecksum, QDataStream, QIODevice, QBuffer
from PyQt5.QtNetwork import QUdpSocket, QHostAddress

from collections import deque


class DataFIFOBuffer:
    def __init__(self, max_time):
        self.max_time = max_time
        self.buffer = deque(maxlen=None)

    def append(self, tstamp, data):
        self.buffer.append([tstamp, data])

    def pop(self):
        return self.buffer.popleft()

    def pop_timeout(self, t_now):
        while self.buffer and self.buffer[0][0] <= t_now - self.max_time:
            self.pop()

    def len(self):
        return len(self.buffer)

    def __iter__(self):
        return iter(self.buffer)

    def reverse_iter(self):
        return reversed(self.buffer)


def send_data_over_udp(udp_send_sock: QUdpSocket, udp_send_addr: QHostAddress, data: QByteArray, port: int):
    ret = udp_send_sock.writeDatagram(data, udp_send_addr, port)
    if ret < 0:
        rospy.logerr("[" + rospy.get_name() + "]:::UDP send failure.. to IP:" +
                     str(udp_send_addr.toString()) + " Port:" + str(port))


class DelayRetransmitterMain:
    def __init__(self, _name, _config, rate):
        self.name = _name
        self.config = _config
        rospy.logwarn("[" + rospy.get_name() + "]::Running")

        self.rt_recv_addr = QHostAddress.AnyIPv4

        # reciving socket of the information from Master Interface
        self.rt_from_recv_socket = QUdpSocket()
        # config['receiving_port'] = params['receiving_port']
        # config['retransmit_ips'] = params['retransmit_ips']
        # config['retransmit_ports'] = params['retransmit_ports']
        # config['retransmit_delay'] = params['retransmit_delay']
        if self.rt_from_recv_socket.bind(self.rt_recv_addr, self.config['receiving_port']):
            rospy.loginfo(
                "[" + rospy.get_name() + "]:::Receiver Listing on UDP Port: "
                + str(self.config['receiving_port']) + " IP: " + repr(self.rt_recv_addr)
                + " setup success"
            )
        else:
            rospy.logerr("[" + rospy.get_name() + "]:::UDP Port: "
                         + str(self.config['receiving_port']) + " setup failed")
            raise RuntimeError("UDP cannot open")

        self.rt_max_delay = (max(self.config['retransmit_delay']) + 0.1) * 1E9
        self.rt_udp_buffer = DataFIFOBuffer(max_time=self.rt_max_delay)
        self.rt_udp_send_addrs = []
        self.rt_udp_send_delay = []
        self.rt_udp_send_ports = []
        for i, (rt_ip, rt_port) in enumerate(zip(self.config['retransmit_ips'], self.config['retransmit_ports'])):
            self.rt_udp_send_addrs.append(QHostAddress(rt_ip))
            self.rt_udp_send_ports.append(int(rt_port))
            rospy.loginfo(
                "[" + rospy.get_name() + "]:::Sending to ip: "
                + str(rt_ip) + "PORT: " + str(rt_port)
                + " setup success"
            )
            if self.config['retransmit_delay'][i] < 0:
                raise RuntimeError("cannot have negative delay")
            self.rt_udp_send_delay.append(int(self.config['retransmit_delay'][i] * 1E9))

        sort_ind = sorted(range(len(self.rt_udp_send_delay)),
                          key=self.rt_udp_send_delay.__getitem__)
        self.rt_udp_send_delay = np.array(self.rt_udp_send_delay)[sort_ind]
        self.rt_udp_send_addrs = np.array(self.rt_udp_send_addrs)[sort_ind]
        self.rt_udp_send_ports = np.array(self.rt_udp_send_ports)[sort_ind]
        rospy.loginfo( "[" + rospy.get_name() + "]:::rt_udp_send_delay: "+str(self.rt_udp_send_delay))
        rospy.loginfo( "[" + rospy.get_name() + "]:::rt_udp_send_addrs: "
                       +str([addr.toString() for addr in self.rt_udp_send_addrs]))
        rospy.loginfo( "[" + rospy.get_name() + "]:::rt_udp_send_ports: "+str(self.rt_udp_send_ports))

        # wait for initial package for Master PC ip
        while not self.rt_from_recv_socket.hasPendingDatagrams():
            time.sleep(0.001)
        datagram, self.source_addr, port = self.rt_from_recv_socket.readDatagram(
            self.rt_from_recv_socket.pendingDatagramSize()
        )
        # ip of the Master Interface PC
        rospy.loginfo(
            "[" + rospy.get_name() + "]:::got Source PC IP: "
            + str(self.source_addr.toString())
        )
        self.rate = rospy.Rate(rate)

    def spin(self):
        try:
            udp_send_sock = QUdpSocket()
            s_time = rospy.Time.now().to_nsec()
            counter = 0

            while True:
                time_now = rospy.Time.now().to_nsec() - s_time

                # Master PC to all sub-instance
                """
                TODO: Logic for buffer
                """
                if self.rt_from_recv_socket.hasPendingDatagrams():
                    datagram, _, port = self.rt_from_recv_socket.readDatagram(
                        self.rt_from_recv_socket.pendingDatagramSize()
                    )
                    counter += 1
                    self.rt_udp_buffer.append(time_now, QByteArray(datagram))
                    """
                    self.rt_udp_send_addrs = []
                    self.rt_udp_buffer
                    self.rt_udp_send_delay = {}
                    """
                    # this is already iterating from the shortest delay to the largest delay
                    instance_index = 0
                    for tstamp, data_array in self.rt_udp_buffer.reverse_iter():
                        if time_now - tstamp >= self.rt_udp_send_delay[instance_index]:
                            # print(time_now, "sending instance", instance_index)
                            # print("time", (time_now - tstamp) / 1e9)
                            send_data_over_udp(
                                udp_send_sock,
                                self.rt_udp_send_addrs[instance_index],
                                data_array,
                                int(self.rt_udp_send_ports[instance_index])
                            )
                            instance_index += 1
                            if instance_index >= len(self.rt_udp_send_addrs):
                                self.rt_udp_buffer.pop()
                                break
                self.rate.sleep()

        except KeyboardInterrupt:
            rospy.loginfo("[" + self.name + "]:: Exit on keyboard interrupt")

MAX_RATE = 2000
if __name__ == '__main__':

    rospy.init_node("manipulator_delay_retransmitter_node",
                    disable_signals=True,
                    anonymous=False)

    name = rospy.get_name()
    params = rospy.get_param(name)

    config = {}

    try:
        config['receiving_port'] = params['receiving_port']
        config['retransmit_ips'] = params['retransmit_ips']
        config['retransmit_ports'] = params['retransmit_ports']
        config['retransmit_delay'] = params['retransmit_delay']

        rospy.loginfo("[" + name + "]:: Parameter load OK.")
    except Exception as e:

        rospy.logerr(name + ": initialization ERROR on parameters")
        rospy.logerr(name + ": " + str(e))
        rospy.signal_shutdown(name + ": initialization ERROR on parameters")
        exit()

    namespace = rospy.get_namespace()

    main_h = DelayRetransmitterMain(_name=name, _config=config, rate=MAX_RATE)

    main_h.spin()
