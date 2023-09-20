#!/usr/bin/python3
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
    def __init__(self, _name, _config):
        self.name = _name
        self.config = _config
        rospy.logwarn("[" + rospy.get_name() + "]::Running")

        self.rt_recv_addr = QHostAddress.AnyIPv4

        # reciving socket of the information from Master Interface
        self.rt_from_master_recv_socket = QUdpSocket()
        if self.rt_from_master_recv_socket.bind(self.rt_recv_addr, self.config['patient_side_port']):
            rospy.loginfo(
                "[" + rospy.get_name() + "]:::Master Receiver Listing on UDP Port: "
                + str(self.config['patient_side_port']) + " IP: " + repr(self.rt_recv_addr)
                + " setup success"
            )
        else:
            rospy.logerr("[" + rospy.get_name() + "]:::UDP Port: "
                         + str(self.config['patient_side_port']) + " setup failed")
            raise RuntimeError("UDP cannot open")

        if sum(self.config['retransmit_to_master_select']) > 1:
            rospy.logwarn("[" + rospy.get_name() + "]::retransmit_to_master_select:"
                          + str(self.config['retransmit_to_master_select']))
            raise RuntimeError("More then 1 re transmit to master selected..")

        self.rt_to_master_recv_socket = QUdpSocket()
        for flag, rt_port in zip(self.config['retransmit_to_master_select'],
                                 self.config['retransmit_os_ports']):
            if flag:
                if self.rt_to_master_recv_socket.bind(self.rt_recv_addr, rt_port):
                    rospy.loginfo(
                        "[" + rospy.get_name() + "]:::Retransmitter Listing on UDP Port: "
                        + str(rt_port) + " IP: " + repr(self.rt_recv_addr)
                        + " For master information: setup success"
                    )
                else:
                    rospy.logerr("[" + rospy.get_name() + "]:::UDP Port: "
                                 + str(rt_port) + " setup failed")
                    raise RuntimeError("UDP cannot open")
                break

        self.rt_max_delay = (max(self.config['retransmit_delay']) + 0.1) * 1E9
        self.rt_udp_InfoFromMaster_buffer = DataFIFOBuffer(max_time=self.rt_max_delay)
        self.rt_udp_InfoFromMaster_send_addrs = []
        self.rt_udp_InfoFromMaster_send_delay = []
        self.rt_udp_InfoFromMaster_send_ports = []
        for i, (rt_ip, rt_port) in enumerate(zip(self.config['retransmit_ips'], self.config['retransmit_ps_ports'])):
            self.rt_udp_InfoFromMaster_send_addrs.append(QHostAddress(rt_ip))
            self.rt_udp_InfoFromMaster_send_ports.append(int(rt_port))
            rospy.loginfo(
                "[" + rospy.get_name() + "]:::Sending to ip: "
                + str(rt_ip) + "PORT: " + str(rt_port)
                + " setup success"
            )
            if self.config['retransmit_delay'][i] < 0:
                raise RuntimeError("cannot have negative delay")
            self.rt_udp_InfoFromMaster_send_delay.append(int(self.config['retransmit_delay'][i] * 1E9))

        sort_ind = sorted(range(len(self.rt_udp_InfoFromMaster_send_delay)),
                          key=self.rt_udp_InfoFromMaster_send_delay.__getitem__)
        self.rt_udp_InfoFromMaster_send_delay = np.array(self.rt_udp_InfoFromMaster_send_delay)[sort_ind]
        self.rt_udp_InfoFromMaster_send_addrs = np.array(self.rt_udp_InfoFromMaster_send_addrs)[sort_ind]
        self.rt_udp_InfoFromMaster_send_ports = np.array(self.rt_udp_InfoFromMaster_send_ports)[sort_ind]


        print(self.rt_udp_InfoFromMaster_send_delay)
        print(self.rt_udp_InfoFromMaster_send_addrs)
        print(self.rt_udp_InfoFromMaster_send_ports)

        # wait for initial package for Master PC ip
        while not self.rt_from_master_recv_socket.hasPendingDatagrams():
            time.sleep(0.001)
        datagram, self.master_pc_addr, port = self.rt_from_master_recv_socket.readDatagram(
            self.rt_from_master_recv_socket.pendingDatagramSize()
        )
        # ip of the Master Interface PC
        rospy.loginfo(
            "[" + rospy.get_name() + "]:::got Master PC IP: "
            + str(self.master_pc_addr.toString())
        )

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
                if self.rt_from_master_recv_socket.hasPendingDatagrams():
                    datagram, _, port = self.rt_from_master_recv_socket.readDatagram(
                        self.rt_from_master_recv_socket.pendingDatagramSize()
                    )
                    counter += 1
                    self.rt_udp_InfoFromMaster_buffer.append(time_now, QByteArray(datagram))
                    """
                    self.rt_udp_InfoFromMaster_send_addrs = []
                    self.rt_udp_InfoFromMaster_buffer
                    self.rt_udp_InfoFromMaster_send_delay = {}
                    """
                    # this is already iterating from the shortest delay to the largest delay
                    instance_index = 0
                    for tstamp, data_array in self.rt_udp_InfoFromMaster_buffer.reverse_iter():
                        if time_now - tstamp >= self.rt_udp_InfoFromMaster_send_delay[instance_index]:
                            # print(time_now, "sending instance", instance_index)
                            # print("time", (time_now - tstamp) / 1e9)
                            send_data_over_udp(
                                udp_send_sock,
                                self.rt_udp_InfoFromMaster_send_addrs[instance_index],
                                data_array,
                                self.rt_udp_InfoFromMaster_send_ports[instance_index]
                            )
                            instance_index += 1
                            if instance_index >= len(self.rt_udp_InfoFromMaster_send_addrs):
                                self.rt_udp_InfoFromMaster_buffer.pop()
                                break

                    # self.rt_udp_InfoFromMaster_buffer.pop_timeout(time_now) # should not be needed
                    # rospy.loginfo("current len:"+str(self.rt_udp_InfoFromMaster_buffer.len()))
                    # rospy.loginfo("oldest time:"+str((time_now-self.rt_udp_InfoFromMaster_buffer.buffer[0][0])/1e9))

                # selected sub-instance to Master PC
                if self.rt_to_master_recv_socket.hasPendingDatagrams():
                    # rospy.loginfo("[" + rospy.get_name() + "]::got info to master")
                    datagram, _, port = self.rt_to_master_recv_socket.readDatagram(
                        self.rt_to_master_recv_socket.pendingDatagramSize()
                    )
                    data_array = QByteArray(datagram)
                    send_data_over_udp(udp_send_sock, self.master_pc_addr,
                                       data_array, int(self.config['operator_side_port']))

        except KeyboardInterrupt:
            rospy.loginfo("[" + self.name + "]:: Exit on keyboard interrupt")


if __name__ == '__main__':

    rospy.init_node("manipulator_delay_retransmitter_node",
                    disable_signals=True,
                    anonymous=False)

    name = rospy.get_name()
    params = rospy.get_param(name)

    config = {}

    try:
        config['patient_side_port'] = params['patient_side_port']
        config['operator_side_port'] = params['operator_side_port']
        config['retransmit_ips'] = params['retransmit_ips']
        config['retransmit_ps_ports'] = params['retransmit_ps_ports']
        config['retransmit_os_ports'] = params['retransmit_os_ports']
        config['retransmit_delay'] = params['retransmit_delay']
        config['retransmit_to_master_select'] = params['retransmit_to_master_select']

        """
            patient_side_port: 2220
            operator_side_port: 2220
            retransmit_ips: ["127.0.0.1", "10.198.113.184"]
            retransmit_ps_ports: [2221, 2223]
            retransmit_os_ports: [2221, 2223]
            retransmit_delay: [0, 1]
            retransmit_to_master_select: [false, true]
        """

        rospy.loginfo("[" + name + "]:: Parameter load OK.")
    except Exception as e:

        rospy.logerr(name + ": initialization ERROR on parameters")
        rospy.logerr(name + ": " + str(e))
        rospy.signal_shutdown(name + ": initialization ERROR on parameters")
        exit()

    namespace = rospy.get_namespace()

    main_h = DelayRetransmitterMain(name, config)

    main_h.spin()
