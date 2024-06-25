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
try:
    import rospy

    NOT_ROS = False
except ImportError:
    import logging

    logging.warning("rospy not found, using logging module")
    NOT_ROS = True

# setup logger retargeting
if NOT_ROS:
    import time

    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger(__name__)
    log_error = logger.error
    log_warn = logger.warning
    log_info = logger.info
    get_name = lambda: "udp teleop re-transmitter"
    get_nsec = time.perf_counter_ns
    is_shutdown = lambda: False
else:
    log_error = rospy.logerr
    log_warn = rospy.logwarn
    log_info = rospy.loginfo
    get_name = rospy.get_name
    get_nsec = rospy.Time.now().to_nsec
    is_shutdown = rospy.is_shutdown

import math
import numpy as np
import os, sys, time, datetime, traceback

from PyQt5.QtCore import QByteArray, qChecksum, QDataStream, QIODevice, QBuffer
from PyQt5.QtNetwork import QUdpSocket, QHostAddress

from collections import deque
import pickle
import tqdm
from utils import RateManager

DISABLE_MAX_RATE = False


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
        log_error(f"[{get_name()}]:::UDP send failure.. to IP:" +
                  str(udp_send_addr.toString()) + " Port:" + str(port))


class Logger:
    def __init__(self, log_file_path):
        self.log_file_path = log_file_path
        os.makedirs(os.path.dirname(self.log_file_path), exist_ok=True)
        self.log_f = open(self.log_file_path, 'wb')

        self.data = []

    def append_data(self, time_s, data):
        self.data.append([time_s, data])

    def print_size(self):
        print(len(self.data))

    def save(self):
        log_warn(f"[{get_name()}]::Logger: saving to path: " + self.log_file_path)
        pickle.dump(self.data, self.log_f)

    def __del__(self):
        self.log_f.close()


class DelayRetransmitterMain:
    def __init__(self, _name, _config, rate):
        self.name = _name
        self.config = _config
        log_warn(f"[{self.name}]::Initializing")

        if _config['enable_logging_to_path'] is not None:
            log_warn(f"[{self.name}]::Logger enabled")
            self.logger = Logger(_config['enable_logging_to_path'])
        else:
            self.logger = None

        if _config['replay_file'] is not None:
            self.rt_from_recv_socket = None
        else:
            self.rt_recv_addr = QHostAddress.AnyIPv4
            # reciving socket of the information Source
            self.rt_from_recv_socket = QUdpSocket()
            if self.rt_from_recv_socket.bind(self.rt_recv_addr, self.config['receiving_port']):
                log_info(
                    f"[{self.name}]:::Receiver Listing on UDP Port: {self.config['receiving_port']} "
                    f"IP: {repr(self.rt_recv_addr)} setup success"
                )
            else:
                log_error(f"[{self.name}]:::UDP Port: {self.config['receiving_port']} setup failed")
                raise RuntimeError("UDP cannot open")

        self.rt_max_delay = (max(self.config['retransmit_delay']) + 0.1) * 1E9
        self.rt_udp_buffer = DataFIFOBuffer(max_time=self.rt_max_delay)
        self.rt_udp_send_addrs = []
        self.rt_udp_send_delay = []
        self.rt_udp_send_ports = []
        for i, (rt_ip, rt_port) in enumerate(zip(self.config['retransmit_ips'], self.config['retransmit_ports'])):
            self.rt_udp_send_addrs.append(QHostAddress(rt_ip))
            self.rt_udp_send_ports.append(int(rt_port))
            log_info(
                f"[{self.name}]:::Sending to ip: "
                + str(rt_ip) + " PORT: " + str(rt_port)
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
        log_info(f"[{self.name}]:::rt_udp_send_delay: {self.rt_udp_send_delay}")
        log_info(f"[{self.name}]:::rt_udp_send_addrs: {str([addr.toString() for addr in self.rt_udp_send_addrs])}")
        log_info(f"[{self.name}]:::rt_udp_send_ports: {str(self.rt_udp_send_ports)}")

        self.rate = RateManager(rate)
        if _config['replay_file'] is None:
            # wait for initial package for Master PC ip
            while not self.rt_from_recv_socket.hasPendingDatagrams():
                self.rate.sleep()
            datagram, self.source_addr, port = self.rt_from_recv_socket.readDatagram(
                self.rt_from_recv_socket.pendingDatagramSize()
            )
            # ip of the Master Interface PC
            log_info(
                f"[{self.name}]:::got Source PC IP: {self.source_addr.toString()}"
            )

    def spin_iter(self, time_now, udp_send_sock):
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

    def spin(self):
        log_warn(f"[{self.name}]::Running")
        try:
            udp_send_sock = QUdpSocket()
            s_time = get_nsec()
            counter = 0

            while not is_shutdown():
                time_now = get_nsec() - s_time

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
                    if self.logger is not None:
                        self.logger.append_data(time_now, QByteArray(datagram))
                    self.spin_iter(time_now, udp_send_sock)
                if not DISABLE_MAX_RATE:
                    self.rate.sleep()
            log_info(f"[{self.name}]:: Shutdown signal received")
        except KeyboardInterrupt as e:
            log_info(f"[{self.name}]:: Exit on keyboard interrupt")

        if self.logger is not None:
            self.logger.save()

    def replay_spin(self, replay_file):
        try:
            with open(replay_file, 'rb') as f:
                log_data = pickle.load(f)
            log_warn(f"[{self.name}]::Enabling spin in replay mode: Duration: {str(log_data[-1][0] / 1E9)} seconds")
        except Exception as e:
            log_error(f"[{self.name}]:: Replay Error: failed to read log file")
        try:
            udp_send_sock = QUdpSocket()
            counter = 0
            if USE_GUI_TQDM:
                p_bar = tqdm.tqdm_gui(
                    desc="Replay Status",
                    total=len(log_data),
                    position=0,
                    leave=False
                )
            else:
                p_bar = tqdm.tqdm(
                    desc="Replay Status",
                    total=len(log_data),
                    position=0,
                    leave=False
                )
            s_time = get_nsec()
            log_warn(f"[{self.name}]::Start time: {s_time}")
            while not is_shutdown():
                time_now = get_nsec() - s_time
                if log_data[counter][0] <= time_now:
                    counter += 1
                    p_bar.update(1)
                    if counter >= len(log_data):
                        p_bar.close()
                        log_warn(f"[{self.name}]::End of replay file")
                        break
                    self.rt_udp_buffer.append(time_now, log_data[counter][1])
                    self.spin_iter(time_now, udp_send_sock)
                if not DISABLE_MAX_RATE:
                    self.rate.sleep()
            log_info(f"[{self.name}]:: Shutdown signal received")
        except KeyboardInterrupt as e:
            log_info(f"[{self.name}]:: Exit on keyboard interrupt")

        p_bar.close()


USE_GUI_TQDM = False

NON_ROS_INIT_PARAM = {
    "retransmit_ips": ["10.198.113.140"],
    "retransmit_ports": [2223],
    "retransmit_delay": [0],
    "replay_file": "../log/11_20_test_random_move-2.udp_log",
}

MAX_RATE = 2000
if __name__ == '__main__':
    if not NOT_ROS:
        rospy.init_node("manipulator_delay_retransmitter_node",
                        disable_signals=True,
                        anonymous=False)
        name = get_name()
        params = rospy.get_param(name)
    else:
        name = get_name()
        params = NON_ROS_INIT_PARAM

    config = {}

    try:
        config['retransmit_ips'] = params['retransmit_ips']
        config['retransmit_ports'] = params['retransmit_ports']
        config['retransmit_delay'] = params['retransmit_delay']
        config['enable_logging_to_path'] = params.get("enable_logging_to_path", None)
        config['replay_file'] = params.get("replay_file", None)

        if config['enable_logging_to_path'] is not None:
            assert config['replay_file'] is None, "Replay cannot be used if logging is enabled"
        if config['replay_file'] is not None:
            assert config['enable_logging_to_path'] is None, "Logging cannot be enabled if set to replay"
            assert os.path.isfile(config['replay_file']), "Defined replay file is not a file"
        else:
            config['receiving_port'] = params['receiving_port']

        log_info(f"[{name}]:: Parameter load OK.")
    except Exception as e:

        log_error(f"[{name}]:: initialization ERROR on parameters")
        log_error(f"[{name}]:: WHAT:{e}")
        if not NOT_ROS:
            rospy.signal_shutdown(name + ": initialization ERROR on parameters")
        exit()

    main_h = DelayRetransmitterMain(_name=name, _config=config, rate=MAX_RATE)

    if config['replay_file'] is not None:
        main_h.replay_spin(config['replay_file'])
    else:
        if NOT_ROS:
            log_error(f"[{get_name()}]::ROS not found, cannot run in non-replay mode")
            raise RuntimeError("ROS not found")
        main_h.spin()

    log_warn(f"[{get_name()}]::Safe exit")

    # # delete parameter on exit
    # rospy.delete_param(os.path.join(name, "enable_logging_to_path"))
    # rospy.delete_param(os.path.join(name, "replay_file"))
