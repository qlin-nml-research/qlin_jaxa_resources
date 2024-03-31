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
import errno

import numpy as np
import dqrobotics as dql
import socket
import time
import multiprocessing as mp
import logging

logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

############################
# udp formatting
# ############################
# "{x1/x2}: {t_x [mm]} {t_y [mm]} {t_z [mm]} {r_x [rad]} {r_y [rad]} {r_z [rad]} {r_w [rad]} {gripper}"
# "{cam}: {t_x [mm]} {t_y [mm]} {t_z [mm]} {r_x [rad]} {r_y [rad]} {r_z [rad]} {r_w [rad]}"

EXPECTED_DATA_KEY = ["x0", "x1", "cam"]


class _ReceiverProcess(mp.Process):
    def __init__(self, port, exit_event=None):
        super(_ReceiverProcess, self).__init__()
        self.port = port
        if exit_event is None:
            exit_event = mp.Event()
        self.exit_event = exit_event
        self.exit_event.clear()
        self.error_event = mp.Event()
        self.error_event.clear()
        self.initialized_event = mp.Event()
        self.initialized_event.clear()

        self.kin_info_queue = mp.Queue(maxsize=100)
        self.kin_info_request_event = mp.Event()
        self.kin_info_request_event.set()

    def exit(self):
        self.exit_event.set()
        logger.info(f"ReceiverProcess::exit called")

    def get_kin_info(self):
        # this will get the existing data in the Queue, but if loop is fast, info should be up to date
        assert self.initialized_event.is_set(), "ReceiverProcess::get_kin_info::Not initialized"
        data_dict = self.kin_info_queue.get()
        self.kin_info_request_event.set()
        return {key: dql.DQ(data_dict[key]) for key in EXPECTED_DATA_KEY}

    def _has_request(self):
        return self.kin_info_request_event.is_set()

    def _send_data_dict(self, data_dict: dict):
        self.kin_info_queue.put({key: data.vec8() for key, data in data_dict.items()})

    def is_initialized(self):
        return self.initialized_event.is_set()

    def run(self):
        logger.info(f"ReceiverProcess::run::Start receiving data from port {self.port}")
        socket_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        requested_data = {key: dql.DQ([1]) for key in EXPECTED_DATA_KEY}
        socket_.bind(("", self.port))
        socket_.setblocking(0)
        logger.info(f"ReceiverProcess::run::Socket binded to port {self.port}")
        self.initialized_event.set()
        try:
            while not self.exit_event.is_set():
                time.sleep(0.001)  # set max rate
                if self._has_request():
                    self._send_data_dict(requested_data)
                    self.kin_info_request_event.clear()
                try:
                    data = socket_.recv(4096)
                    # logger.info(f"ReceiverProcess::run::Received: {data}")
                    data = data.decode("utf-8")
                    id_key, data = data.split(":")
                    data = data.strip().split(" ")
                    data = [float(d) for d in data]
                    t = dql.DQ([data[0], data[1], data[2]]) * 0.001
                    r = dql.DQ([data[6], data[3], data[4], data[5]])
                    x = r + 0.5 * dql.E_ * t * r
                    requested_data[id_key] = x
                    # logger.info(f"ReceiverProcess::run::Received: {id_key}-> {x}")

                except socket.error as e:
                    err = e.args[0]
                    if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                        # no data to read
                        continue
        except KeyboardInterrupt:
            logger.info(f"ReceiverProcess::run::KeyboardInterrupt")
        except Exception as e:
            logger.error(f"ReceiverProcess::run::Error: {e}")
            self.error_event.set()

        logger.info(f"ReceiverProcess::run::Exit called")


class InfoReceiver:
    def __init__(self, port, blocking=False):
        self.exit_event = mp.Event()
        self.receiver_process_ = _ReceiverProcess(port, exit_event=self.exit_event)
        self.receiver_process_.start()
        self.kin_data = {key: dql.DQ([1]) for key in EXPECTED_DATA_KEY}
        if blocking:
            self.wait_for_initialization()
            if not self.receiver_process_.is_initialized():
                logger.error(f"InfoReceiver::init::Not initialized")

    def wait_for_initialization(self, timeout=None):
        while not self.receiver_process_.is_initialized():
            time.sleep(0.1)
            if timeout is not None:
                timeout -= 0.1
                if timeout <= 0:
                    logger.error(f"InfoReceiver::wait_for_initialization::timeout")
                    return False
        return True

    def exit(self, timeout=10):
        logger.info(f"InfoReceiver::exit called")
        self.exit_event.set()
        logger.info(f"InfoReceiver::exit::exit called")
        self.receiver_process_.join(timeout=timeout)
        if self.receiver_process_.is_alive():
            logger.error(f"InfoReceiver::exit::exit failed")
            self.receiver_process_.terminate()
        logger.info(f"InfoReceiver::exit::exit completed")

    def get_kin_info(self):
        # if self.receiver_process_.has_new_kin_info():
        #     logger.error(f"InfoReceiver::get_kin_info::has new new data")
        self.kin_data = self.receiver_process_.get_kin_info()

        return self.kin_data["x0"], self.kin_data["x1"], self.kin_data["cam"]


if __name__ == "__main__":
    receiver = InfoReceiver(20034, blocking=True)
    counter = 0

    try:
        while True:
            s_time = time.perf_counter()
            x1, x2, cam = receiver.get_kin_info()
            e_time = time.perf_counter()
            logger.info(f"{counter}:: x1: {x1}, x2: {x2}, cam: {cam}, dt: {e_time - s_time}")
            # logger.info(f"{counter}:: x1: {x1}, x2: {x2}, cam: {cam}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        logger.info(f"exit on keyboard interrupt")
    finally:
        receiver.exit()
        logger.info(f"exit completed")
