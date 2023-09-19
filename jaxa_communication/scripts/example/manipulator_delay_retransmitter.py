#!/usr/bin/python3
import math

import numpy as np
import rospy
import os, sys, time, datetime, traceback


from PyQt5.QtCore import QByteArray, qChecksum, QDataStream, QIODevice, QBuffer
from PyQt5.QtNetwork import QUdpSocket, QHostAddress

from sas_operator_side_receiver import OperatorSideReceiverInterface
import dqrobotics as dql

from manipulator_delay_common import manipulator_udp_format

timed_out = 1.0  # sec
rate_limit = 1000


class TransmitDataStruct:
    _fields = manipulator_udp_format

    def __init__(self):
        for name_, dtype_, len_ in self._fields:
            setattr(self, name_, np.zeros(len_))

def delay_retransmitter_main(_name, _config):
    rospy.logwarn("[" + rospy.get_name() + "]::Running")

    udp_send_sock = QUdpSocket()
    udp_send_addr = QHostAddress(_config['remote_ip'])
    rospy.loginfo(
        "[" + rospy.get_name() + "]:::Sending to ip: "
        + str(_config['remote_ip'])
        + " setup success"
    )
    # Initialize the OperatorSideReceiverInterface
    osri = OperatorSideReceiverInterface()

    # Each master manipulator will have a label assigned to it.
    # For example, 0_1 means computer 0, manipulator 1
    for manipulator_ns in _config['manipulators_list']:
        osri.add_manipulator_manager(manipulator_ns)

    # If you want any information from that master, retrieve it first
    manipulators_list = []
    for manipulator_ns in _config['manipulators_list']:
        manipulators_list.append(osri.get_manipulator_manager_ptr(manipulator_ns))

    rospy.loginfo(
        "[" + rospy.get_name() + "]:::Manipulators initialization success"
    )

    try:
        updated_time = rospy.get_time()
        rate = rospy.Rate(rate_limit)
        while True:
            if rospy.get_time() - updated_time > timed_out:
                updated_time = rospy.get_time()

            out_data = TransmitDataStruct()

            for manipulator_ns in _config['manipulators_list']:
                pass




            outArray = QByteArray()
            outBuffer = QBuffer(outArray)
            outBuffer.open(QIODevice.WriteOnly)
            outStream = QDataStream(outBuffer)
            outStream.setVersion(18)
            outStream.setByteOrder(QDataStream.BigEndian)
            for key_, dtype_, length_ in out_data._fields:
                seg = getattr(out_data, key_)
                for ind in range(length_):
                    if dtype_ == 'str':
                        outStream.writeQString(qstr=seg[ind])
                    if dtype_ == 'int':
                        outStream.writeInt32(seg[ind])
                    if dtype_ == 'float':
                        outStream.writeFloat(seg[ind])
                    if type == 'bool':
                        outStream.writeInt8(seg[ind])
            outStream.writeUInt16(qChecksum(outArray))
            ret = udp_send_sock.writeDatagram(outArray, udp_send_addr, _config['port'])
            if ret < 0:
                rospy.logerr("[" + rospy.get_name() + "]:::UDP send failure..")

            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("[" + rospy.get_name() + "]:::exit on keyboard interrupt")

    pass


if __name__ == '__main__':

    rospy.init_node("manipulator_delay_retransmitter_node",
                    disable_signals=True,
                    anonymous=False)

    name = rospy.get_name()
    params = rospy.get_param(name)

    config = {}

    try:
        config['port'] = params['port']
        config['remote_ip'] = params['remote_ip']
        config['delay'] = params['delay']
        config['manipulators_list'] = params['manipulators_list']

        rospy.loginfo("[" + name + "]:: Parameter load OK.")
    except Exception as e:

        rospy.logerr(name + ": initialization ERROR on parameters")
        rospy.logerr(name + ": " + str(e))
        rospy.signal_shutdown(name + ": initialization ERROR on parameters")
        exit()

    namespace = rospy.get_namespace()

    delay_retransmitter_main(name, config)
