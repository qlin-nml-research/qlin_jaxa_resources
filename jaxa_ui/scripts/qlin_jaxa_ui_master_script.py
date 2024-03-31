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
import sys
import time
import dqrobotics as dql

import cv2
from PySide6 import QtGui
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import QApplication, QMainWindow, QLabel
import logging

# Important:
# You need to run the following command to generate the ui_form.py file
#     pyside6-uic form.ui -o ui_form.py, or
#     pyside2-uic form.ui -o ui_form.py
from JaxaMasterSecondaryUI.ui_form import Ui_MainWindow

from tools import AsyncVideoCapture, InfoReceiver

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    def __init__(self, video_device, kin_info_receiver, parent=None):
        super().__init__(parent)
        logger.info(f"MainWindow::init::initializing")
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        #####################################
        # video related
        #####################################
        self.video_device = video_device
        self.frame_counter = 0
        self.frame_time = time.perf_counter()
        # create video image display label
        # self.image_label = self.ui.videoImageLabel
        self.im_display_size = (640, 360)

        #####################################
        # kinematics info receiver
        #####################################
        self.kin_info_receiver = kin_info_receiver
        if info_receiver is not None:
            self.kin_info_receiver.wait_for_initialization(timeout=1)
        logger.info(f"MainWindow::init::kin_info_receiver initialized")

        #####################################
        # UI update
        #####################################
        self.timer_vid = QTimer(self)
        self.timer_vid.timeout.connect(self.update_frame)
        self.timer_vid.start(10)
        self.timer_kin = QTimer(self)
        self.timer_kin.timeout.connect(self.update_kin_info)
        self.timer_kin.start(100)

        # self.setFixedSize(self.ui.mainGridLayout.sizeHint())

        logger.info(f"MainWindow::init::initialized")

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(*self.im_display_size, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

    def update_kin_info(self):
        if self.kin_info_receiver is None:
            return
        logger.info(f"MainWindow::update_kin_info::updating kin info")
        x1, x2, cam = self.kin_info_receiver.get_kin_info()

        # self.ui.kinInfoLabel.setText(f"x1: {x1}, x2: {x2}, cam: {cam}")

    def update_frame(self):
        ret, frame = self.video_device.read()
        if ret:
            # self.ui.label.setPixmap(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            qt_img = self.convert_cv_qt(frame)
            self.ui.videoImageLabel.setPixmap(qt_img)

            ctime = time.perf_counter()
            fps = 1.0 / (ctime - self.frame_time)
            self.frame_time = ctime
            self.ui.videoMetaLabel.setText(f"Frame: {self.frame_counter}, FPS: {fps:.2f}")
            self.frame_counter += 1


if __name__ == "__main__":
    tool_info_reciever_port = 20034

    # testing
    cap = AsyncVideoCapture(0)
    info_receiver = InfoReceiver(tool_info_reciever_port, blocking=True)
    # info_receiver = None

    # cap = AsyncDecklinkCapture(0)
    app = QApplication(sys.argv)
    try:
        widget = MainWindow(video_device=cap, kin_info_receiver=info_receiver)
        widget.show()
    except Exception as e:
        print(e)

    exc_code = app.exec()
    cap.release()
    if info_receiver is not None:
        info_receiver.exit()
    sys.exit(exc_code)
