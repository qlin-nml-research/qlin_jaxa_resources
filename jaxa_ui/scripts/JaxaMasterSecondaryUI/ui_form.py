# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'form.ui'
##
## Created by: Qt User Interface Compiler version 6.6.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QGridLayout, QLabel, QLayout,
    QMainWindow, QProgressBar, QSizePolicy, QStatusBar,
    QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(864, 591)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.gridLayoutWidget = QWidget(self.centralwidget)
        self.gridLayoutWidget.setObjectName(u"gridLayoutWidget")
        self.gridLayoutWidget.setGeometry(QRect(0, 10, 856, 551))
        self.mainGridLayout = QGridLayout(self.gridLayoutWidget)
        self.mainGridLayout.setObjectName(u"mainGridLayout")
        self.mainGridLayout.setContentsMargins(0, 0, 0, 0)
        self.metaInfoGridLayout = QGridLayout()
        self.metaInfoGridLayout.setObjectName(u"metaInfoGridLayout")
        self.metaInfoGridLayout.setSizeConstraint(QLayout.SetDefaultConstraint)
        self.metaInfoGridLayout.setHorizontalSpacing(10)
        self.dIndicator_arm1 = QProgressBar(self.gridLayoutWidget)
        self.dIndicator_arm1.setObjectName(u"dIndicator_arm1")
        self.dIndicator_arm1.setEnabled(True)
        self.dIndicator_arm1.setValue(24)
        self.dIndicator_arm1.setAlignment(Qt.AlignCenter)
        self.dIndicator_arm1.setOrientation(Qt.Vertical)

        self.metaInfoGridLayout.addWidget(self.dIndicator_arm1, 0, 0, 1, 1)

        self.dIndicator_arm2_label = QLabel(self.gridLayoutWidget)
        self.dIndicator_arm2_label.setObjectName(u"dIndicator_arm2_label")
        self.dIndicator_arm2_label.setMinimumSize(QSize(0, 40))

        self.metaInfoGridLayout.addWidget(self.dIndicator_arm2_label, 2, 1, 1, 1)

        self.dIndicator_arm2 = QProgressBar(self.gridLayoutWidget)
        self.dIndicator_arm2.setObjectName(u"dIndicator_arm2")
        self.dIndicator_arm2.setValue(24)
        self.dIndicator_arm2.setOrientation(Qt.Vertical)

        self.metaInfoGridLayout.addWidget(self.dIndicator_arm2, 0, 1, 1, 1)

        self.dIndicator_arm1_label = QLabel(self.gridLayoutWidget)
        self.dIndicator_arm1_label.setObjectName(u"dIndicator_arm1_label")
        self.dIndicator_arm1_label.setMinimumSize(QSize(0, 40))

        self.metaInfoGridLayout.addWidget(self.dIndicator_arm1_label, 2, 0, 1, 1)


        self.mainGridLayout.addLayout(self.metaInfoGridLayout, 0, 3, 1, 1)

        self.videoMetaLabel = QLabel(self.gridLayoutWidget)
        self.videoMetaLabel.setObjectName(u"videoMetaLabel")
        self.videoMetaLabel.setMinimumSize(QSize(0, 30))
        self.videoMetaLabel.setMaximumSize(QSize(1920, 30))
        self.videoMetaLabel.setAlignment(Qt.AlignCenter)

        self.mainGridLayout.addWidget(self.videoMetaLabel, 1, 1, 1, 1)

        self.videoImageLabel = QLabel(self.gridLayoutWidget)
        self.videoImageLabel.setObjectName(u"videoImageLabel")
        self.videoImageLabel.setMinimumSize(QSize(640, 360))
        self.videoImageLabel.setMaximumSize(QSize(1920, 1080))
        self.videoImageLabel.setAlignment(Qt.AlignCenter)

        self.mainGridLayout.addWidget(self.videoImageLabel, 0, 1, 1, 1)

        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"JAXA Leader Secondary UI", None))
        self.dIndicator_arm2_label.setText(QCoreApplication.translate("MainWindow", u"Arm2", None))
        self.dIndicator_arm1_label.setText(QCoreApplication.translate("MainWindow", u"Arm1", None))
        self.videoMetaLabel.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.videoImageLabel.setText(QCoreApplication.translate("MainWindow", u"IMAGE_LABEL", None))
    # retranslateUi

