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
        MainWindow.resize(930, 570)
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        MainWindow.setMinimumSize(QSize(930, 570))
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.MinimumExpanding, QSizePolicy.Policy.MinimumExpanding)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.centralwidget.sizePolicy().hasHeightForWidth())
        self.centralwidget.setSizePolicy(sizePolicy1)
        self.gridLayoutWidget = QWidget(self.centralwidget)
        self.gridLayoutWidget.setObjectName(u"gridLayoutWidget")
        self.gridLayoutWidget.setGeometry(QRect(0, 10, 930, 521))
        self.mainGridLayout = QGridLayout(self.gridLayoutWidget)
        self.mainGridLayout.setObjectName(u"mainGridLayout")
        self.mainGridLayout.setContentsMargins(0, 0, 0, 0)
        self.metaInfoGridLayout = QGridLayout()
        self.metaInfoGridLayout.setObjectName(u"metaInfoGridLayout")
        self.metaInfoGridLayout.setSizeConstraint(QLayout.SetDefaultConstraint)
        self.metaInfoGridLayout.setHorizontalSpacing(10)
        self.dIndicator_arm2 = QProgressBar(self.gridLayoutWidget)
        self.dIndicator_arm2.setObjectName(u"dIndicator_arm2")
        self.dIndicator_arm2.setValue(24)
        self.dIndicator_arm2.setOrientation(Qt.Vertical)

        self.metaInfoGridLayout.addWidget(self.dIndicator_arm2, 1, 2, 1, 1)

        self.dIndicatorDepth_arm2_label = QLabel(self.gridLayoutWidget)
        self.dIndicatorDepth_arm2_label.setObjectName(u"dIndicatorDepth_arm2_label")
        self.dIndicatorDepth_arm2_label.setMinimumSize(QSize(80, 40))
        self.dIndicatorDepth_arm2_label.setMaximumSize(QSize(80, 16777215))

        self.metaInfoGridLayout.addWidget(self.dIndicatorDepth_arm2_label, 3, 2, 1, 1)

        self.dIndicator_arm1 = QProgressBar(self.gridLayoutWidget)
        self.dIndicator_arm1.setObjectName(u"dIndicator_arm1")
        self.dIndicator_arm1.setEnabled(True)
        self.dIndicator_arm1.setValue(24)
        self.dIndicator_arm1.setAlignment(Qt.AlignCenter)
        self.dIndicator_arm1.setTextVisible(True)
        self.dIndicator_arm1.setOrientation(Qt.Vertical)
        self.dIndicator_arm1.setInvertedAppearance(False)
        self.dIndicator_arm1.setTextDirection(QProgressBar.TopToBottom)

        self.metaInfoGridLayout.addWidget(self.dIndicator_arm1, 1, 1, 1, 1)

        self.armDepthLabel = QLabel(self.gridLayoutWidget)
        self.armDepthLabel.setObjectName(u"armDepthLabel")
        self.armDepthLabel.setMinimumSize(QSize(100, 0))
        self.armDepthLabel.setMaximumSize(QSize(100, 16777215))

        self.metaInfoGridLayout.addWidget(self.armDepthLabel, 3, 0, 1, 1)

        self.armDepthLabel_2 = QLabel(self.gridLayoutWidget)
        self.armDepthLabel_2.setObjectName(u"armDepthLabel_2")
        self.armDepthLabel_2.setMaximumSize(QSize(70, 16777215))

        self.metaInfoGridLayout.addWidget(self.armDepthLabel_2, 1, 0, 1, 1)

        self.dIndicatorDepth_arm1_label = QLabel(self.gridLayoutWidget)
        self.dIndicatorDepth_arm1_label.setObjectName(u"dIndicatorDepth_arm1_label")
        self.dIndicatorDepth_arm1_label.setMinimumSize(QSize(80, 40))
        self.dIndicatorDepth_arm1_label.setMaximumSize(QSize(80, 16777215))

        self.metaInfoGridLayout.addWidget(self.dIndicatorDepth_arm1_label, 3, 1, 1, 1)

        self.dIndicator_arm1_label = QLabel(self.gridLayoutWidget)
        self.dIndicator_arm1_label.setObjectName(u"dIndicator_arm1_label")
        self.dIndicator_arm1_label.setMinimumSize(QSize(80, 20))
        self.dIndicator_arm1_label.setMaximumSize(QSize(80, 20))

        self.metaInfoGridLayout.addWidget(self.dIndicator_arm1_label, 2, 1, 1, 1)

        self.dIndicator_arm2_label = QLabel(self.gridLayoutWidget)
        self.dIndicator_arm2_label.setObjectName(u"dIndicator_arm2_label")
        self.dIndicator_arm2_label.setMinimumSize(QSize(80, 20))
        self.dIndicator_arm2_label.setMaximumSize(QSize(80, 20))

        self.metaInfoGridLayout.addWidget(self.dIndicator_arm2_label, 2, 2, 1, 1)

        self.arm1Label = QLabel(self.gridLayoutWidget)
        self.arm1Label.setObjectName(u"arm1Label")
        self.arm1Label.setMinimumSize(QSize(80, 20))
        self.arm1Label.setMaximumSize(QSize(78, 20))

        self.metaInfoGridLayout.addWidget(self.arm1Label, 0, 1, 1, 1)

        self.arm2Label = QLabel(self.gridLayoutWidget)
        self.arm2Label.setObjectName(u"arm2Label")
        self.arm2Label.setMinimumSize(QSize(80, 20))
        self.arm2Label.setMaximumSize(QSize(78, 20))

        self.metaInfoGridLayout.addWidget(self.arm2Label, 0, 2, 1, 1)


        self.mainGridLayout.addLayout(self.metaInfoGridLayout, 0, 3, 1, 1)

        self.videoMetaLabel = QLabel(self.gridLayoutWidget)
        self.videoMetaLabel.setObjectName(u"videoMetaLabel")
        self.videoMetaLabel.setMinimumSize(QSize(0, 30))
        self.videoMetaLabel.setMaximumSize(QSize(1920, 30))
        self.videoMetaLabel.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)

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
        self.dIndicatorDepth_arm2_label.setText(QCoreApplication.translate("MainWindow", u"Arm2", None))
        self.armDepthLabel.setText(QCoreApplication.translate("MainWindow", u"Depth [cm]\n"
"(from Camera)", None))
        self.armDepthLabel_2.setText(QCoreApplication.translate("MainWindow", u"Est. Depth\n"
"to Platform", None))
        self.dIndicatorDepth_arm1_label.setText(QCoreApplication.translate("MainWindow", u"Arm1", None))
        self.dIndicator_arm1_label.setText(QCoreApplication.translate("MainWindow", u"Arm1", None))
        self.dIndicator_arm2_label.setText(QCoreApplication.translate("MainWindow", u"Arm1", None))
        self.arm1Label.setText(QCoreApplication.translate("MainWindow", u"Arm1", None))
        self.arm2Label.setText(QCoreApplication.translate("MainWindow", u"Arm2", None))
        self.videoMetaLabel.setText(QCoreApplication.translate("MainWindow", u"fpn n...", None))
        self.videoImageLabel.setText(QCoreApplication.translate("MainWindow", u"IMAGE_LABEL", None))
    # retranslateUi

