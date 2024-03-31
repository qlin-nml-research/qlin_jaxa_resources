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
        MainWindow.resize(896, 574)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.gridLayoutWidget = QWidget(self.centralwidget)
        self.gridLayoutWidget.setObjectName(u"gridLayoutWidget")
        self.gridLayoutWidget.setGeometry(QRect(0, 0, 856, 551))
        self.mainGridLayout = QGridLayout(self.gridLayoutWidget)
        self.mainGridLayout.setObjectName(u"mainGridLayout")
        self.mainGridLayout.setContentsMargins(0, 0, 0, 0)
        self.videoParentWidget = QWidget(self.gridLayoutWidget)
        self.videoParentWidget.setObjectName(u"videoParentWidget")
        self.videoParentWidget.setMinimumSize(QSize(640, 360))
        self.videoParentWidget.setMaximumSize(QSize(1920, 1080))

        self.mainGridLayout.addWidget(self.videoParentWidget, 0, 1, 1, 1)

        self.metaInfoGridLayout = QGridLayout()
        self.metaInfoGridLayout.setObjectName(u"metaInfoGridLayout")
        self.metaInfoGridLayout.setSizeConstraint(QLayout.SetDefaultConstraint)
        self.metaInfoGridLayout.setHorizontalSpacing(10)
        self.progressBar = QProgressBar(self.gridLayoutWidget)
        self.progressBar.setObjectName(u"progressBar")
        self.progressBar.setEnabled(True)
        self.progressBar.setValue(24)
        self.progressBar.setAlignment(Qt.AlignCenter)
        self.progressBar.setOrientation(Qt.Vertical)

        self.metaInfoGridLayout.addWidget(self.progressBar, 0, 0, 1, 1)

        self.label_2 = QLabel(self.gridLayoutWidget)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setMinimumSize(QSize(0, 40))

        self.metaInfoGridLayout.addWidget(self.label_2, 2, 1, 1, 1)

        self.progressBar_2 = QProgressBar(self.gridLayoutWidget)
        self.progressBar_2.setObjectName(u"progressBar_2")
        self.progressBar_2.setValue(24)
        self.progressBar_2.setOrientation(Qt.Vertical)

        self.metaInfoGridLayout.addWidget(self.progressBar_2, 0, 1, 1, 1)

        self.label = QLabel(self.gridLayoutWidget)
        self.label.setObjectName(u"label")
        self.label.setMinimumSize(QSize(0, 40))

        self.metaInfoGridLayout.addWidget(self.label, 2, 0, 1, 1)


        self.mainGridLayout.addLayout(self.metaInfoGridLayout, 0, 3, 1, 1)

        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"JAXA Leader Secondary UI", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
    # retranslateUi

