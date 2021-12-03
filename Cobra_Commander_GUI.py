# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Cobra_Commander_GUI.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.setEnabled(True)
        MainWindow.resize(512, 696)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.centralwidget.sizePolicy().hasHeightForWidth())
        self.centralwidget.setSizePolicy(sizePolicy)
        self.centralwidget.setObjectName("centralwidget")
        self.b_enable = QtWidgets.QPushButton(self.centralwidget)
        self.b_enable.setEnabled(False)
        self.b_enable.setGeometry(QtCore.QRect(20, 330, 471, 41))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.b_enable.setFont(font)
        self.b_enable.setObjectName("b_enable")
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QtCore.QRect(20, 390, 221, 261))
        self.groupBox.setAutoFillBackground(False)
        self.groupBox.setObjectName("groupBox")
        self.verticalLayoutWidget_3 = QtWidgets.QWidget(self.groupBox)
        self.verticalLayoutWidget_3.setGeometry(QtCore.QRect(10, 20, 201, 211))
        self.verticalLayoutWidget_3.setObjectName("verticalLayoutWidget_3")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_3)
        self.verticalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.b_extend = QtWidgets.QPushButton(self.verticalLayoutWidget_3)
        self.b_extend.setObjectName("b_extend")
        self.verticalLayout.addWidget(self.b_extend)
        self.b_retract = QtWidgets.QPushButton(self.verticalLayoutWidget_3)
        self.b_retract.setObjectName("b_retract")
        self.verticalLayout.addWidget(self.b_retract)
        self.horizontalLayout.addLayout(self.verticalLayout)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.l_current1A = QtWidgets.QLabel(self.verticalLayoutWidget_3)
        self.l_current1A.setAlignment(QtCore.Qt.AlignCenter)
        self.l_current1A.setObjectName("l_current1A")
        self.verticalLayout_3.addWidget(self.l_current1A)
        self.s_extend_retract = QtWidgets.QSlider(self.verticalLayoutWidget_3)
        self.s_extend_retract.setMinimum(1)
        self.s_extend_retract.setMaximum(100)
        self.s_extend_retract.setOrientation(QtCore.Qt.Horizontal)
        self.s_extend_retract.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.s_extend_retract.setObjectName("s_extend_retract")
        self.verticalLayout_3.addWidget(self.s_extend_retract)
        self.horizontalLayout.addLayout(self.verticalLayout_3)
        self.verticalLayout_7.addLayout(self.horizontalLayout)
        self.line = QtWidgets.QFrame(self.verticalLayoutWidget_3)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.verticalLayout_7.addWidget(self.line)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.b_rotateCW = QtWidgets.QPushButton(self.verticalLayoutWidget_3)
        self.b_rotateCW.setObjectName("b_rotateCW")
        self.verticalLayout_2.addWidget(self.b_rotateCW)
        self.b_rotateCCW = QtWidgets.QPushButton(self.verticalLayoutWidget_3)
        self.b_rotateCCW.setObjectName("b_rotateCCW")
        self.verticalLayout_2.addWidget(self.b_rotateCCW)
        self.horizontalLayout_2.addLayout(self.verticalLayout_2)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem1)
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.l_current1B = QtWidgets.QLabel(self.verticalLayoutWidget_3)
        self.l_current1B.setAlignment(QtCore.Qt.AlignCenter)
        self.l_current1B.setObjectName("l_current1B")
        self.verticalLayout_4.addWidget(self.l_current1B)
        self.s_rotate = QtWidgets.QSlider(self.verticalLayoutWidget_3)
        self.s_rotate.setMinimum(1)
        self.s_rotate.setMaximum(100)
        self.s_rotate.setOrientation(QtCore.Qt.Horizontal)
        self.s_rotate.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.s_rotate.setObjectName("s_rotate")
        self.verticalLayout_4.addWidget(self.s_rotate)
        self.horizontalLayout_2.addLayout(self.verticalLayout_4)
        self.verticalLayout_7.addLayout(self.horizontalLayout_2)
        self.line_3 = QtWidgets.QFrame(self.verticalLayoutWidget_3)
        self.line_3.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_3.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_3.setObjectName("line_3")
        self.verticalLayout_7.addWidget(self.line_3)
        self.b_drive1_fault = QtWidgets.QPushButton(self.verticalLayoutWidget_3)
        self.b_drive1_fault.setFlat(False)
        self.b_drive1_fault.setObjectName("b_drive1_fault")
        self.verticalLayout_7.addWidget(self.b_drive1_fault)
        self.label_7 = QtWidgets.QLabel(self.groupBox)
        self.label_7.setGeometry(QtCore.QRect(30, 240, 111, 16))
        self.label_7.setObjectName("label_7")
        self.l_tempDrive1 = QtWidgets.QLabel(self.groupBox)
        self.l_tempDrive1.setGeometry(QtCore.QRect(150, 240, 35, 16))
        self.l_tempDrive1.setObjectName("l_tempDrive1")
        self.groupBox_2 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QtCore.QRect(260, 390, 231, 261))
        self.groupBox_2.setObjectName("groupBox_2")
        self.label_15 = QtWidgets.QLabel(self.groupBox_2)
        self.label_15.setGeometry(QtCore.QRect(30, 240, 121, 16))
        self.label_15.setObjectName("label_15")
        self.l_tempDrive2 = QtWidgets.QLabel(self.groupBox_2)
        self.l_tempDrive2.setGeometry(QtCore.QRect(150, 240, 35, 16))
        self.l_tempDrive2.setObjectName("l_tempDrive2")
        self.verticalLayoutWidget_6 = QtWidgets.QWidget(self.groupBox_2)
        self.verticalLayoutWidget_6.setGeometry(QtCore.QRect(20, 20, 201, 211))
        self.verticalLayoutWidget_6.setObjectName("verticalLayoutWidget_6")
        self.verticalLayout_13 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_6)
        self.verticalLayout_13.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_13.setObjectName("verticalLayout_13")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.verticalLayout_14 = QtWidgets.QVBoxLayout()
        self.verticalLayout_14.setObjectName("verticalLayout_14")
        self.b_tilt_positive = QtWidgets.QPushButton(self.verticalLayoutWidget_6)
        self.b_tilt_positive.setObjectName("b_tilt_positive")
        self.verticalLayout_14.addWidget(self.b_tilt_positive)
        self.b_tile_negative = QtWidgets.QPushButton(self.verticalLayoutWidget_6)
        self.b_tile_negative.setObjectName("b_tile_negative")
        self.verticalLayout_14.addWidget(self.b_tile_negative)
        self.horizontalLayout_6.addLayout(self.verticalLayout_14)
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_6.addItem(spacerItem2)
        self.verticalLayout_16 = QtWidgets.QVBoxLayout()
        self.verticalLayout_16.setObjectName("verticalLayout_16")
        self.l_current2B = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        self.l_current2B.setAlignment(QtCore.Qt.AlignCenter)
        self.l_current2B.setObjectName("l_current2B")
        self.verticalLayout_16.addWidget(self.l_current2B)
        self.s_tilt = QtWidgets.QSlider(self.verticalLayoutWidget_6)
        self.s_tilt.setMinimum(1)
        self.s_tilt.setMaximum(100)
        self.s_tilt.setOrientation(QtCore.Qt.Horizontal)
        self.s_tilt.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.s_tilt.setObjectName("s_tilt")
        self.verticalLayout_16.addWidget(self.s_tilt)
        self.horizontalLayout_6.addLayout(self.verticalLayout_16)
        self.verticalLayout_13.addLayout(self.horizontalLayout_6)
        self.line_5 = QtWidgets.QFrame(self.verticalLayoutWidget_6)
        self.line_5.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_5.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_5.setObjectName("line_5")
        self.verticalLayout_13.addWidget(self.line_5)
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.verticalLayout_17 = QtWidgets.QVBoxLayout()
        self.verticalLayout_17.setObjectName("verticalLayout_17")
        self.b_pan_positive = QtWidgets.QPushButton(self.verticalLayoutWidget_6)
        self.b_pan_positive.setObjectName("b_pan_positive")
        self.verticalLayout_17.addWidget(self.b_pan_positive)
        self.b_pan_negative = QtWidgets.QPushButton(self.verticalLayoutWidget_6)
        self.b_pan_negative.setObjectName("b_pan_negative")
        self.verticalLayout_17.addWidget(self.b_pan_negative)
        self.horizontalLayout_8.addLayout(self.verticalLayout_17)
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_8.addItem(spacerItem3)
        self.verticalLayout_18 = QtWidgets.QVBoxLayout()
        self.verticalLayout_18.setObjectName("verticalLayout_18")
        self.l_current2A = QtWidgets.QLabel(self.verticalLayoutWidget_6)
        self.l_current2A.setAlignment(QtCore.Qt.AlignCenter)
        self.l_current2A.setObjectName("l_current2A")
        self.verticalLayout_18.addWidget(self.l_current2A)
        self.s_pan = QtWidgets.QSlider(self.verticalLayoutWidget_6)
        self.s_pan.setMinimum(1)
        self.s_pan.setMaximum(100)
        self.s_pan.setOrientation(QtCore.Qt.Horizontal)
        self.s_pan.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.s_pan.setObjectName("s_pan")
        self.verticalLayout_18.addWidget(self.s_pan)
        self.horizontalLayout_8.addLayout(self.verticalLayout_18)
        self.verticalLayout_13.addLayout(self.horizontalLayout_8)
        self.line_7 = QtWidgets.QFrame(self.verticalLayoutWidget_6)
        self.line_7.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_7.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_7.setObjectName("line_7")
        self.verticalLayout_13.addWidget(self.line_7)
        self.b_drive2_fault = QtWidgets.QPushButton(self.verticalLayoutWidget_6)
        self.b_drive2_fault.setFlat(False)
        self.b_drive2_fault.setObjectName("b_drive2_fault")
        self.verticalLayout_13.addWidget(self.b_drive2_fault)
        self.groupBox_3 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_3.setGeometry(QtCore.QRect(20, 180, 191, 111))
        self.groupBox_3.setObjectName("groupBox_3")
        self.gridLayoutWidget_2 = QtWidgets.QWidget(self.groupBox_3)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(10, 20, 171, 80))
        self.gridLayoutWidget_2.setObjectName("gridLayoutWidget_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.b_wide = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.b_wide.setObjectName("b_wide")
        self.gridLayout_2.addWidget(self.b_wide, 1, 2, 1, 1)
        self.b_manualSelect = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.b_manualSelect.setObjectName("b_manualSelect")
        self.gridLayout_2.addWidget(self.b_manualSelect, 1, 0, 1, 1)
        self.b_far = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.b_far.setObjectName("b_far")
        self.gridLayout_2.addWidget(self.b_far, 1, 1, 1, 1)
        self.b_tele = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.b_tele.setObjectName("b_tele")
        self.gridLayout_2.addWidget(self.b_tele, 0, 2, 1, 1)
        self.b_cameraPower = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.b_cameraPower.setObjectName("b_cameraPower")
        self.gridLayout_2.addWidget(self.b_cameraPower, 0, 0, 1, 1)
        self.b_near = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.b_near.setObjectName("b_near")
        self.gridLayout_2.addWidget(self.b_near, 0, 1, 1, 1)
        self.groupBox_4 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_4.setGeometry(QtCore.QRect(230, 10, 261, 281))
        self.groupBox_4.setObjectName("groupBox_4")
        self.verticalLayoutWidget_5 = QtWidgets.QWidget(self.groupBox_4)
        self.verticalLayoutWidget_5.setGeometry(QtCore.QRect(10, 20, 241, 251))
        self.verticalLayoutWidget_5.setObjectName("verticalLayoutWidget_5")
        self.verticalLayout_15 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_5)
        self.verticalLayout_15.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_15.setObjectName("verticalLayout_15")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.s_ne = QtWidgets.QSlider(self.verticalLayoutWidget_5)
        self.s_ne.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.s_ne.setMinimum(1)
        self.s_ne.setMaximum(1000)
        self.s_ne.setPageStep(100)
        self.s_ne.setSliderPosition(1000)
        self.s_ne.setOrientation(QtCore.Qt.Horizontal)
        self.s_ne.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.s_ne.setObjectName("s_ne")
        self.gridLayout.addWidget(self.s_ne, 1, 1, 1, 1)
        self.s_nw = QtWidgets.QSlider(self.verticalLayoutWidget_5)
        self.s_nw.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.s_nw.setMinimum(1)
        self.s_nw.setMaximum(1000)
        self.s_nw.setPageStep(100)
        self.s_nw.setProperty("value", 1000)
        self.s_nw.setSliderPosition(1000)
        self.s_nw.setOrientation(QtCore.Qt.Horizontal)
        self.s_nw.setInvertedControls(False)
        self.s_nw.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.s_nw.setObjectName("s_nw")
        self.gridLayout.addWidget(self.s_nw, 1, 0, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.verticalLayoutWidget_5)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 0, 1, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.verticalLayoutWidget_5)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 0, 0, 1, 1)
        self.s_sw = QtWidgets.QSlider(self.verticalLayoutWidget_5)
        self.s_sw.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.s_sw.setMinimum(1)
        self.s_sw.setMaximum(1000)
        self.s_sw.setPageStep(100)
        self.s_sw.setSliderPosition(1000)
        self.s_sw.setOrientation(QtCore.Qt.Horizontal)
        self.s_sw.setTickPosition(QtWidgets.QSlider.TicksAbove)
        self.s_sw.setObjectName("s_sw")
        self.gridLayout.addWidget(self.s_sw, 2, 0, 1, 1)
        self.s_se = QtWidgets.QSlider(self.verticalLayoutWidget_5)
        self.s_se.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.s_se.setMinimum(1)
        self.s_se.setMaximum(1000)
        self.s_se.setPageStep(100)
        self.s_se.setSliderPosition(1000)
        self.s_se.setOrientation(QtCore.Qt.Horizontal)
        self.s_se.setTickPosition(QtWidgets.QSlider.TicksAbove)
        self.s_se.setObjectName("s_se")
        self.gridLayout.addWidget(self.s_se, 2, 1, 1, 1)
        self.label = QtWidgets.QLabel(self.verticalLayoutWidget_5)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 3, 0, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.verticalLayoutWidget_5)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 3, 1, 1, 1)
        self.verticalLayout_15.addLayout(self.gridLayout)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem4)
        self.b_enableLEDs = QtWidgets.QPushButton(self.verticalLayoutWidget_5)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.b_enableLEDs.sizePolicy().hasHeightForWidth())
        self.b_enableLEDs.setSizePolicy(sizePolicy)
        self.b_enableLEDs.setObjectName("b_enableLEDs")
        self.horizontalLayout_4.addWidget(self.b_enableLEDs)
        spacerItem5 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem5)
        self.verticalLayout_15.addLayout(self.horizontalLayout_4)
        self.s_max = QtWidgets.QSlider(self.verticalLayoutWidget_5)
        self.s_max.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.s_max.setMinimum(1)
        self.s_max.setMaximum(1000)
        self.s_max.setPageStep(100)
        self.s_max.setOrientation(QtCore.Qt.Horizontal)
        self.s_max.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.s_max.setObjectName("s_max")
        self.verticalLayout_15.addWidget(self.s_max)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.dial = QtWidgets.QDial(self.verticalLayoutWidget_5)
        self.dial.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.dial.setInvertedAppearance(True)
        self.dial.setInvertedControls(True)
        self.dial.setWrapping(True)
        self.dial.setNotchesVisible(False)
        self.dial.setObjectName("dial")
        self.horizontalLayout_3.addWidget(self.dial)
        self.b_setMinBrightnessLED = QtWidgets.QPushButton(self.verticalLayoutWidget_5)
        self.b_setMinBrightnessLED.setObjectName("b_setMinBrightnessLED")
        self.horizontalLayout_3.addWidget(self.b_setMinBrightnessLED)
        spacerItem6 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem6)
        self.b_resetMinimumBrightnessLED = QtWidgets.QPushButton(self.verticalLayoutWidget_5)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.b_resetMinimumBrightnessLED.sizePolicy().hasHeightForWidth())
        self.b_resetMinimumBrightnessLED.setSizePolicy(sizePolicy)
        self.b_resetMinimumBrightnessLED.setObjectName("b_resetMinimumBrightnessLED")
        self.horizontalLayout_3.addWidget(self.b_resetMinimumBrightnessLED)
        self.verticalLayout_15.addLayout(self.horizontalLayout_3)
        self.groupBox_5 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_5.setGeometry(QtCore.QRect(20, 80, 191, 91))
        self.groupBox_5.setObjectName("groupBox_5")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.groupBox_5)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(20, 20, 151, 61))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.s_joystick_speed = QtWidgets.QSlider(self.verticalLayoutWidget)
        self.s_joystick_speed.setMinimum(1)
        self.s_joystick_speed.setMaximum(100)
        self.s_joystick_speed.setSliderPosition(50)
        self.s_joystick_speed.setOrientation(QtCore.Qt.Horizontal)
        self.s_joystick_speed.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.s_joystick_speed.setObjectName("s_joystick_speed")
        self.verticalLayout_6.addWidget(self.s_joystick_speed)
        self.gamepad_connection = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.gamepad_connection.setEnabled(True)
        self.gamepad_connection.setFlat(False)
        self.gamepad_connection.setObjectName("gamepad_connection")
        self.verticalLayout_6.addWidget(self.gamepad_connection)
        self.line_2 = QtWidgets.QFrame(self.centralwidget)
        self.line_2.setGeometry(QtCore.QRect(20, 300, 471, 20))
        self.line_2.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_2.setObjectName("line_2")
        self.groupBox_6 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_6.setGeometry(QtCore.QRect(20, 10, 191, 61))
        self.groupBox_6.setObjectName("groupBox_6")
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.groupBox_6)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(10, 20, 171, 31))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.b_heartBeat = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.b_heartBeat.setObjectName("b_heartBeat")
        self.horizontalLayout_5.addWidget(self.b_heartBeat)
        self.serialNumberSelect3 = QtWidgets.QComboBox(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.serialNumberSelect3.sizePolicy().hasHeightForWidth())
        self.serialNumberSelect3.setSizePolicy(sizePolicy)
        self.serialNumberSelect3.setObjectName("serialNumberSelect3")
        self.horizontalLayout_5.addWidget(self.serialNumberSelect3)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 512, 21))
        self.menubar.setObjectName("menubar")
        self.menuSettings = QtWidgets.QMenu(self.menubar)
        self.menuSettings.setObjectName("menuSettings")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionCamera_Toggle_Flip = QtWidgets.QAction(MainWindow)
        self.actionCamera_Toggle_Flip.setObjectName("actionCamera_Toggle_Flip")
        self.actionOpen_Alert_Current = QtWidgets.QAction(MainWindow)
        self.actionOpen_Alert_Current.setObjectName("actionOpen_Alert_Current")
        self.menuSettings.addAction(self.actionCamera_Toggle_Flip)
        self.menuSettings.addAction(self.actionOpen_Alert_Current)
        self.menubar.addAction(self.menuSettings.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Cobra Commander GUI r0"))
        self.b_enable.setText(_translate("MainWindow", "Enable Motors"))
        self.groupBox.setTitle(_translate("MainWindow", "Drive #1: Shoulder"))
        self.b_extend.setText(_translate("MainWindow", "EXTEND"))
        self.b_retract.setText(_translate("MainWindow", "RETRACT"))
        self.l_current1A.setToolTip(_translate("MainWindow", "Non-Channel Side Motor Current (CH:1A)"))
        self.l_current1A.setText(_translate("MainWindow", "Current (A)"))
        self.s_extend_retract.setToolTip(_translate("MainWindow", "Extend/Retract Velocity Control"))
        self.b_rotateCW.setText(_translate("MainWindow", "ROT CW"))
        self.b_rotateCCW.setText(_translate("MainWindow", "ROT CCW"))
        self.l_current1B.setToolTip(_translate("MainWindow", "Channel Side Motor Current (CH:1B)"))
        self.l_current1B.setText(_translate("MainWindow", "Current (A)"))
        self.s_rotate.setToolTip(_translate("MainWindow", "Rotation Velocity Control"))
        self.b_drive1_fault.setToolTip(_translate("MainWindow", "When fault changes color red"))
        self.b_drive1_fault.setText(_translate("MainWindow", "Drive #1 Fault Status"))
        self.label_7.setText(_translate("MainWindow", "Drive #1 Temp (F):"))
        self.l_tempDrive1.setText(_translate("MainWindow", "F"))
        self.groupBox_2.setTitle(_translate("MainWindow", "Drive #2: Pan/Tilt"))
        self.label_15.setText(_translate("MainWindow", "Drive #2 Temp (F):"))
        self.l_tempDrive2.setText(_translate("MainWindow", "F"))
        self.b_tilt_positive.setText(_translate("MainWindow", "TILT +"))
        self.b_tile_negative.setText(_translate("MainWindow", "TILT -"))
        self.l_current2B.setToolTip(_translate("MainWindow", "Tilt Motor Current (CH:2B)"))
        self.l_current2B.setText(_translate("MainWindow", "Current (A)"))
        self.s_tilt.setToolTip(_translate("MainWindow", "Tilt Velocity Control"))
        self.b_pan_positive.setText(_translate("MainWindow", "PAN +"))
        self.b_pan_negative.setText(_translate("MainWindow", "PAN -"))
        self.l_current2A.setToolTip(_translate("MainWindow", "Pan Motor Current (CH:2A)"))
        self.l_current2A.setText(_translate("MainWindow", "Current (A)"))
        self.s_pan.setToolTip(_translate("MainWindow", "Pan Velocity Control"))
        self.b_drive2_fault.setToolTip(_translate("MainWindow", "When fault changes color red"))
        self.b_drive2_fault.setText(_translate("MainWindow", "Drive #2 Fault Status"))
        self.groupBox_3.setTitle(_translate("MainWindow", "Camera Control"))
        self.b_wide.setText(_translate("MainWindow", "Wide"))
        self.b_manualSelect.setToolTip(_translate("MainWindow", "Toggle between manual/auto focus"))
        self.b_manualSelect.setText(_translate("MainWindow", "M/S"))
        self.b_far.setText(_translate("MainWindow", "Far"))
        self.b_tele.setText(_translate("MainWindow", "Tele"))
        self.b_cameraPower.setToolTip(_translate("MainWindow", "Camera Power"))
        self.b_cameraPower.setText(_translate("MainWindow", "Power"))
        self.b_near.setText(_translate("MainWindow", "Near"))
        self.groupBox_4.setTitle(_translate("MainWindow", "Camera Lighting"))
        self.s_ne.setToolTip(_translate("MainWindow", "NE LED Intensity Slider"))
        self.s_nw.setToolTip(_translate("MainWindow", "NW LED Intensity Slider"))
        self.label_3.setText(_translate("MainWindow", "NE LED"))
        self.label_2.setText(_translate("MainWindow", "NW LED"))
        self.s_sw.setToolTip(_translate("MainWindow", "SW LED Intensity Slider"))
        self.s_se.setToolTip(_translate("MainWindow", "SE LED Intensity Slider"))
        self.label.setText(_translate("MainWindow", "SW LED"))
        self.label_4.setText(_translate("MainWindow", "SE LED"))
        self.b_enableLEDs.setToolTip(_translate("MainWindow", "Enable power to LED\'s"))
        self.b_enableLEDs.setText(_translate("MainWindow", "LED Enable"))
        self.s_max.setToolTip(_translate("MainWindow", "Maximum LED Brightness"))
        self.dial.setToolTip(_translate("MainWindow", "Changes group LED intensity"))
        self.b_setMinBrightnessLED.setToolTip(_translate("MainWindow", "Press this button when all LED\'s are at minimum brightness, use L/R arrow keys to set each LED"))
        self.b_setMinBrightnessLED.setText(_translate("MainWindow", "Set Minimum"))
        self.b_resetMinimumBrightnessLED.setToolTip(_translate("MainWindow", "Reset minimum LED brightness"))
        self.b_resetMinimumBrightnessLED.setText(_translate("MainWindow", "Reset"))
        self.groupBox_5.setTitle(_translate("MainWindow", "Gamepad Control"))
        self.s_joystick_speed.setToolTip(_translate("MainWindow", "Maximum speed for maximum joystick deflection."))
        self.gamepad_connection.setToolTip(_translate("MainWindow", "Gamepad Connection: Green = Connected"))
        self.gamepad_connection.setText(_translate("MainWindow", "Gamepad Connection"))
        self.groupBox_6.setTitle(_translate("MainWindow", "Connection"))
        self.b_heartBeat.setToolTip(_translate("MainWindow", "Flashing green when no connection errors. Click to help identify connected controller."))
        self.b_heartBeat.setText(_translate("MainWindow", "Heart Beat"))
        self.serialNumberSelect3.setToolTip(_translate("MainWindow", "Select controller, if not listed parmeter file is not installed."))
        self.menuSettings.setTitle(_translate("MainWindow", "Settings"))
        self.actionCamera_Toggle_Flip.setText(_translate("MainWindow", "Camera Toggle Flip"))
        self.actionCamera_Toggle_Flip.setStatusTip(_translate("MainWindow", "Automatically rotate the camera video feed by 180 degrees"))
        self.actionCamera_Toggle_Flip.setShortcut(_translate("MainWindow", "Ctrl+F"))
        self.actionOpen_Alert_Current.setText(_translate("MainWindow", "High Current Warning"))
        self.actionOpen_Alert_Current.setStatusTip(_translate("MainWindow", "Set the current in (A) for when the user is alerted for high current."))
        self.actionOpen_Alert_Current.setShortcut(_translate("MainWindow", "Ctrl+W"))
