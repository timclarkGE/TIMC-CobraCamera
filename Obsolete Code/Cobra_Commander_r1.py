###############################################################################################
# timothy.clark@ge.com, 11/08/2022
# This script runs the Cobra Commander 006N8537 via Phidgets and two Varedan LA-310T-GE
#
# This script was written with:
#   Phidget22 - Version 1.8 - Built Dec 2 2021 16:16:49, 1.8.20211202
#   Python version 3.9.5
#
# This script is compatible with Cobra Commander Main PCB, 006N7093, only revision 2
#
#   Applicable Documents for Cobra Commander/Camera:
#   -------------------------------------------------------
#       DOC-0014-1035 Cobra Commander Software Files
#       DOC-0014-1037 Cobra Commander Configuration Files
#
#       006N8537 – Top Level Assembly for Cobra Commander
#       006N8577 – Cobra Commander Enclosure
#       006N7093 – Cobra Commander Main PCB
#           Build Instructions: DOC-0014-1080
#       006N8228 – Connection Cables within Cobra Commander
#
#       006N7722 – Cobra Camera PCB (Inside Cobra Camera)
#           Build Instructions: DOC – 0014-0904
#


from Cobra_Commander_GUIr1 import Ui_MainWindow
from Dialog_Rumble_Current import Ui_Dialog
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc
from PyQt5 import QtGui as qtg

from Phidget22.Devices.VoltageOutput import *
from Phidget22.Devices.VoltageInput import *
from Phidget22.Devices.DigitalOutput import *
from Phidget22.Devices.DigitalInput import *
from Phidget22.Devices.TemperatureSensor import *
from Phidget22.Net import *
from Phidget22.Devices.Log import *
from Phidget22.LogLevel import *

from XInput import *
import time
import math
import os
import datetime
import serial

# Phidget Hub Serial Numbers, initialize with garbage hub value
HG = 999999
HUB0 = HG
HUB1 = HG - 1
HUB2 = HG - 2

# LED default minimum brightness settings
LG = 1000
NW = LG
NE = LG
SE = LG
SW = LG

MOTOR_RESISTANCE = 84

Net.enableServerDiscovery(PhidgetServerType.PHIDGETSERVER_DEVICEREMOTE)
# Enable phidget logging
try:
    Log.enable(LogLevel.PHIDGET_LOG_VERBOSE,
               "./Cobra_Commander_Log_Files/" + datetime.datetime.now().strftime(
                   'CobraCommanderLogFile_%Y-%m-%d_%H-%M.txt'))
except PhidgetException as e:
    os.mkdir("./Cobra_Commander_Log_Files")
    print("Created directory for log files in folder:", os.getcwd())
    Log.enable(LogLevel.PHIDGET_LOG_VERBOSE,
               "./Cobra_Commander_Log_Files/" + datetime.datetime.now().strftime(
                   'CobraCommanderLogFile_%Y-%m-%d_%H-%M.txt'))


# Class which takes input from the gamepad and calls the functions associate with each button on the GUI
class GamepadInput(EventHandler, qtc.QObject):
    message_mode = qtc.pyqtSignal(str)

    def __init__(self, controller):
        super(GamepadInput, self).__init__(controller)
        qtc.QObject.__init__(self)
        # TIMC
        self.gpb_wide = GamepadButton("BACK", mw.CameraWide)
        self.gpb_tele = GamepadButton("START", mw.CameraTele)
        self.gpb_far = GamepadButton("RIGHT_SHOULDER", mw.CameraFar)
        self.gpb_near = GamepadButton("LEFT_SHOULDER", mw.CameraNear)
        self.gpb_camera = GamepadCameraButtons(self.gpb_wide, self.gpb_tele, self.gpb_far, self.gpb_near)
        self.controller = controller
        self.lightState = 0  # 0: Light Mode Off, 1: Light Vector Mode, 2: Light Intensity Mode
        self.lightTimer = 0
        self.motionReady = False
        self.rumbleSetting = 0

    def process_button_event(self, event):
        button_dict = get_button_values(get_state(0))

        # Check camera inputs
        if event.button == self.gpb_wide.gamepad_button:
            if button_dict.get(event.button):
                self.gpb_camera.wide_pressed()
            else:
                self.gpb_camera.wide_released()
        elif event.button == self.gpb_tele.gamepad_button:
            if button_dict.get(event.button):
                self.gpb_camera.tele_pressed()
            else:
                self.gpb_camera.tele_released()
        elif event.button == self.gpb_far.gamepad_button:
            if button_dict.get(event.button):
                self.gpb_camera.far_pressed()
            else:
                self.gpb_camera.far_released()
        elif event.button == self.gpb_near.gamepad_button:
            if button_dict.get(event.button):
                self.gpb_camera.near_pressed()
            else:
                self.gpb_camera.near_released()

        # Left Thumb button controls LED state: Light Vector Mode or Light Intensity Mode
        elif event.button == "LEFT_THUMB":
            # If button is pressed, start timer
            if button_dict.get(event.button):
                self.lightTimer = time.time()
                # Disable all prior motions
                self.stop_motion()
            # Apply logic upon release of button
            elif not button_dict.get(event.button):
                # Disable all prior motions if button pressed missed
                self.stop_motion()
                # Check if long click
                if (time.time() - self.lightTimer) > 1.5:
                    # Enter intensity mode from any state if long click
                    if not self.lightState == 2:
                        self.lightState = 2
                        self.rumble(30000)
                        self.message_mode.emit("Light Intensity Mode")
                else:
                    # Enter Light Vector Mode
                    if not self.lightState:
                        self.lightState = 1
                        self.rumble(25000)
                        self.message_mode.emit("Light Vector Mode")
                    # Exit Light Vector Mode
                    elif self.lightState == 1:
                        self.lightState = 0
                        self.rumble(0)
                        self.message_mode.emit("")
                    # Enter Light Vector Mode from Light Intensity Mode
                    elif self.lightState == 2:
                        self.lightState = 1
                        self.rumble(25000)
                        self.message_mode.emit("Light Vector Mode")
                        # Stop updates to light intensity
                        mw.LightControl.setPoint = 0

    def process_stick_event(self, event):
        # Pan and Tilt Joystick == RIGHT
        if self.motionReady:
            sfactor = float(mw.s_joystick_speed.value() / 100)
            if event.stick == RIGHT:
                # Camera Tilt Motion
                if event.y > 0:
                    mw.Motor2B.jogPositive()
                    mw.Motor2B.disableJogNegative()
                    mw.Motor2B.slider.setValue(int(abs(event.y * 100 * sfactor)))
                elif event.y < 0:
                    mw.Motor2B.jogNegative()
                    mw.Motor2B.disableJogPositive()
                    mw.Motor2B.slider.setValue(int(abs(event.y * 100 * sfactor)))
                elif event.y == 0:
                    mw.Motor2B.disableJogPositive()
                    mw.Motor2B.disableJogNegative()
                    mw.Motor2B.slider.setValue(int(abs(event.y * 100 * sfactor)))

                # Camera Pan Motion
                if event.x < 0:
                    mw.Motor2A.jogPositive()
                    mw.Motor2A.disableJogNegative()
                    mw.Motor2A.slider.setValue(int(abs(event.x * 100 * sfactor)))
                    # print(mw.b_pan_positive.isChecked(),mw.b_pan_negative.isChecked()) # These show false
                elif event.x > 0:
                    mw.Motor2A.jogNegative()
                    mw.Motor2A.disableJogPositive()
                    mw.Motor2A.slider.setValue(int(abs(event.x * 100 * sfactor)))
                    # print(mw.b_pan_positive.isChecked(), mw.b_pan_negative.isChecked())
                elif event.x == 0:
                    mw.Motor2A.disableJogPositive()
                    mw.Motor2A.disableJogNegative()
                    mw.Motor2A.slider.setValue(int(abs(event.x * 100 * sfactor)))
                    # print(mw.b_pan_positive.isChecked(), mw.b_pan_negative.isChecked())

            # Left stick controls shoulder in light state 0, light throw in state 1, and max brightness in state 2
            if event.stick == LEFT:
                if not self.lightState:
                    mw.shoulder.multi_axis(event.x * sfactor, event.y * sfactor)
                elif self.lightState == 1:
                    mw.LightControl.gamepadThrowLight(event.x, event.y)
                elif self.lightState == 2:
                    mw.LightControl.setPoint = int(event.y * 20 * -1)

        elif self.check_joysticks_centered():
            self.motionReady = True
            self.message_mode.emit("Gamepad Ready")
        else:
            self.message_mode.emit("Center Gamepad Joysticks to Activate")

    # Trigger input is not used for Cobra Camera
    def process_trigger_event(self, event):
        pass

    def process_connection_event(self, event):
        if event.type == EVENT_CONNECTED:
            mw.gamepad_connection.setStyleSheet("background-color : green")
            self.stop_motion()

            if self.check_joysticks_centered():
                self.motionReady = True
            else:
                self.motionReady = False
                self.message_mode.emit("Gamepad Not Ready, Center Joysticks")

        elif event.type == EVENT_DISCONNECTED:
            mw.gamepad_connection.setStyleSheet("background-color : light grey")
            self.stop_motion()
            self.motionReady = False
            self.message_mode.emit("Gamepad Disconnected")

    def rumble(self, value):
        if value:
            set_vibration(0, value, value)
            self.rumbleSetting = value

        else:
            set_vibration(0, 0, 0)
            self.rumbleSetting = 0

    def stop_motion(self):
        mw.Motor2A.slider.setValue(0)
        mw.Motor2B.slider.setValue(0)
        mw.shoulder.multi_axis(0, 0)

    def check_joysticks_centered(self):
        v = get_thumb_values(get_state(0))
        lx = v[0][0]
        ly = v[0][1]
        rx = v[1][0]
        ry = v[1][1]

        if lx == 0 and ly == 0 and rx == 0 and ry == 0:
            return True
        else:
            return False


# Phidget class for opening and saving data about phidgets channels connected
class Phidget(VoltageOutput, VoltageInput, DigitalOutput, DigitalInput, TemperatureSensor):
    num_of_phidgets = 0
    attached_phidgets = 0
    attached_phidgets_mask = 0
    instance_list = []
    data_interval = 80  # ms
    voltage_change_trigger = 0.01

    def __init__(self, phidget_type, sn, hub_port, ch, change_handler=None):
        if phidget_type == "VoltageOutput":
            VoltageOutput.__init__(self)
            self.setIsHubPortDevice(False)
        elif phidget_type == "VoltageInput":
            VoltageInput.__init__(self)
            self.setIsHubPortDevice(False)
            self.change_handler = change_handler
            self.setOnVoltageChangeHandler(change_handler)
        elif phidget_type == "DigitalOutput":
            DigitalOutput.__init__(self)
            self.setIsHubPortDevice(False)
        elif phidget_type == "DigitalInput":
            DigitalInput.__init__(self)
            self.setIsHubPortDevice(True)
            self.change_handler = change_handler
            self.setOnStateChangeHandler(change_handler)
        elif phidget_type == "TemperatureSensor":
            TemperatureSensor.__init__(self)
            self.setIsHubPortDevice(False)
            self.change_handler = change_handler
            self.setOnTemperatureChangeHandler(change_handler)
        else:
            print("ERROR: Problem creating instance of class")

        Phidget.attached_phidgets_mask += 0b1 << Phidget.num_of_phidgets
        Phidget.num_of_phidgets += 1
        Phidget.instance_list.append(self)

        self.attached = 0
        self.sn = sn
        self.applySerialNumber()
        self.id = Phidget.num_of_phidgets - 1  # The first phidget connected is ID = 0
        self.setHubPort(hub_port)
        self.setChannel(ch)
        self.setOnAttachHandler(self.onAttach)
        self.setOnDetachHandler(self.onDetach)
        self.setOnErrorHandler(self.onError)
        self.phidgetType = phidget_type

    # Set an instance variable representing phidget channel is attached and update class variable
    def onAttach(self, trash):
        self.attached = 1
        Phidget.attached_phidgets |= (0b1 << self.id)
        # Set the data interval and trigger for the phidgets which sense voltage for calculating current
        if self.phidgetType == "VoltageInput":
            self.setDataInterval(Phidget.data_interval)
            self.setVoltageChangeTrigger(Phidget.voltage_change_trigger)

    # Set a instance variable representing phidget channel is not attached and update class variable
    def onDetach(self, trash):
        self.attached = 0
        temp = ~(0b1 << self.id)
        Phidget.attached_phidgets &= temp

    # Respond with state of attachment: 0 not attached, 1 attached. Redundant compared to getAttached()
    def isAttached(self):
        try:
            return self.getAttached()
        except PhidgetException as e:
            print("Exception 0A, " + str(self.id) + ": " + str(hex(e.code)) + " " + str(self) + " " + str(time.time()))
            return -1

    def onError(self, trash, error_code, error_description):
        print("Phidget Error Code:" + str(hex(error_code)))

    def applySerialNumber(self):
        if self.sn == "HUB0":
            self.setDeviceSerialNumber(HUB0)
        elif self.sn == "HUB1":
            self.setDeviceSerialNumber(HUB1)
        elif self.sn == "HUB2":
            self.setDeviceSerialNumber(HUB2)


# Added functionality to Phidget class for toggle and latching of switch
class ToggleSwitch(Phidget):

    def __init__(self, phidget_type, sn, hub_port, ch):
        super(ToggleSwitch, self).__init__(phidget_type, sn, hub_port, ch)

    # Toggle the state of the switch, latch output
    def toggle(self):
        try:
            if not self.attached:
                return -1
            elif self.getState():
                self.setState(False)
                return self.getState()
            elif not self.getState():
                self.setState(True)
                return self.getState()
        except PhidgetException as e:
            print("Exception 0, " + str(self.id) + ": " + str(hex(e.code)) + " " + str(self) + " " + str(time.time()))
            return -1

    def switch_on(self):
        try:
            if not self.attached:
                return -1
            else:
                self.setState(True)
                return self.getState()
        except PhidgetException as e:
            print("Exception 1, " + str(self.id) + ": " + str(hex(e.code)) + " " + str(self) + " " + str(time.time()))
            return -1

    def switch_off(self):
        try:
            if not self.attached:
                return -1
            else:
                self.setState(False)
                return self.getState()
        except PhidgetException as e:
            print("Exception 2, " + str(self.id) + ": " + str(hex(e.code)) + " " + str(self) + " " + str(time.time()))
            return -1

    def grabState(self):
        try:
            return self.getState()
        except PhidgetException as e:
            print("Exception 3, " + str(self.id) + ": " + str(hex(e.code)) + " " + str(self) + " " + str(time.time()))
            return -1


# Class for measuring motor current_tilt
class CurrentFeedback(Phidget, qtc.QObject):
    current_updated = qtc.pyqtSignal()  # TIMC test signal to be deleted later
    trans_con = 0.25  # A/V transconductance for converting voltage to current
    high_current_warning = 0.2
    instance_list = []

    def __init__(self, phidget_type, sn, hub_port, ch, label: qtw.QLabel, slider_power: qtw.QSlider,
                 slider_speed: qtw.QSlider):
        super(CurrentFeedback, self).__init__(phidget_type, sn, hub_port, ch, self.onUpdate)
        qtc.QObject.__init__(self)
        self.label = label
        self.power_slider = slider_power
        self.speed_slider = slider_speed
        self.high_current_state = False
        self.rumble_value = 0
        CurrentFeedback.instance_list.append(self)

    def onUpdate(self, trash, voltage):
        current = abs(CurrentFeedback.trans_con * voltage)
        # In all instances of the motors being active, update the text
        if mw.Enable.isActive:
            self.label.setText("{:.3f}".format(current) + " (A)")
            # Calculate voltage drop over umbilical and motor
            v_drop = current * 9.8 + current * MOTOR_RESISTANCE
            # Calculate the required voltage at P5 to achieve commanded voltage
            new_voltage = v_drop + (self.speed_slider.value() / 10) * 7.2
            # Convert new voltage to new command voltage set point
            set_point = new_voltage / 7.2
            if 10 >= set_point >= -10:
                self.power_slider.setValue(int(abs(set_point) * 10))

            # self.current_updated.emit("{:.3f}".format(abs(current_tilt)) + " (A)", self.description)
            # self.current_updated.emit()
        elif self.label.text() != "N/A (A)":
            self.label.setText("N/A (A)")
            self.label.setStyleSheet("background-color : light grey")
            self.high_current_state = False
            self.rumble_value = 0

        # Check if current_tilt is high and alert user with red background
        alert = current > CurrentFeedback.high_current_warning

        # If there is a high current_tilt and the high current_tilt state has not been activated
        if alert and not self.high_current_state:
            # If the motors are enabled then alert the user
            if mw.Enable.isActive:
                self.label.setStyleSheet("background-color : red")
                self.high_current_state = True
                self.rumble_value = 50000

        # If not alert but high_current_state is set, change state
        elif not alert and self.high_current_state:
            self.label.setStyleSheet("background-color : light grey")
            self.high_current_state = False
            self.rumble_value = 0

        # Rumble if any axis are in the high current_tilt state
        rumble_check = 0
        alert_rumble_freq = 50000
        for i in CurrentFeedback.instance_list:
            if i.high_current_state:
                if self.rumble_value != alert_rumble_freq:
                    my_handler.rumble(alert_rumble_freq)
                    self.rumble_value = alert_rumble_freq
                rumble_check = 1

        if rumble_check == 0:
            if self.rumble_value != 0:
                my_handler.rumble(0)
                self.rumble_value = 0


# Class for Voltage Output phidget which can control motor speed and LED intensity
class VariableVoltageSource(Phidget):
    def __init__(self, phidget_type, sn, hub_port, ch):
        super(VariableVoltageSource, self).__init__(phidget_type, sn, hub_port, ch)

    def voltageSet(self, value):
        try:
            self.setVoltage(value)
        except PhidgetException as e:
            print("Exception 4, " + str(self.id) + ": " + str(hex(e.code)) + " " + str(self) + " " + str(time.time()))


# Class for sensing faults on Varedan
class FaultSense(Phidget, qtc.QObject):

    def __init__(self, phidget_type, sn, hub_port, ch, button: qtw.QPushButton):
        Phidget.__init__(self, phidget_type, sn, hub_port, ch, self.onStateChange)
        qtc.QObject.__init__(self)
        self.button = button
        self.isFault = False

    # state == True when there is a fault
    def onStateChange(self, trash, state):
        if state == 1:
            self.button.setStyleSheet("background-color : red")
            self.isFault = True

        elif state == 0:
            self.button.setStyleSheet("background-color : light grey")
            self.isFault = False


# Class to change temperature label to solid red or blinking red with input from Thermocouple Phidget
class Thermometer(Phidget, qtc.QObject):
    high_temp_warning = qtc.pyqtSignal()

    def __init__(self, phidget_type, sn, hub_port, ch, label: qtw.QLabel):
        Phidget.__init__(self, phidget_type, sn, hub_port, ch, self.onTempChange)
        qtc.QObject.__init__(self)
        self.label = label
        self.highTempFlag = False

    def onTempChange(self, trash, temperature):
        f = (temperature * 9 / 5) + 32
        self.label.setText(str(round(f, 1)))
        if f > 120 and self.highTempFlag == False:
            self.high_temp_warning.emit()
            self.label.setStyleSheet("background-color : red")
            self.highTempFlag = True
        # Flashes the label color red, but also sometimes creates error: "Could not parse stylesheet.... "l_tempDrive2")
        elif f < 212 and self.highTempFlag == True:
            self.highTempFlag = False
            self.label.setStyleSheet("background-color : light grey")


# Combine VoltageOutput Phidget with GUI slider and jog buttons. This is for the Pan and Tilt motor axes.
class IndependentMotor():
    def __init__(self, vvs, slider: qtw.QSlider, jog_pos: qtw.QPushButton, jog_neg: qtw.QPushButton):
        self.buttonJogPositive = jog_pos
        self.buttonJogNegative = jog_neg
        self.vvs = vvs  # Variable voltage source
        self.slider = slider
        self.slider.valueChanged.connect(self.cmdVoltage)
        self.pos_jog_enabled = False
        self.neg_jog_enabled = False
        self.buttonJogPositive.pressed.connect(self.jogPositive)
        self.buttonJogNegative.pressed.connect(self.jogNegative)
        self.buttonJogPositive.released.connect(self.disableJogPositive)
        self.buttonJogNegative.released.connect(self.disableJogNegative)

    # Called everytime the slider is changed or jog button is pressed, checks if jog motion is being commanded
    def cmdVoltage(self, value):
        if self.pos_jog_enabled is True and self.neg_jog_enabled is False:
            self.vvs.voltageSet(value / 10)
        elif self.pos_jog_enabled is False and self.neg_jog_enabled is True:
            self.vvs.voltageSet(- value / 10)
        else:
            self.vvs.voltageSet(0)

    def jogPositive(self):
        self.pos_jog_enabled = True
        self.neg_jog_enabled = False
        self.buttonJogPositive.setStyleSheet("background-color : green")
        self.cmdVoltage(self.slider.value())

    def jogNegative(self):
        self.neg_jog_enabled = True
        self.pos_jog_enabled = False
        self.buttonJogNegative.setStyleSheet("background-color : green")
        self.cmdVoltage(self.slider.value())

    # Sets positive jog flag to False and calls cmdVoltage to stop movement
    def disableJogPositive(self):
        self.pos_jog_enabled = False
        self.buttonJogPositive.setStyleSheet("background-color : light grey")
        self.cmdVoltage(self.slider.value())

    # Sets negative jog flag to False and calls cmdVoltage to stop movement
    def disableJogNegative(self):
        self.neg_jog_enabled = False
        self.buttonJogNegative.setStyleSheet("background-color : light grey")
        self.cmdVoltage(self.slider.value())


# Combine buttons and sliders to control two shoulder motors. One slider controls both motors for extend movement,
# second slider controls shoulder rotational movement.
class Shoulder:
    def __init__(self, extend: qtw.QPushButton, retract: qtw.QPushButton, cw: qtw.QPushButton, ccw: qtw.QPushButton,
                 slider_er: qtw.QSlider, slider_rot: qtw.QSlider, channel_side: VariableVoltageSource,
                 non_channel: VariableVoltageSource):
        self.extendButton = extend
        self.retractButton = retract
        self.cwButton = cw
        self.ccwButton = ccw
        self.sliderExtendRetract = slider_er
        self.sliderRotation = slider_rot
        self.channelSideMtr = channel_side
        self.nonChannelMtr = non_channel
        self.extendButton.pressed.connect(self.extend)
        self.retractButton.pressed.connect(self.retract)
        self.cwButton.pressed.connect(self.rot_cw)
        self.ccwButton.pressed.connect(self.rot_ccw)

        self.extendButton.released.connect(self.stop_motion)
        self.retractButton.released.connect(self.stop_motion)
        self.cwButton.released.connect(self.stop_motion)
        self.ccwButton.released.connect(self.stop_motion)
        self.extendButton.setStyleSheet("QPushButton:pressed{background-color: green}")
        self.retractButton.setStyleSheet("QPushButton:pressed{background-color: green}")
        self.cwButton.setStyleSheet("QPushButton:pressed{background-color: green}")
        self.ccwButton.setStyleSheet("QPushButton:pressed{background-color: green}")
        self.init = False

    def extend(self):
        voltage = self.sliderExtendRetract.value() / 10
        self.nonChannelMtr.voltageSet(-voltage)
        self.channelSideMtr.voltageSet(voltage)

    def retract(self):
        voltage = self.sliderExtendRetract.value() / 10
        self.nonChannelMtr.voltageSet(voltage)
        self.channelSideMtr.voltageSet(-voltage)

    def rot_cw(self):
        voltage = self.sliderRotation.value() / 10
        self.nonChannelMtr.voltageSet(-voltage)
        self.channelSideMtr.voltageSet(-voltage)

    def rot_ccw(self):
        voltage = self.sliderRotation.value() / 10
        self.nonChannelMtr.voltageSet(voltage)
        self.channelSideMtr.voltageSet(voltage)

    def stop_motion(self):
        if self.init:
            self.channelSideMtr.voltageSet(0)
            self.nonChannelMtr.voltageSet(0)
        else:
            self.init = True

    # Method called by GamepadInput which calculates motor speeds for multi-axis movement of extend and rotation of
    # shoulder
    def multi_axis(self, x, y):

        # Check for boundary position movements of joystick
        if x == 0 and y > 0:
            self.sliderExtendRetract.setValue(abs(int(y * 100)))
            self.extend()
        elif x == 0 and y < 0:
            self.sliderExtendRetract.setValue(abs(int(y * 100)))
            self.retract()
        elif y == 0 and x > 0:
            self.sliderRotation.setValue(abs(int(x * 100)))
            self.rot_cw()
        elif y == 0 and x < 0:
            self.sliderRotation.setValue(abs(int(x * 100)))
            self.rot_ccw()
        elif y == 0 and x == 0:
            self.stop_motion()
            self.sliderExtendRetract.setValue(0)
            self.sliderRotation.setValue(0)

        else:
            try:
                unit_vec_mag = round(math.sqrt(x ** 2 + y ** 2), 6)
                mag_1 = unit_vec_mag * 10
                angle = abs(math.degrees(math.asin(y / unit_vec_mag)))
                mag_2 = round(abs((angle - 45) / 45) * unit_vec_mag, 6) * 10
                # Quadrant I
                if x > 0 and y > 0:
                    self.nonChannelMtr.voltageSet(-mag_1)
                    if angle >= 45:
                        self.channelSideMtr.voltageSet(mag_2)
                    elif angle < 45:
                        self.channelSideMtr.voltageSet(-mag_2)
                # Quadrant II
                elif x < 0 and y > 0:
                    self.channelSideMtr.voltageSet(mag_1)
                    if angle >= 45:
                        self.nonChannelMtr.voltageSet(-mag_2)
                    elif angle < 45:
                        self.nonChannelMtr.voltageSet(mag_2)
                # Quadrant III
                elif x < 0 and y < 0:
                    self.nonChannelMtr.voltageSet(mag_1)
                    if angle >= 45:
                        self.channelSideMtr.voltageSet(-mag_2)
                    elif angle < 45:
                        self.channelSideMtr.voltageSet(mag_2)
                # Quadrant IV
                elif 0 < x and y < 0:
                    self.channelSideMtr.voltageSet(-mag_1)
                    if angle >= 45:
                        self.nonChannelMtr.voltageSet(mag_2)
                    elif angle < 45:
                        self.nonChannelMtr.voltageSet(-mag_2)
            except:
                print("ERROR Shoulder Calculation", x, y, round(math.sqrt(x ** 2 + y ** 2), 6))


# Gets slider and VVS to control LED
class LightEmittingDiode:
    def __init__(self, led_cmd: VariableVoltageSource, slider: qtw.QSlider, id):
        self.enabled = False
        self.slider = slider  # values 1-1000
        self.LEDcmd = led_cmd
        self.id = id
        self.slider.valueChanged.connect(self.updateLED)
        self.max_illumination = self.slider.minimum()
        self.sliderMaximum = self.slider.maximum()  # Absolute minimum brightness
        self.sliderMinimum = self.slider.minimum()  # Absolute maximum brightness
        # Set the minimum slider control value for minimum_sp brightness
        self.minimum_sp = self.sliderMaximum

    def updateLED(self):
        if self.LEDcmd.isAttached():
            if self.LEDcmd.getVoltageOutputRange() == VoltageOutputRange.VOLTAGE_OUTPUT_RANGE_10V:
                self.LEDcmd.setVoltageOutputRange(VoltageOutputRange.VOLTAGE_OUTPUT_RANGE_5V)
            self.LEDcmd.voltageSet(self.slider.value() / (self.sliderMaximum / 5.0))

    def setMinimumBrightness(self, value):
        self.slider.setMaximum(value)
        self.minimum_sp = value
        self.updateGlobals()

    def resetBrightness(self):
        self.slider.setMaximum(self.sliderMaximum)
        self.minimum_sp = self.sliderMaximum
        self.updateGlobals()

    def updateGlobals(self):
        global NW, NE, SE, SW
        if self.id == "nw_led":
            NW = self.minimum_sp
        elif self.id == "ne_led":
            NE = self.minimum_sp
        elif self.id == "se_led":
            SE = self.minimum_sp
        elif self.id == "sw_led":
            SW = self.minimum_sp


# Takes in the four LED's the two slider bars, LED enable, set minimum, and reset buttons
class CameraLighting(qtc.QObject):
    updated_minimum_brightness = qtc.pyqtSignal(str, str, str, str)

    def __init__(self, led_nw: LightEmittingDiode, led_ne, led_se, led_sw, intensity_slider: qtw.QSlider,
                 button_min: qtw.QPushButton, button_reset: qtw.QPushButton, button_enable: qtw.QPushButton,
                 switch: ToggleSwitch, edit_nw: qtw.QLineEdit, edit_ne: qtw.QLineEdit, edit_se: qtw.QLineEdit,
                 edit_sw: qtw.QLineEdit, button_apply: qtw.QPushButton):
        qtc.QObject.__init__(self)
        self.LEDs = []
        self.LEDs.append(led_nw)
        self.LEDs.append(led_ne)
        self.LEDs.append(led_se)
        self.LEDs.append(led_sw)
        self.GroupIntensity = intensity_slider
        self.lastGroupIntensityValue = 0
        self.onTimerUpdateGroup = qtc.QTimer()
        self.onTimerUpdateGroup.timeout.connect(
            self.gamepadUpdateGroupBrightness)
        self.onTimerUpdateGroup.start(100)
        self.setPoint = 0  # value that gets changed to be positive or negative, input from gamepad
        self.setMinButton = button_min
        self.resetButton = button_reset
        self.enableButton = button_enable
        self.enableSwitch = switch
        self.edits = []
        self.edits.append(edit_nw)
        self.edits.append(edit_ne)
        self.edits.append(edit_se)
        self.edits.append(edit_sw)

        integers_only = qtg.QIntValidator()
        integers_only.setRange(1, 999)
        for i in range(len(self.edits)):
            self.edits[i].setValidator(integers_only)
        self.applyButton = button_apply

        self.enableButton.clicked.connect(self.toggleLEDs)
        self.setMinButton.clicked.connect(self.setMinimumBrightnessFromSlider)
        self.resetButton.clicked.connect(self.resetLEDs)
        self.GroupIntensity.valueChanged.connect(self.updateGroupIntensity)
        self.applyButton.clicked.connect(self.applyLineEditMinimumBrightness)

    #
    def initLighting(self):
        global NW, NE, SE, SW
        # Populate the edits with the global values determined at startup
        self.edits[0].setText(str(NW))
        self.edits[1].setText(str(NE))
        self.edits[2].setText(str(SE))
        self.edits[3].setText(str(SW))
        self.LEDs[0].slider.setValue(NW)
        self.LEDs[1].slider.setValue(NE)
        self.LEDs[2].slider.setValue(SE)
        self.LEDs[3].slider.setValue(SW)
        self.setMinButton.click()
        self.applyButton.click()

    # Toggle power to LED drivers
    def toggleLEDs(self):
        if self.enableSwitch.isAttached():
            # Set LED brightness before enabling power to LED drivers
            for i in self.LEDs:
                i.updateLED()
            # Change state of switch
            rv = self.enableSwitch.grabState()
            if rv == 0:
                if self.enableSwitch.switch_on() == 1:
                    self.enableButton.setStyleSheet("background-color : green")
            elif rv == 1:
                if self.enableSwitch.switch_off() == 0:
                    self.enableButton.setStyleSheet("background-color : light grey")
            else:
                self.enableButton.setStyleSheet("background-color : red")

        else:
            self.enableButton.setStyleSheet("background-color : red")

    # When user clicks on "Set Minimum" the current_tilt position of the LED control sliders will be used.
    def setMinimumBrightnessFromSlider(self):
        count = 0
        a = []
        for i in self.LEDs:
            value = i.slider.value()
            i.setMinimumBrightness(value)
            self.edits[count].setText(str(value))
            i.updateLED()
            a.append(str(value))
            count += 1
        self.updated_minimum_brightness.emit(a[0], a[1], a[2], a[3])

    # Invoked when user applies values found in edit boxes to set the minimum brightness for LED's
    def applyLineEditMinimumBrightness(self):
        for i in range(len(self.edits)):
            self.LEDs[i].setMinimumBrightness(int(self.edits[i].text()))

    # Update all LED's such that their control input slider reverts back to default values
    def resetLEDs(self):
        for i in self.LEDs:
            i.resetBrightness()

    # Update all LED's to similar percentage brightness
    def updateGroupIntensity(self, value):
        for i in self.LEDs:
            # Find the percentage of group intensity being requested by user
            percentage = value / self.GroupIntensity.maximum()
            # Calculate the range from maximum brightness to setting which corresponds to minimum brightness
            span = i.minimum_sp - i.sliderMinimum
            i.slider.setValue(int(span * percentage))

    # Method which is called by a timer to increase the group brightness based on gamepad input
    def gamepadUpdateGroupBrightness(self):
        if self.setPoint != 0:
            sp = (self.setPoint + self.GroupIntensity.value())
            self.GroupIntensity.setValue(sp)

    # Method which starts a timer for updating the slider for group brightness
    def gamepadProcessSetpoint(self):
        # Start a timer that updates the group brightness every 100 ms
        self.onTimerUpdateGroup.start(100)

    def gamepadThrowLight(self, x, y):
        # Area of light square is 1, 1x1 square
        a = 0.5  # 1/2 length of side
        area = [0, 0, 0, 0]
        res = 200  # Resolution for area calculation (for loop only takes integers)
        dxdy = 0
        max_slider = []
        min_slider = []
        for i in self.LEDs:
            max_slider.append(i.slider.maximum())
            min_slider.append(i.slider.minimum())

        # Calculate the corner vertices: NW, NE, SE, SW
        nwc = [x - a, y + a]
        nec = [x + a, y + a]
        sec = [x + a, y - a]
        swc = [x - a, y - a]

        corners = [nwc, nec, sec, swc]

        # Apply resolution to the vertices of the
        for i in corners:
            i[0] = int(i[0] * res)
            i[1] = int(i[1] * res)

        for y in range(nwc[1], swc[1], -1):
            for x in range(nwc[0], nec[0]):
                if y > 0 < x:
                    area[1] += 1
                elif y >= 0 >= x:
                    area[0] += 1
                elif y < 0 >= x:
                    area[3] += 1
                elif y < 0 < x:
                    area[2] += 1
                dxdy += 1

        # Calc range of LED possible slider values, set LED brightness: 1000: OFF, 1: Full Brightness
        for i in range(len(self.LEDs)):
            delta = max_slider[i] - min_slider[i]
            area[i] = area[i] / dxdy
            sv = max_slider[i] - int(area[i] * delta)
            self.LEDs[i].slider.setValue(sv)

    def validateButtonColor(self):
        rv = self.enableSwitch.grabState()
        if rv == -1:
            self.enableButton.setStyleSheet("background-color : red")
        elif rv == 0:
            self.enableButton.setStyleSheet("background-color : light grey")


# Pairing of a switch with a button on the GUI where button is used to toggle
class SimpleToggleButton:
    def __init__(self, button, switch):
        self.button = button
        self.switch = switch
        self.button.clicked.connect(self.activate)

    # Activate changes the state of the switch and updates the button color
    def activate(self):
        rv = self.switch.toggle()
        if rv == 1:
            self.button.setStyleSheet("background-color : green")
        elif rv == 0:
            self.button.setStyleSheet("background-color : light grey")
        else:
            self.button.setStyleSheet("background-color : red")

    def validateButtonColor(self):
        rv = self.switch.grabState()
        if rv == -1:
            self.button.setStyleSheet("background-color : red")
        elif rv == 0:
            self.button.setStyleSheet("background-color : light grey")


# Pairing of a switch with a button on the GUI where button is used turn something on or turn something off, latching
class SimpleButton:
    s = serial.Serial("COM4", '9600')
    zoom_in = b'\x81\x01\x04\x07\x26\xFF'
    zoom_out = b'\x81\x01\x04\x07\x36\xFF'
    zoom_stop = b'\x81\x01\x04\x07\x00\xFF'
    focus_toggle = b'\x81\x01\x04\x38\x10\xFF'
    focus_far = b'\x81\x01\x04\x08\x02\xFF'
    focus_near = b'\x81\x01\x04\x08\x03\xFF'
    focus_stop = b'\x81\x01\x04\x08\x00\xFF'
    activate_full_auto_exposure = b'\x81\x01\x04\x39\x00\xFF'
    increase_exposure_compensation = b'\x81\x01\x04\x0E\x02\xFF'
    decrease_exposure_compensation = b'\x81\x01\x04\x0E\x03\xFF'
    reset_exposure_compensation = b'\x81\x01\x04\x0E\x00\xFF'

    activate_bright_mode = b'\x81\x01\x04\x39\x0D\xFF'
    bright_up = b'\x81\x01\x04\x0D\x02\xFF'
    bright_down = b'\x81\x01\x04\x0D\x03\xFF'
    reset_bright_mode = b'\x81\x01\x04\x0D\x00\xFF'

    def __init__(self, button, cmd):
        self.button = button
        self.cmd = cmd
        self.button.pressed.connect(self.sendStartCommand)
        self.button.released.connect(self.sendStopCommand)

    def sendStartCommand(self):
        if self.cmd == "in":
            data = self.zoom_in
        elif self.cmd == "out":
            data = self.zoom_out
        elif self.cmd == "near":
            data = self.focus_near
        elif self.cmd == "far":
            data = self.focus_far
        elif self.cmd == "m/s":
            data = self.focus_toggle
        elif self.cmd == "full_auto":
            data = self.activate_full_auto_exposure
        elif self.cmd == "e_comp_inc":
            data = self.increase_exposure_compensation
        elif self.cmd == "e_comp_dec":
            data = self.decrease_exposure_compensation
        elif self.cmd == "e_comp_reset":
            data = self.reset_exposure_compensation
        elif self.cmd == "bright_mode":
            data = self.activate_bright_mode
        elif self.cmd == "bright_inc":
            data = self.bright_up
        elif self.cmd == "bright_dec":
            data = self.bright_down
        elif self.cmd == "bright_reset":
            data = self.reset_bright_mode

        self.s.write(data)

    def sendStopCommand(self):
        if self.cmd == "in" or self.cmd == "out":
            self.s.write(self.zoom_stop)
        elif self.cmd == "near" or self.cmd == "far":
            self.s.write(self.focus_stop)


# Class for button which applies important sequencing of power to Varedan to prevent motor runaway
class EnableMotorsButton:

    def __init__(self, button, sw_bus: ToggleSwitch, sw_amp_e: ToggleSwitch, sw_logic: ToggleSwitch):
        super(EnableMotorsButton, self).__init__()
        self.button = button
        self.sw_bus = sw_bus
        self.sw_amp_e = sw_amp_e
        self.sw_logic = sw_logic
        self.button.clicked.connect(self.activate)
        self.isActive = False
        self.lastActive = 0

    def activate(self):
        if self.isActive is False:
            rv0 = self.sw_logic.switch_on()
            rv1 = self.sw_bus.switch_on()
            rv2 = self.sw_amp_e.switch_on()
            if rv0 == 1 and rv1 == 1 and rv2 == 1:
                self.button.setStyleSheet("background-color : green")
                self.isActive = True
                self.lastActive = time.time()
            # If any of the switches are not connected go into failsafe state
            elif rv0 == -1 or rv1 == -1 or rv2 == -1:
                self.button.setStyleSheet("background-color : red")
                self.enterFailSafe()
            # Logic issue without Phidget connection issue
            else:
                self.button.setStyleSheet("background-color : grey")
                self.enterFailSafe()

        elif self.isActive is True:
            rv0 = self.sw_bus.switch_off()
            rv1 = self.sw_amp_e.switch_off()
            rv2 = self.sw_logic.grabState()
            if rv0 == 0 and rv1 == 0 and rv2 == 1:
                self.button.setStyleSheet("background-color : light grey")
                self.isActive = False
            # If any of the switches are not connected go into failsafe state
            elif rv0 == -1 or rv1 == -1 or rv2 == -1:
                self.button.setStyleSheet("background-color : red")
                self.enterFailSafe()
            # Logic issue without Phidget connection issue
            else:
                self.button.setStyleSheet("background-color : grey")
                self.enterFailSafe()

    # Called when there is a fault, connection issue, or logic issue.
    # Safely shuts down the power switches in the correct order
    def enterFailSafe(self):
        # The power on sequence (logic, bus, enable) creates a temporary "Bus Under Voltage" fault which is ignored
        if time.time() - self.lastActive > 0.1:
            self.sw_bus.switch_off()
            self.sw_amp_e.switch_off()
            self.sw_logic.switch_off()
            self.isActive = False

    def validateButtonColor(self):
        rv0 = self.sw_bus.grabState()
        rv1 = self.sw_amp_e.grabState()
        rv2 = self.sw_logic.grabState()
        if rv0 == -1 or rv1 == -1 or rv2 == -1:
            self.button.setStyleSheet("background-color : red")
            self.enterFailSafe()
        elif rv0 == 0 and rv1 == 0 and rv2 == 0:
            self.button.setStyleSheet("background-color : light grey")


# Pair a button on the game pad with a SimpleButton, no logic regarding if button can be pressed
class GamepadButton:
    # Type hint sting and type of class
    def __init__(self, button_name: str, linked_gui_button: SimpleButton):
        self.gamepad_button = button_name
        self.linked_gui_button = linked_gui_button
        self.buttonActive = False

    def pressed(self):
        self.linked_gui_button.sendStartCommand()
        self.buttonActive = True

    def released(self):
        self.linked_gui_button.sendStopCommand()
        self.buttonActive = False

    def isActive(self):
        return self.buttonActive


# Combines instances of GamepadButtons for camera control.
class GamepadCameraButtons:
    def __init__(self, *buttons: GamepadButton):
        self.wide = buttons[0]
        self.tele = buttons[1]
        self.far = buttons[2]
        self.near = buttons[3]
        self.nearLastPressed = 0.0
        self.farLastPressed = 0.0
        self.toggleWait = 0.5  # Seconds

    # Checks if button press is allowable. i.e. can't zoom and focus at the same time
    def wide_pressed(self):
        if not (self.tele.isActive() or self.far.isActive() or self.near.isActive()):
            self.wide.pressed()
        elif self.tele.isActive():
            mw.CameraPWR.activate()

    def wide_released(self):
        if not (self.tele.isActive() or self.far.isActive() or self.near.isActive()):
            self.wide.released()

    def tele_pressed(self):
        if not (self.wide.isActive() or self.far.isActive() or self.near.isActive()):
            self.tele.pressed()
        elif self.wide.isActive():
            mw.CameraPWR.activate()

    def tele_released(self):
        if not (self.wide.isActive() or self.far.isActive() or self.near.isActive()):
            self.tele.released()

    def far_pressed(self):
        if not (self.wide.isActive() or self.tele.isActive() or self.near.isActive()):
            self.far.pressed()
            sec = time.time()
            # Check for double tap to toggle state of manual select button
            if (sec - self.farLastPressed) < self.toggleWait:
                self.toggle_ms()
            self.farLastPressed = time.time()

    def far_released(self):
        if not (self.wide.isActive() or self.tele.isActive() or self.near.isActive()):
            self.far.released()

    def near_pressed(self):
        if not (self.wide.isActive() or self.tele.isActive() or self.far.isActive()):
            self.near.pressed()
            sec = time.time()
            # Check for double tap to toggle state of manual select button
            if (sec - self.nearLastPressed) < self.toggleWait:
                self.toggle_ms()
            self.nearLastPressed = time.time()

    def near_released(self):
        if not (self.wide.isActive() or self.tele.isActive() or self.far.isActive()):
            self.near.released()

    def toggle_ms(self):
        mw.CameraMS.sendStartCommand()


# The state of the controller as defined by three hub serial numbers and LED values
class State:
    def __init__(self):
        self.hubs = []
        self.leds = []
        self.sn = None
        self.legitimate = False

    def initializeState(self, hubs, leds):
        error = 0
        # Only initialize once, check if empty list
        if not self.hubs:
            for i in hubs:
                self.hubs.append(i)
                if len(str(i)) != 6:
                    error += 1
        if not self.leds:
            for i in leds:
                self.leds.append(i)
                try:
                    value = int(i)
                    if value > 1000 or value < 0:
                        error += 1
                except:
                    # If the casting to int didn't work there was bad input
                    error += 1
            # Legitimize the state if all SN are 6 digits and there are three total
            if len(hubs) == 3 and not error and len(leds) == 4:
                self.legitimate = True

    def activateState(self):
        global HUB0, HUB1, HUB2, NW, NE, SE, SW
        if self.legitimate:
            HUB0 = self.hubs[0]
            HUB1 = self.hubs[1]
            HUB2 = self.hubs[2]
            NW = self.leds[0]
            NE = self.leds[1]
            SE = self.leds[2]
            SW = self.leds[3]

            return True
        else:
            return False


# Contains information about configuration files and corresponding hub state, assumes correct config file passed
class Configuration(State):
    instance_list = []

    def __init__(self, file_name):
        super(Configuration, self).__init__()
        self.fileName = file_name
        self.fileObject = None
        self.isActive = False
        self.readFile()
        Configuration.instance_list.append(self)

    # Read in the file and set global hub variables if config file is active
    def readFile(self):
        data_hubs = []
        data_leds = []
        search_text_hubs = ["HUB0 = ", "HUB1 = ", "HUB2 = "]
        search_text_leds = ["NW = ", "NE = ", "SE = ", "SW = "]
        self.fileObject = open(self.fileName, "r")
        contents = self.fileObject.read()

        # Get hub serial numbers
        for i in range(len(search_text_hubs)):
            data_hubs.append(int(contents.split(search_text_hubs[i])[1].split("\n")[0]))

        # Get LED values
        for i in range(len(search_text_leds)):
            data_leds.append(int(contents.split(search_text_leds[i])[1].split("\n")[0]))

        # Send data to create State
        self.initializeState(data_hubs, data_leds)

        # Check if active file was read and activate
        if "YES" in contents:
            if self.activateState():
                self.isActive = True

        self.fileObject.close()

    # Check if possible to make configuration active, then modify config file
    def makeConfigurationActive(self):
        if self.legitimate:
            if self.activateState():
                self.fileObject = open(self.fileName, "r")
                contents = self.fileObject.read()
                self.fileObject.close()
                if "NO" in contents:
                    self.fileObject = open(self.fileName, "w")
                    contents = contents.replace("NO", "YES")
                    self.fileObject.write(contents)
                    self.fileObject.close()
                self.isActive = True

    def makeConfigurationInactive(self):
        global HG, HUB0, HUB1, HUB2, LG, NW, NE, SE, SW

        self.fileObject = open(self.fileName, "r")
        contents = self.fileObject.read()
        self.fileObject.close()
        if "YES" in contents:
            self.fileObject = open(self.fileName, "w")
            contents = contents.replace("YES", "NO")
            self.fileObject.write(contents)
            self.fileObject.close()
        self.isActive = False

        # Reset global HUB variables
        HUB0 = HG
        HUB1 = HG - 1
        HUB2 = HG - 2
        NW = LG
        NE = LG
        SE = LG
        SW = LG


# Go between for Phidget control, takes in the combo box
class ConfigurationManager:
    def __init__(self, combo_box: qtw.QComboBox):
        self.comboBox = combo_box
        self.configurations = []
        self.configDir = "./Cobra_Commander_Config_Files/"
        self.activeInstanceIndex = None
        self.offlineIndex = None
        self.start()
        self.comboBox.currentIndexChanged.connect(self.changeConfiguration)

    def start(self):
        # Create a configuration for each valid config file
        try:
            config_files = os.listdir(self.configDir)
        except:
            print("Created directory for configuration files in folder:", os.getcwd())
            os.mkdir("./" + self.configDir)
            config_files = os.listdir(self.configDir)
            # Open window for user to drop in config files
            os.startfile(os.getcwd() + "/" + self.configDir)
            msg = qtw.QMessageBox()
            msg.setIcon(qtw.QMessageBox.Information)
            msg.setWindowTitle("Information")
            msg.setText(
                "Copy cobra camera configuration files to the location:\n(see window that just opened)\n\n" + os.getcwd() + "\\" +
                self.configDir.split("./")[1].split("/")[0])
            msg.exec_()

        for i in config_files:
            validated_fn = self.validateFile(i)
            if validated_fn:
                self.configurations.append(Configuration(validated_fn))
                self.comboBox.addItem(i)
        self.comboBox.addItem("OFFLINE")
        self.offlineIndex = self.comboBox.count() - 1

        # Check for an active configuration
        active_instances = []
        for i in self.configurations:
            if i.isActive:
                active_instances.append(i)

        # If no active instances, set in offline mode which is the last index
        if len(active_instances) == 0:
            self.activeInstanceIndex = self.offlineIndex
            self.comboBox.setCurrentIndex(self.activeInstanceIndex)
        # Only one active configuration,
        elif len(active_instances) == 1:
            self.activeInstanceIndex = self.matchInstanceIndex(active_instances[0])
            self.comboBox.setCurrentIndex(self.activeInstanceIndex)
        # If more than one active configuration file make only the first active
        elif len(active_instances) > 1:
            self.activeInstanceIndex = 0
            for i in range(len(active_instances)):
                if i > 0:
                    active_instances[i].makeConfigurationInactive()
            self.comboBox.setCurrentIndex(self.activeInstanceIndex)
            self.configurations[self.activeInstanceIndex].makeConfigurationActive()

    # Returns filename string of valid file, otherwise returns None
    def validateFile(self, fn):
        full_fn = self.configDir + fn
        rv = None
        try:
            f = open(full_fn)
            contents = f.read()
            if ".cc" in full_fn:
                if "HUB0 = " in contents:
                    if "HUB1 = " in contents:
                        if "HUB2 = " in contents:
                            if "NW = " in contents:
                                if "NE = " in contents:
                                    if "SE = " in contents:
                                        if "SW = " in contents:
                                            rv = full_fn
                                            f.close()
        except OSError:
            print("Error opening: " + full_fn)

        return rv

    # Match the active instance to the index value in the combo box
    def matchInstanceIndex(self, instance: Configuration):
        for i in range(len(self.configurations)):
            if self.configurations[i].fileName == instance.fileName:
                return i

    # Called when user changes configuration from the combobox
    def changeConfiguration(self):
        new_index = self.comboBox.currentIndex()
        mw.Enable.enterFailSafe()
        for i in Phidget.instance_list:
            i.close()
        if self.activeInstanceIndex < self.offlineIndex:
            self.configurations[self.activeInstanceIndex].makeConfigurationInactive()
        if new_index != self.offlineIndex:
            self.configurations[new_index].makeConfigurationActive()
        self.activeInstanceIndex = new_index

        for i in Phidget.instance_list:
            i.applySerialNumber()
            i.open()
            print(i.getDeviceSerialNumber())

        # After a configuration change, update the LED settings and validate the button colors
        mw.LightControl.initLighting()
        mw.Enable.validateButtonColor()
        mw.LightControl.validateButtonColor()
        mw.CameraPWR.validateButtonColor()

    # Update the configuration file with new LED settings
    def updateConfigurationForLEDs(self, nw, ne, se, sw):
        search_text = ["NW = ", "NE = ", "SE = ", "SW = "]
        data = []
        # Validate the configuration file if there is an active configuration
        if self.offlineIndex != self.activeInstanceIndex:
            f = open(self.configurations[self.activeInstanceIndex].fileName, "r")
            contents = f.read()
            f.close()
            # Extract current_tilt LED settings
            for i in range(len(search_text)):
                data.append(contents.split(search_text[i])[1].split("\n")[0])

            # Replace LED settings with new data
            contents = contents.replace(search_text[0] + data[0], search_text[0] + nw)
            contents = contents.replace(search_text[1] + data[1], search_text[1] + ne)
            contents = contents.replace(search_text[2] + data[2], search_text[2] + se)
            contents = contents.replace(search_text[3] + data[3], search_text[3] + sw)

            # Write the new data to the configuration file
            f = open(self.configurations[self.activeInstanceIndex].fileName, "w")
            f.write(contents)
            f.close()


# Popup window with the ability to set the warning motor current_tilt threshold
class DialogWindowAlertCurrent(qtw.QDialog, Ui_Dialog):
    max_current = 0.6  # Amps
    submitted = qtc.pyqtSignal(float)

    def __init__(self):
        super(DialogWindowAlertCurrent, self).__init__()
        self.setupUi(self)
        self.s_alertCurrent.valueChanged.connect(self.updateLineEdit)
        self.updateLineEdit()
        self.buttonBox.accepted.connect(self.on_submit)

    def updateLineEdit(self):
        slider = self.s_alertCurrent.value()
        self.e_current.setText(self.convert_current(slider))

    def convert_current(self, value):
        max = self.s_alertCurrent.maximum()
        min = self.s_alertCurrent.minimum()
        c = DialogWindowAlertCurrent.max_current / (max - min)
        rv = round(c * int(value), 2)
        return str(rv)

    def on_submit(self):
        self.submitted.emit(float(self.e_current.text()))


# Pull in GUI information and pair with code.
class UserWindow(qtw.QMainWindow, Ui_MainWindow):
    count = 0

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setupUi(self)  # Reads in data from Qt Designer ui file

        # Manage the selection options in the serial number combo box
        self.CM = ConfigurationManager(self.serialNumberSelect3)

        # Initialize Switches (Phidget Channel Qty: 11)
        self.swBus = ToggleSwitch("DigitalOutput", "HUB0", 5, 3)
        self.swEnable = ToggleSwitch("DigitalOutput", "HUB0", 1, 1)
        self.swStatusLED = ToggleSwitch("DigitalOutput", "HUB0", 1, 3)
        self.swCameraPWR = ToggleSwitch("DigitalOutput", "HUB0", 5, 1)
        self.swEnableLEDs = ToggleSwitch("DigitalOutput", "HUB0", 1, 2)
        self.swLogic = ToggleSwitch("DigitalOutput", "HUB0", 1, 0)

        # Initialize Simple Buttons
        self.CameraPWR = SimpleToggleButton(self.b_cameraPower, self.swCameraPWR)
        self.CameraNear = SimpleButton(self.b_near, "near")
        self.CameraFar = SimpleButton(self.b_far, "far")
        self.CameraWide = SimpleButton(self.b_wide, "out")
        self.CameraTele = SimpleButton(self.b_tele, "in")
        self.CameraMS = SimpleButton(self.b_manualSelect, "m/s")
        self.CameraExpSel = SimpleButton(self.b_activate_full_auto_exposure_mode, "full_auto")
        self.CameraExpInc = SimpleButton(self.b_exp_comp_increase, "e_comp_inc")
        self.CameraExpDec = SimpleButton(self.b_exp_comp_decrease, "e_comp_dec")
        self.CameraExpReset = SimpleButton(self.b_exp_comp_reset, "e_comp_reset")
        self.CameraBrightSel = SimpleButton(self.b_activate_bright_mode, "bright_mode")
        self.CameraBrightInc = SimpleButton(self.b_bright_mode_increase, "bright_inc")
        self.CameraBrightDec = SimpleButton(self.b_bright_mode_decrease, "bright_dec")
        self.CameraBrightReset = SimpleButton(self.b_bright_mode_reset, "bright_reset")

        # Initialize LED Brightness Command Voltage (Phidget Channel Qty: 4)
        self.NorthWestLEDcmd = VariableVoltageSource("VoltageOutput", "HUB1", 4, 0)
        self.NorthEastLEDcmd = VariableVoltageSource("VoltageOutput", "HUB1", 1, 0)
        self.SouthEastLEDcmd = VariableVoltageSource("VoltageOutput", "HUB1", 3, 0)
        self.SouthWestLEDcmd = VariableVoltageSource("VoltageOutput", "HUB1", 2, 0)

        # Pair LED with GUI Slider Bar
        self.NorthWestLED = LightEmittingDiode(self.NorthWestLEDcmd, self.s_nw, "nw_led")
        self.NorthEastLED = LightEmittingDiode(self.NorthEastLEDcmd, self.s_ne, "ne_led")
        self.SouthEastLED = LightEmittingDiode(self.SouthEastLEDcmd, self.s_se, "se_led")
        self.SouthWestLED = LightEmittingDiode(self.SouthWestLEDcmd, self.s_sw, "sw_led")

        # Pair All LED Controls
        self.LightControl = CameraLighting(self.NorthWestLED, self.NorthEastLED, self.SouthEastLED, self.SouthWestLED,
                                           self.s_group_intensity, self.b_setMinBrightnessLED,
                                           self.b_resetMinimumBrightnessLED, self.b_enableLEDs, self.swEnableLEDs,
                                           self.nw_edit, self.ne_edit, self.se_edit, self.sw_edit, self.apply_min_sp)
        self.LightControl.updated_minimum_brightness.connect(self.CM.updateConfigurationForLEDs)
        # Initialize Motors Command Voltage (Phidget Channel Qty: 4)
        self.Motor1Acmd = VariableVoltageSource("VoltageOutput", "HUB0", 2, 0)
        self.Motor1Bcmd = VariableVoltageSource("VoltageOutput", "HUB0", 3, 0)
        self.Motor2Acmd = VariableVoltageSource("VoltageOutput", "HUB2", 1, 0)
        self.Motor2Bcmd = VariableVoltageSource("VoltageOutput", "HUB2", 0, 0)

        # Pair GUI Slider and Buttons with Motor Command Voltage for Independent Movement of Motors
        self.Motor2A = IndependentMotor(self.Motor2Acmd, self.s_pan, self.b_pan_negative, self.b_pan_positive)
        self.Motor2B = IndependentMotor(self.Motor2Bcmd, self.s_tilt, self.b_tilt_positive, self.b_tile_negative)
        self.shoulder = Shoulder(self.b_extend, self.b_retract, self.b_rotateCW, self.b_rotateCCW,
                                 self.s_extend_retract, self.s_rotate, self.Motor1Bcmd, self.Motor1Acmd)

        # Pair Fault Sensing: Drive #1 Shoulder Motors, Drive #2 Pan/Tilt/Aux. Binary Output: Fault/No Fault
        # (Phidget Channel Qty: 2)
        self.Drive1Fault = FaultSense("DigitalInput", "HUB1", 0, 0, self.b_drive1_fault)
        self.Drive2Fault = FaultSense("DigitalInput", "HUB1", 5, 0, self.b_drive2_fault)
        self.b_drive1_fault.pressed.connect(self.resetPressed)
        self.b_drive2_fault.pressed.connect(self.resetPressed)

        # Pair Thermocouples with Label on GUI (Phidget Channel Qty: 2)
        self.TempDrive1 = Thermometer("TemperatureSensor", "HUB0", 4, 1, self.l_tempDrive1)
        self.TempDrive2 = Thermometer("TemperatureSensor", "HUB0", 4, 2, self.l_tempDrive2)
        self.TempDrive1.high_temp_warning.connect(self.temperatureWarning)
        self.TempDrive2.high_temp_warning.connect(self.temperatureWarning)

        # Motor Enable Requires Control: Enable Amplifier, +-96V Bus Voltage,
        self.Enable = EnableMotorsButton(self.b_enable, self.swBus, self.swEnable, self.swLogic)

        # Initialize Voltage Input Devices That Represent Motor Current (Phidget Channel Qty: 4)
        self.vm_1A = CurrentFeedback("VoltageInput", "HUB2", 4, 0, self.l_current1A, self.s_extend_retract,
                                     self.s_extend_retract_input)
        self.vm_1B = CurrentFeedback("VoltageInput", "HUB2", 5, 0, self.l_current1B, self.s_rotate, self.s_rotate_input)
        self.vm_2A = CurrentFeedback("VoltageInput", "HUB2", 3, 0, self.l_current2A, self.s_pan, self.s_pan_input)
        self.vm_2B = CurrentFeedback("VoltageInput", "HUB2", 2, 0, self.l_current2B, self.s_tilt, self.s_tilt_input)
        self.vm_1A.current_updated.connect(self.updateCurrentLabel)  # TIMC to delete later

        # Initialize heartbeat timer to toggle status LED every second. Heartbeat starts when all Phidgets are connected
        self.heartbeat = qtc.QTimer()
        self.heartbeat.timeout.connect(self.beat)
        self.heartbeat.start(1000)
        self.b_heartBeat.pressed.connect(self.myocardial_infarction)
        self.b_heartBeat.released.connect(self.defibrillation)
        self.offline = True

        # Code for Menu Options
        self.alertSettings = DialogWindowAlertCurrent()
        self.actionOpen_Alert_Current.triggered.connect(self.alertSettings.show)
        self.alertSettings.submitted.connect(self.updateCurrentWarning)
        self.actionAdd_Config_Files.triggered.connect(lambda: os.startfile(os.getcwd() + "/" + self.CM.configDir))

        # self.setFocus()
        app.focusChanged.connect(self.rumbleContinuation)
        self.setFocusPolicy(qtc.Qt.StrongFocus)  # This only works when the user clicks on the
        # Load the LED settings
        self.LightControl.initLighting()

    # TIMC this is a test function to be deleted later
    def updateCurrentLabel(self):
        current = 0
        channel = 0
        if channel == "1A":
            self.l_current1A.setText(current)
        elif channel == "1B":
            self.l_current1B.setText(current)
        elif channel == "2A":
            self.l_current2A.setText(current)
        elif channel == "2B":
            self.l_current2B.setText(current)

    @qtc.pyqtSlot(float)
    def updateCurrentWarning(self, value):
        CurrentFeedback.high_current_warning = value

    @qtc.pyqtSlot(str)
    def updateLightMode(self, message):
        self.statusbar.showMessage(message)

    def temperatureWarning(self):
        self.statusbar.showMessage("WARNING: High Internal Temperature, Check Fans!", 1000)

    def beat(self):
        if not ((~Phidget.attached_phidgets) & Phidget.attached_phidgets_mask):
            if self.swStatusLED.isAttached():
                # Toggle and get the returned new LED state
                rv = self.swStatusLED.toggle()
                if rv == 1:
                    self.b_heartBeat.setStyleSheet("background-color : green")
                elif rv == 0:
                    self.b_heartBeat.setStyleSheet("background-color : light grey")
                    UserWindow.count += 1
                else:
                    # Phidget has just disconnected
                    self.b_heartBeat.setStyleSheet("background-color : red")
            else:
                # Phidget is disconnected
                self.b_heartBeat.setStyleSheet("background-color : red")

            if self.offline:
                text = "All " + str(Phidget.num_of_phidgets) + " Phidgets Channels Attached Successfully"
                self.statusbar.showMessage(text, 5000)
                # if self.swLogic.switch_on() == 1 and self.swBus.switch_on() == 1:
                self.b_enable.setEnabled(True)
                self.offline = False

        # All Phidgets are not attached, display error code
        else:
            self.statusbar.showMessage("Phidget Error Code (L->F): " + format(Phidget.attached_phidgets,
                                                                              "0" + str(Phidget.num_of_phidgets) + "b"))
            if not self.offline:
                self.b_heartBeat.setStyleSheet("background-color : red")
                # Check Motor
                self.Enable.validateButtonColor()
                # Check Camera Power
                self.CameraPWR.validateButtonColor()
                # Check LED Power
                self.LightControl.validateButtonColor()
            self.offline = True

    # Called when user presses and holds the heartbeat button to identify Cobra Commander
    def myocardial_infarction(self):
        self.b_heartBeat.setStyleSheet("background-color : green")
        self.heartbeat.stop()
        self.swStatusLED.switch_on()

    # Called when user releases heartbeat button
    def defibrillation(self):
        self.b_heartBeat.setStyleSheet("background-color : light grey")
        self.heartbeat.start(1000)

    # When GUI is closed, safely shutdown the hardware.
    def closeEvent(self, event):
        self.swEnable.switch_off()
        self.swBus.switch_off()
        time.sleep(1)
        self.swLogic.switch_off()
        self.close_phidgets()

    # Method to allow for GUI to open first and then attempt to connecto to Phidgets
    def open_phidgets(self):
        self.statusbar.showMessage("Opening Phidget Channels")
        for i in Phidget.instance_list:
            i.open()

    def close_phidgets(self):
        self.statusbar.showMessage("Closing Phidget Channels")
        for i in Phidget.instance_list:
            i.close()

    # Method to reset Varedan fault
    def resetPressed(self):
        if self.Drive1Fault.isFault or self.Drive2Fault.isFault:
            self.swEnable.switch_off()
            self.swBus.switch_off()
            self.swLogic.switch_off()
            wait_for_reset = qtc.QTimer()
            wait_for_reset.singleShot(1000, self.Enable.activate)
        else:
            self.statusbar.showMessage("No Fault Detected", 1000)

    # When focus has changed away from the GUI rumble stops but the Gamepad input still works. When focus is returned
    # to the window this method starts the rumble again.
    def rumbleContinuation(self):
        my_handler.rumble(my_handler.rumbleSetting)


if __name__ == '__main__':
    app = qtw.QApplication([])
    # Create the main window
    mw = UserWindow()
    mw.show()
    mw.open_phidgets()
    # Create a thread to receive input from the first gamepad (0) plugged into the computer
    my_handler = GamepadInput(0)
    my_gamepad_thread = GamepadThread(my_handler)

    # Connect a signal from the gamepad thread to update the status bar in the main window
    my_handler.message_mode.connect(mw.updateLightMode)
    app.exec_()
