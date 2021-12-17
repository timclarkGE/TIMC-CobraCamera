###############################################################################################
# timothy.clark@ge.com, 12/17/2021
# This script runs the Cobra Commander 006N8537 via Phidgets and two Varedan LA-310T-GE
#
# This script was written with:
#   Phidget22 - Version 1.8 - Built Dec 2 2021 16:16:49, 1.8.20211202
#   Python version 3.9.5
#
# This script is compatible with Cobra Commander Main PCB, 006N7093, revisions 0 and 1.
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


from Cobra_Commander_GUI import Ui_MainWindow
from Dialog_Rumble_Current import Ui_Dialog
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc

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

# Phidget Hub Serial Numbers, initialize with garbage
HG = 999999
HUB1 = HG
HUB2 = HG - 1
HUB3 = HG - 2
HUB4 = HG - 3

Net.enableServerDiscovery(PhidgetServerType.PHIDGETSERVER_DEVICEREMOTE)
# Enable phidget logging
try:
    Log.enable(LogLevel.PHIDGET_LOG_VERBOSE,
               "./Cobra_Camera_Log_Files/" + datetime.datetime.now().strftime('CobraCameraLogFile_%Y-%m-%d_%H-%M.txt'))
except PhidgetException as e:
    os.mkdir("./Cobra_Camera_Log_Files")
    print("Created directory for log files in folder:", os.getcwd())
    Log.enable(LogLevel.PHIDGET_LOG_VERBOSE,
               "./Cobra_Camera_Log_Files/" + datetime.datetime.now().strftime('CobraCameraLogFile_%Y-%m-%d_%H-%M.txt'))


# Class which takes input from the gamepad and calls the functions associate with each button on the GUI
class GamepadInput(EventHandler, qtc.QObject):
    message_mode = qtc.pyqtSignal(str)

    def __init__(self, controller):
        super(GamepadInput, self).__init__(controller)
        qtc.QObject.__init__(self)
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
                    mw.shoulder.multi_axis(event.x * sfactor * 0.5, event.y * sfactor * 0.5)
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

    # Set an instance variable representing phidget channel is attached and update class variable
    def onAttach(self, trash):
        self.attached = 1
        Phidget.attached_phidgets |= (0b1 << self.id)

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
        if self.sn == "HUB1":
            self.setDeviceSerialNumber(HUB1)
        elif self.sn == "HUB2":
            self.setDeviceSerialNumber(HUB2)
        elif self.sn == "HUB3":
            self.setDeviceSerialNumber(HUB3)
        elif self.sn == "HUB4":
            self.setDeviceSerialNumber(HUB4)


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


# Class for measuring motor current
class CurrentFeedback(Phidget):
    trans_con = 0.25  # A/V transconductance
    high_current_warning = 0.2
    num_of_feedback = 0
    instance_list = []

    def __init__(self, phidget_type, sn, hub_port, ch, label: qtw.QLabel):
        super(CurrentFeedback, self).__init__(phidget_type, sn, hub_port, ch, self.onUpdate)
        self.label = label
        self.high_current_state = False
        self.rumble_value = 0
        CurrentFeedback.instance_list.append(self)

    def onUpdate(self, trash, voltage):
        current = CurrentFeedback.trans_con * voltage
        # In all instances of the motors being active, update the text
        if mw.Enable.isActive:
            self.label.setText("{:.3f}".format(abs(current)) + " (A)")
        elif self.label.text() != "N/A (A)":
            self.label.setText("N/A (A)")
            self.label.setStyleSheet("background-color : light grey")
            self.high_current_state = False
            self.rumble_value = 0

        # Check if current is high and alert user with red background
        alert = abs(current) > CurrentFeedback.high_current_warning

        # If there is a high current and the high current state has not been activated
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

        # Rumble if any axis are in the high current state
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
    def __init__(self, led_cmd: VariableVoltageSource, slider: qtw.QSlider):
        self.enabled = False
        self.slider = slider  # values 1-1000
        self.LEDcmd = led_cmd
        self.slider.valueChanged.connect(self.updateLED)
        self.sliderMaximum = self.slider.maximum()
        self.sliderMinimum = self.slider.minimum()

    def updateLED(self):
        if self.LEDcmd.isAttached():
            if self.LEDcmd.getVoltageOutputRange() == VoltageOutputRange.VOLTAGE_OUTPUT_RANGE_10V:
                self.LEDcmd.setVoltageOutputRange(VoltageOutputRange.VOLTAGE_OUTPUT_RANGE_5V)
            self.LEDcmd.voltageSet(self.slider.value() / (self.sliderMaximum / 5.0))

    def setMinimumBrightness(self):
        self.slider.setMaximum(self.slider.value())

    def resetBrightness(self):
        self.slider.setMaximum(self.sliderMaximum)
        self.slider.setMinimum(self.sliderMinimum)

    def setMaximumBrightness(self, value):
        self.slider.setMinimum(value)


# Takes in the four LED's the two slider bars, LED enable, set minimum, and reset buttons
class CameraLighting:
    def __init__(self, led_nw: LightEmittingDiode, led_ne, led_se, led_sw, slider_max: qtw.QSlider,
                 group_dial: qtw.QDial, button_min: qtw.QPushButton, button_reset: qtw.QPushButton,
                 button_enable: qtw.QPushButton, switch: ToggleSwitch):
        self.LEDs = []
        self.LEDs.append(led_nw)
        self.LEDs.append(led_ne)
        self.LEDs.append(led_se)
        self.LEDs.append(led_sw)
        self.maxBrightness = slider_max
        self.GroupIntensity = group_dial
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

        self.enableButton.clicked.connect(self.toggleLEDs)
        self.setMinButton.clicked.connect(self.updateMinimumBrightness)
        self.resetButton.clicked.connect(self.resetLEDs)
        self.GroupIntensity.valueChanged.connect(self.dialUpdateGroupBrightness)
        self.maxBrightness.valueChanged.connect(self.updateGroupMaxBrightness)

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

    def updateMinimumBrightness(self):
        for i in self.LEDs:
            i.setMinimumBrightness()
            i.updateLED()

    def updateGroupMaxBrightness(self):
        for i in self.LEDs:
            i.setMaximumBrightness(self.maxBrightness.value())
            i.updateLED()

    def resetLEDs(self):
        self.maxBrightness.setValue(self.maxBrightness.minimum())
        for i in self.LEDs:
            i.resetBrightness()

    def dialUpdateGroupBrightness(self):
        value = self.GroupIntensity.value()
        # Consider when dial crosses from 0 to 99, or 99 to 0
        if abs(self.lastGroupIntensityValue - value) > 30:
            if value < 30:
                change = (value + 100) - self.lastGroupIntensityValue
            elif value > 60:
                change = value - (self.lastGroupIntensityValue + 100)
            else:
                change = 0
            self.lastGroupIntensityValue = value
        # Standard changes in dial
        else:
            change = value - self.lastGroupIntensityValue
            self.lastGroupIntensityValue = value

        for i in self.LEDs:
            sv = i.slider.value()
            i.slider.setValue(change + sv)

    # Method which is called by a timer to rotate the group dial for brightness based on gamepad input
    def gamepadUpdateGroupBrightness(self):
        if self.setPoint != 0:
            sp = (self.setPoint + self.GroupIntensity.value()) % 99
            self.GroupIntensity.setValue(sp)

    # Method which starts a timer for updating the groupd dial for brightness
    def gamepadProcessSetpoint(self):
        # Start a timer that updates the group brightness dial every 100 ms
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
    def __init__(self, button, switch):
        self.button = button
        self.switch = switch
        self.button.pressed.connect(self.activate)
        self.button.released.connect(self.deactivate)

    def activate(self):
        rv = self.switch.switch_on()
        if rv == 1:
            self.button.setStyleSheet("background-color : green")
        elif rv == 0:
            self.button.setStyleSheet("background-color : light grey")
        else:
            self.button.setStyleSheet("background-color : red")

    def deactivate(self):
        rv = self.switch.switch_off()
        if rv == 1:
            self.button.setStyleSheet("background-color : green")
        elif rv == 0:
            self.button.setStyleSheet("background-color : light grey")
        else:
            self.button.setStyleSheet("background-color : red")


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
        self.linked_gui_button.activate()
        self.buttonActive = True

    def released(self):
        self.linked_gui_button.deactivate()
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
        # Check if manual select is not active
        if not mw.CameraMS.switch.grabState():
            mw.CameraMS.activate()
            time.sleep(0.15)
            mw.CameraMS.deactivate()


# The hub state of the controller is defined by four hub serial numbers
class HubState:
    def __init__(self):
        self.hubs = []
        self.legitimate = False

    def initializeHubs(self, hubs):
        error = 0
        # Only initialize once, check if empty list
        if not self.hubs:
            for i in hubs:
                self.hubs.append(i)
                if len(str(i)) != 6:
                    error += 1
            # Legitimize the state if all SN are 6 digits and there are four total
            if len(hubs) == 4 and not error:
                self.legitimate = True

    def activateHubState(self):
        global HUB1, HUB2, HUB3, HUB4
        if self.legitimate:
            HUB1 = self.hubs[0]
            HUB2 = self.hubs[1]
            HUB3 = self.hubs[2]
            HUB4 = self.hubs[3]
            return True
        else:
            return False

    def getHubs(self):
        if self.legitimate:
            return self.hubs
        else:
            return None


# Contains information about configuration files and corresponding hub state, assumes correct config file passed
class Configuration(HubState):
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
        data = []
        search_text = ["HUB1 = ", "HUB2 = ", "HUB3 = ", "HUB4 = "]
        self.fileObject = open(self.fileName, "r")
        contents = self.fileObject.read()

        # Get hub serial numbers
        for i in range(len(search_text)):
            data.append(int(contents.split(search_text[i])[1].split("\n")[0]))

        # Send data to create HubState
        self.initializeHubs(data)

        # Check if active file was read and activate
        if "YES" in contents:
            if self.activateHubState():
                self.isActive = True

        self.fileObject.close()

    # Check if possible to make configuration active, then modify config file
    def makeConfigurationActive(self):
        if self.legitimate:
            if self.activateHubState():
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
        global HG, HUB1, HUB2, HUB3, HUB4

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
        HUB1 = HG
        HUB2 = HG - 1
        HUB3 = HG - 2
        HUB4 = HG - 3


# Go between for Phidget control, takes in the combo box
class ConfigurationManager:
    def __init__(self, combo_box: qtw.QComboBox):
        self.comboBox = combo_box
        self.configurations = []
        self.configDir = "./Cobra_Camera_Config_Files/"
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
                if "HUB1 = " in contents:
                    if "HUB2 = " in contents:
                        if "HUB3 = " in contents:
                            if "HUB4 = " in contents:
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

        mw.Enable.validateButtonColor()
        mw.LightControl.validateButtonColor()
        mw.CameraPWR.validateButtonColor()


# Popup window with the ability to set the warning motor current threshold
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
        self.swBus = ToggleSwitch("DigitalOutput", "HUB2", 1, 3)
        self.swEnable = ToggleSwitch("DigitalOutput", "HUB2", 0, 1)
        self.swStatusLED = ToggleSwitch("DigitalOutput", "HUB2", 0, 3)
        self.swCameraPWR = ToggleSwitch("DigitalOutput", "HUB2", 1, 1)
        self.swCameraNear = ToggleSwitch("DigitalOutput", "HUB4", 1, 3)
        self.swCameraFar = ToggleSwitch("DigitalOutput", "HUB4", 1, 2)
        self.swCameraWide = ToggleSwitch("DigitalOutput", "HUB4", 1, 1)
        self.swCameraTele = ToggleSwitch("DigitalOutput", "HUB4", 1, 0)
        self.swCameraMS = ToggleSwitch("DigitalOutput", "HUB2", 1, 2)
        self.swEnableLEDs = ToggleSwitch("DigitalOutput", "HUB2", 0, 2)
        self.swLogic = ToggleSwitch("DigitalOutput", "HUB2", 0, 0)

        # Initialize Simple Buttons
        self.CameraPWR = SimpleToggleButton(self.b_cameraPower, self.swCameraPWR)
        self.CameraNear = SimpleButton(self.b_near, self.swCameraNear)
        self.CameraFar = SimpleButton(self.b_far, self.swCameraFar)
        self.CameraWide = SimpleButton(self.b_wide, self.swCameraWide)
        self.CameraTele = SimpleButton(self.b_tele, self.swCameraTele)
        self.CameraMS = SimpleButton(self.b_manualSelect, self.swCameraMS)

        # Initialize LED Brightness Command Voltage (Phidget Channel Qty: 4)

        self.NorthWestLEDcmd = VariableVoltageSource("VoltageOutput", "HUB1", 4, 0)
        self.NorthEastLEDcmd = VariableVoltageSource("VoltageOutput", "HUB1", 1, 0)
        self.SouthEastLEDcmd = VariableVoltageSource("VoltageOutput", "HUB1", 3, 0)
        self.SouthWestLEDcmd = VariableVoltageSource("VoltageOutput", "HUB1", 2, 0)

        # Pair LED with GUI Slider Bar
        self.NorthWestLED = LightEmittingDiode(self.NorthWestLEDcmd, self.s_nw)
        self.NorthEastLED = LightEmittingDiode(self.NorthEastLEDcmd, self.s_ne)
        self.SouthEastLED = LightEmittingDiode(self.SouthEastLEDcmd, self.s_se)
        self.SouthWestLED = LightEmittingDiode(self.SouthWestLEDcmd, self.s_sw)

        # Pair All LED Controls
        self.LightControl = CameraLighting(self.NorthWestLED, self.NorthEastLED, self.SouthEastLED, self.SouthWestLED,
                                           self.s_max, self.dial, self.b_setMinBrightnessLED,
                                           self.b_resetMinimumBrightnessLED, self.b_enableLEDs, self.swEnableLEDs)

        # Initialize Motors Command Voltage (Phidget Channel Qty: 4)
        self.Motor1Acmd = VariableVoltageSource("VoltageOutput", "HUB2", 4, 0)
        self.Motor1Bcmd = VariableVoltageSource("VoltageOutput", "HUB2", 3, 0)
        self.Motor2Acmd = VariableVoltageSource("VoltageOutput", "HUB3", 1, 0)
        self.Motor2Bcmd = VariableVoltageSource("VoltageOutput", "HUB3", 0, 0)
        # self.Motor2Ccmd = VariableVoltageSource("VoltageOutput", HUB2, 2, 0) # Aux motor, ready for future designs

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
        self.TempDrive1 = Thermometer("TemperatureSensor", "HUB4", 2, 1, self.l_tempDrive1)
        self.TempDrive2 = Thermometer("TemperatureSensor", "HUB4", 2, 2, self.l_tempDrive2)
        self.TempDrive1.high_temp_warning.connect(self.temperatureWarning)
        self.TempDrive2.high_temp_warning.connect(self.temperatureWarning)

        # Motor Enable Requires Control: Enable Amplifier, +-96V Bus Voltage,
        self.Enable = EnableMotorsButton(self.b_enable, self.swBus, self.swEnable, self.swLogic)

        # Initialize Voltage Input Devices That Represent Motor Current (Phidget Channel Qty: 4)
        self.vm_1A = CurrentFeedback("VoltageInput", "HUB4", 0, 0, self.l_current1A)
        self.vm_1B = CurrentFeedback("VoltageInput", "HUB3", 5, 0, self.l_current1B)
        self.vm_2A = CurrentFeedback("VoltageInput", "HUB3", 3, 0, self.l_current2A)
        self.vm_2B = CurrentFeedback("VoltageInput", "HUB3", 2, 0, self.l_current2B)

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
