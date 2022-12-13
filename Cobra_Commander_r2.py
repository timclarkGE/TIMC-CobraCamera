from Cobra_Commander_GUIr2 import Ui_MainWindow
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

DEBUG = False

Net.enableServerDiscovery(PhidgetServerType.PHIDGETSERVER_DEVICEREMOTE)


# Function to check if gamepad stick movement is initial, i.e. last position was zero, new position is not zero
def initial_gamepad_stick_change(last_value, new_value):
    if last_value == 0 and new_value != 0:
        if new_value > 0:
            return 1
        else:
            return -1
    else:
        return 0


# Function to check if gamepad stick movement is final, i.e. last position was not zero, new position is zero
def final_gamepad_stick_change(last_value, new_value):
    if last_value != 0 and new_value == 0:
        if last_value > 0:
            return 1
        else:
            return -1
    else:
        return 0


# Function to check if gamepad stick movement is a speed change, but not from or to zero speed
def gamepad_stick_change(last_value, new_value):
    if last_value != 0 and new_value != 0 and last_value != new_value:
        return round(new_value, 4)
    else:
        # Return zero if possible final move, possible initial move, or no speed change
        return 0


class MyGamepadThread(qtc.QThread):
    # Signals
    gamepad_inactive = qtc.pyqtSignal()
    gamepad_connected = qtc.pyqtSignal()
    # refresh_wait = 0.025  # with a slow computer this value can cause lag with joystick inputs
    refresh_wait = 0.05

    multi_stopped = qtc.pyqtSignal()
    multiple_axes_moved = qtc.pyqtSignal(float, float)
    paned_positive = qtc.pyqtSignal(float)
    paned_negative = qtc.pyqtSignal(float)
    pan_stopped = qtc.pyqtSignal()
    tilted_positive = qtc.pyqtSignal(float)
    tilted_negative = qtc.pyqtSignal(float)
    tilt_stopped = qtc.pyqtSignal()

    # def __init__(self, index_speed_signal, scan_speed_signal, enabled_index, enabled_scan):
    def __init__(self, over_current_signal):
        super(MyGamepadThread, self).__init__()
        self.connected_last = tuple((False, False, False, False))
        self.over_current_sig = over_current_signal
        self.over_current_sig.connect(self.rumble_for_over_current)

        # State variables
        self.s_shoulder_extend_initial = False
        self.s_shoulder_extend_final = False
        self.s_shoulder_retract_initial = False
        self.s_shoulder_retract_final = False
        self.s_shoulder_stationary = True
        self.s_shoulder_speed_changed = False

        self.s_rotation_cw_initial = False
        self.s_rotation_cw_final = False
        self.s_rotation_ccw_initial = False
        self.s_rotation_ccw_final = False
        self.s_rotation_stationary = True
        self.s_rotation_speed_changed = False

        self.s_pan_positive_initial = False
        self.s_pan_positive_final = False
        self.s_pan_negative_initial = False
        self.s_pan_negative_final = False
        self.s_pan_stationary = True
        self.s_pan_speed_changed = False

        self.s_tilt_positive_initial = False
        self.s_tilt_positive_final = False
        self.s_tilt_negative_initial = False
        self.s_tilt_negative_final = False
        self.s_tilt_stationary = True
        self.s_tilt_speed_changed = False

        self.stationary_flag_set = True
        self.pan_stationary_flag_set = True
        self.tilt_stationary_flag_set = True

        self.s_left_button = False
        self.s_last_left_button_click = 0
        self.light_state = 0  # 0: no light mode, 1: light vector, 2: light intensity

        self.btn_state_last = None
        self.thumb_state_last = None
        self.trigger_state_last = None

    def rumble_for_over_current(self):
        print(get_connected().index(True))
        print(set_vibration(get_connected().index(True), 50000, 50000))

    # def update_index_restore_speed(self, speed):
    #     if DEBUG:
    #         print("# INDEX restore speed received")
    #     self.index_restore_speed = speed
    #
    # def update_scan_restore_speed(self, speed):
    #     if DEBUG:
    #         print("# SCAN restore speed received")
    #     self.scan_restore_speed = speed
    #
    # def update_enabled_status_for_index(self, state):
    #     if DEBUG:
    #         print("# Setting Gamepad INDEX to be:", state)
    #     self.is_index_enabled = state
    #
    # def update_enabled_status_for_scan(self, state):
    #     if DEBUG:
    #         print("# Setting Gamepad SCAN to be:", state)
    #     self.is_scan_enabled = state

    def run(self):
        while True:
            time.sleep(self.refresh_wait)
            query_connections = get_connected()
            # This loop won't gather information if there is not exactly one gamepad connected
            if query_connections.count(True) == 1:
                index = query_connections.index(True)
                state = get_state(index)

                btn_state = get_button_values(state)
                thumb_state = get_thumb_values(state)
                trigger_state = get_trigger_values(state)

                # Define the different stick values
                left_stick_x = thumb_state[0][0]
                left_stick_y = thumb_state[0][1]
                right_stick_x = thumb_state[1][0]
                right_stick_y = thumb_state[1][1]
                # Check if this is the first iteration of the loop and pass
                if self.btn_state_last is not None and self.thumb_state_last is not None and self.trigger_state_last is not None:
                    # Define the last stick values
                    left_stick_last_x = self.thumb_state_last[0][0]
                    left_stick_last_y = self.thumb_state_last[0][1]
                    right_stick_last_x = self.thumb_state_last[1][0]
                    right_stick_last_y = self.thumb_state_last[1][1]

                    # Check for initial movement: return 1 if initial movement is positive, return -1 if initial move
                    # is negative
                    x_init_left = initial_gamepad_stick_change(left_stick_last_x, left_stick_x)
                    y_init_left = initial_gamepad_stick_change(left_stick_last_y, left_stick_y)
                    x_init_right = initial_gamepad_stick_change(right_stick_last_x, right_stick_x)
                    y_init_right = initial_gamepad_stick_change(right_stick_last_y, right_stick_y)

                    # Check for final movement: return 1 if final movement was from positive value, return -1 if final
                    # movement was from negative
                    x_final_left = final_gamepad_stick_change(left_stick_last_x, left_stick_x)
                    y_final_left = final_gamepad_stick_change(left_stick_last_y, left_stick_y)
                    x_final_right = final_gamepad_stick_change(right_stick_last_x, right_stick_x)
                    y_final_right = final_gamepad_stick_change(right_stick_last_y, right_stick_y)

                    # Check for: Position Change Not From Zero (pcnfz)
                    x_pcnfz_left = gamepad_stick_change(left_stick_last_x, left_stick_x)
                    y_pcnfz_left = gamepad_stick_change(left_stick_last_y, left_stick_y)
                    x_pcnfz_right = gamepad_stick_change(right_stick_last_x, right_stick_x)
                    y_pcnfz_right = gamepad_stick_change(right_stick_last_y, right_stick_y)

                    # Get state of the left button to determine how the left joystick input should be interpreted
                    self.s_left_button = btn_state["LEFT_THUMB"]

                    # Button was just clicked, record time
                    if self.s_left_button and not self.btn_state_last["LEFT_THUMB"]:
                        self.s_last_left_button_click = time.time()
                    # Button was just released
                    elif not self.s_left_button and self.btn_state_last["LEFT_THUMB"]:
                        # Check if short click which means go back one state, or enter light vector mode from state 0
                        if 0 < (time.time() - self.s_last_left_button_click) <= 1.5:
                            # Check if in light vector mode
                            if not self.light_state:
                                print("Entered Vector Mode")
                                set_vibration(index, 20000, 20000)
                                self.light_state = 1
                                self.deactivate_shoulder_and_rotation_axes()
                            elif self.light_state == 1:
                                print("Exited light mode")
                                set_vibration(index, 0, 0)
                                self.light_state = 0
                            elif self.light_state == 2:
                                print("Exiting light intensity mode to light vector mode")
                                set_vibration(index, 20000, 20000)
                                self.light_state = 1
                        # Long press enters light intensity mode
                        elif 1.5 < (time.time() - self.s_last_left_button_click) < 6:
                            self.light_state = 2
                            print("Entered Light Intensity Mode")
                            set_vibration(index, 30000, 30000)
                            self.deactivate_shoulder_and_rotation_axes()

                    # If no light state active, set state variables for shoulder and rotation axes
                    if not self.light_state:
                        self.process_if_move("shoulder", y_init_left, y_final_left, y_pcnfz_left, left_stick_y)
                        self.process_if_move("rotation", x_init_left, x_final_left, x_pcnfz_left, left_stick_x)

                    # Set the state variables of pan and tilt regardless
                    self.process_if_move("pan", x_init_right, x_final_right, x_pcnfz_right, right_stick_x)
                    self.process_if_move("tilt", y_init_right, y_final_right, y_pcnfz_right, right_stick_y)

                    # If shoulder or rotation not stationary, then prepare to apply new motor commands
                    if not self.s_shoulder_stationary or not self.s_rotation_stationary:
                        self.multi_axis_movement(left_stick_x, left_stick_y)
                        self.stationary_flag_set = False

                    # If shoulder and rotation are stationary, then send the stop command unless it has been previously set
                    if self.s_shoulder_stationary and self.s_rotation_stationary:
                        if not self.stationary_flag_set:
                            self.multi_stopped.emit()
                            self.stationary_flag_set = True

                    # Check state variables for pan axis
                    if self.s_pan_positive_initial:
                        self.paned_positive.emit(right_stick_x)
                    elif self.s_pan_negative_initial:
                        self.paned_negative.emit(abs(right_stick_x))
                    elif self.s_pan_positive_final or self.s_pan_negative_final:
                        self.pan_stopped.emit()
                    elif self.s_pan_speed_changed > 0:
                        self.paned_positive.emit(right_stick_x)
                        self.pan_stationary_flag_set = False
                    elif self.s_pan_speed_changed < 0:
                        self.paned_negative.emit(abs(right_stick_x))
                        self.pan_stationary_flag_set = False

                    if self.s_pan_stationary:
                        if not self.pan_stationary_flag_set:
                            self.pan_stopped.emit()
                            self.pan_stationary_flag_set = True

                    # Check state variables for tilt axis
                    if self.s_tilt_positive_initial:
                        self.tilted_positive.emit(right_stick_y)
                    elif self.s_tilt_negative_initial:
                        self.tilted_negative.emit(abs(right_stick_y))
                    elif self.s_tilt_positive_final or self.s_tilt_negative_final:
                        self.tilt_stopped.emit()
                    elif self.s_tilt_speed_changed > 0:
                        self.tilted_positive.emit(right_stick_y)
                        self.tilt_stationary_flag_set = False
                    elif self.s_tilt_speed_changed < 0:
                        self.tilted_negative.emit(abs(right_stick_y))
                        self.tilt_stationary_flag_set = False

                    if self.s_tilt_stationary:
                        if not self.tilt_stationary_flag_set:
                            self.tilt_stopped.emit()
                            self.tilt_stationary_flag_set = True

                self.thumb_state_last = thumb_state
                self.btn_state_last = btn_state
                self.trigger_state_last = trigger_state

            # Check if gamepad was disconnected, stop movement. Also don't allow for multiple gamepads
            if query_connections != self.connected_last:
                new_count = query_connections.count(True)
                if new_count != 1:
                    self.gamepad_inactive.emit()
                else:
                    self.gamepad_connected.emit()

            self.connected_last = query_connections

            # except:
            #     print("# Problem with Gamepad")

    def deactivate_shoulder_and_rotation_axes(self):
        print("TODO, deactivate shoulder and rotation axes")
        self.s_shoulder_extend_initial = False
        self.s_shoulder_extend_final = False
        self.s_shoulder_retract_initial = False
        self.s_shoulder_retract_final = False
        self.s_shoulder_stationary = True
        self.s_shoulder_speed_changed = False

        self.s_rotation_cw_initial = False
        self.s_rotation_cw_final = False
        self.s_rotation_ccw_initial = False
        self.s_rotation_ccw_final = False
        self.s_rotation_stationary = True
        self.s_rotation_speed_changed = False

    # Process input from initial_gamepad_stick_change() and final_gamepad_stick_change()
    def process_if_move(self, axis, initial_movement, final_movement, pcnfz, current_position):
        if initial_movement:
            # stationary = False
            if initial_movement > 0:
                positive_motion_initial = True
                negative_motion_initial = False
            else:
                positive_motion_initial = False
                negative_motion_initial = True
        else:
            positive_motion_initial = False
            negative_motion_initial = False

        if final_movement:
            # stationary = True
            if final_movement > 0:
                positive_motion_final = True
                negative_motion_final = False
            else:
                positive_motion_final = False
                negative_motion_final = True
        else:
            positive_motion_final = False
            negative_motion_final = False

        # Stationary if not final move, not initial move, current position is zero
        if not final_movement and not initial_movement and current_position == 0:
            stationary = True
        else:
            stationary = False

        if not stationary and pcnfz:
            position_change = pcnfz
        else:
            position_change = 0

        if axis == "shoulder":
            self.s_shoulder_extend_initial = positive_motion_initial
            self.s_shoulder_retract_initial = negative_motion_initial
            self.s_shoulder_extend_final = positive_motion_final
            self.s_shoulder_retract_final = negative_motion_final
            self.s_shoulder_stationary = stationary
            self.s_shoulder_speed_changed = position_change

        elif axis == "rotation":
            self.s_rotation_cw_initial = positive_motion_initial
            self.s_rotation_ccw_initial = negative_motion_initial
            self.s_rotation_cw_final = positive_motion_final
            self.s_rotation_ccw_final = negative_motion_final
            self.s_rotation_stationary = stationary
            self.s_rotation_speed_changed = position_change

        elif axis == "pan":
            self.s_pan_positive_initial = positive_motion_initial
            self.s_pan_negative_initial = negative_motion_initial
            self.s_pan_positive_final = positive_motion_final
            self.s_pan_negative_final = negative_motion_final
            self.s_pan_stationary = stationary
            self.s_pan_speed_changed = position_change

        elif axis == "tilt":
            self.s_tilt_positive_initial = positive_motion_initial
            self.s_tilt_negative_initial = negative_motion_initial
            self.s_tilt_positive_final = positive_motion_final
            self.s_tilt_negative_final = negative_motion_final
            self.s_tilt_stationary = stationary
            self.s_tilt_speed_changed = position_change

    # Input x and y position of joystick. This method is not called if both axes are stationary
    def multi_axis_movement(self, x, y):
        # Check for boundary position movements of joystick
        # When x == 0 the channel side and non-channel motors are opposite. The channel side motor must be positive for extend i.e. y>0
        if x == 0 and y:
            self.multiple_axes_moved.emit(y, -y)
        # When y == 0 the channel side and non-channel motors must be the same. The channel side motor must be negative for CW i.e. x>0
        elif y == 0 and x:
            self.multiple_axes_moved.emit(-x, -x)
        # Below calculates a type of move that involve both extend/retract and CW/CCW
        elif x != 0 or y != 0:
            try:
                unit_vec_mag = round(math.sqrt(x ** 2 + y ** 2), 6)
                mag_1 = unit_vec_mag
                angle = abs(math.degrees(math.asin(y / unit_vec_mag)))
                mag_2 = round(abs((angle - 45) / 45) * unit_vec_mag, 6)
                # Quadrant I
                if x > 0 and y > 0:
                    if angle >= 45:
                        self.multiple_axes_moved.emit(mag_2, -mag_1)
                    elif angle < 45:
                        self.multiple_axes_moved.emit(-mag_2, -mag_1)
                # Quadrant II
                elif x < 0 and y > 0:
                    if angle >= 45:
                        self.multiple_axes_moved.emit(mag_1, -mag_2)
                    elif angle < 45:
                        self.multiple_axes_moved.emit(mag_1, mag_2)
                # Quadrant III
                elif x < 0 and y < 0:
                    if angle >= 45:
                        self.multiple_axes_moved.emit(-mag_2, mag_1)
                    elif angle < 45:
                        self.multiple_axes_moved.emit(mag_2, mag_1)
                # Quadrant IV
                elif 0 < x and y < 0:
                    if angle >= 45:
                        self.multiple_axes_moved.emit(-mag_1, mag_2)
                    elif angle < 45:
                        self.multiple_axes_moved.emit(-mag_1, -mag_2)
            except:
                print("ERROR Shoulder Calculation", x, y, round(math.sqrt(x ** 2 + y ** 2), 6))




    def stop(self):
        if DEBUG:
            print("# Gamepad thread terminated")
        self.exit()
        self.gamepad_inactive.emit()
        # Restore GUI velocity to value before activating gamepad
        self.index_speed_updated.emit(self.index_restore_speed)
        self.scan_speed_updated.emit(self.scan_restore_speed)
        self.connected_last = tuple((False, False, False, False))
        self.btn_state_last = None
        self.thumb_state_last = None
        self.trigger_state_last = None


# Phidget class for opening and saving data about phidgets channels connected
class Phidget(VoltageOutput, VoltageInput, DigitalOutput, DigitalInput, TemperatureSensor):
    num_of_phidgets = 0
    attached_phidgets = 0
    attached_phidgets_mask = 0
    instance_list = []
    data_interval = 200  # ms

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
            # Set the voltage change trigger: 0.004 is 1mA
            # self.setVoltageChangeTrigger(0.002)

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


# Voltage Input Phidgets measure the voltage output from the operational amplifiers to determine the commanded current.
#   At times the reported current is very high because when the amplifiers are not enabled, the op-amps are commanding
#   full positive or negative current. During these times the reported current should be ignored.
class CurrentFeedback(Phidget, qtc.QObject):
    current_updated = qtc.pyqtSignal(float)  # Each time the phidget has a new data point, this signal is emitted.
    trans_con = 0.25  # A/V transconductance for converting voltage to current

    def __init__(self, phidget_type, sn, hub_port, ch):
        super(CurrentFeedback, self).__init__(phidget_type, sn, hub_port, ch, self.onUpdate)
        qtc.QObject.__init__(self)

    def onUpdate(self, trash, voltage):
        current = CurrentFeedback.trans_con * voltage
        # The voltage will be greater than 10V or less than -10V when the amplifier is not enabled and current is 0A
        if -10 > voltage or voltage > 10:
            self.current_updated.emit(0)
        else:
            self.current_updated.emit(current)


# Class for Voltage Output phidget which can control motor speed and LED intensity
class VariableVoltageSource(Phidget):
    def __init__(self, phidget_type, sn, hub_port, ch):
        super(VariableVoltageSource, self).__init__(phidget_type, sn, hub_port, ch)

    def voltageSet(self, value):
        try:
            self.setVoltage(value)
        except PhidgetException as e:
            print("Exception 4, " + str(self.id) + ": " + str(hex(e.code)) + " " + str(self) + " " + str(time.time()))


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


# Class for all four motors used on Cobra Camera
#   Generic Motor P/N: DCX16S GB KL 48V
#   Generic Gearhead P/N: GPX16HP 44:1
#   Private Custom P/N:B7D131BD6396 Revision 1
class Motor(qtc.QObject):
    res_umb = 9.8  # ohms , resistance of umbilical
    res_mot = 84  # ohms, resistance of 48V motor
    cmd_mag = 7.2  # multiplier used convert +-10V command signal to +-V for motor at P5 terminal on LA-310T

    def __init__(self, current: CurrentFeedback, v_cmd: VariableVoltageSource, ch):
        qtc.QObject.__init__(self)
        # Initialize state variables, sv
        self.sv_motor_enabled = False
        self.sv_last_cmd_speed = 0  # units V
        self.sv_last_cmd_voltage = 0  # units V
        self.mtr_current = current
        self.v_cmd = v_cmd
        self.ch = ch
        self.mtr_current.current_updated.connect(self.motor_update)

    def jog_cw(self, speed):
        self.sv_last_cmd_speed = speed
        if self.sv_motor_enabled:
            self.v_cmd.voltageSet(self.sv_last_cmd_voltage)

    # Called from GUI or gamepad
    def jog_ccw(self, speed):
        self.sv_last_cmd_speed = -speed
        if self.sv_motor_enabled:
            self.v_cmd.voltageSet(self.sv_last_cmd_voltage)

    def stop_jog(self):
        self.sv_last_cmd_speed = 0
        self.v_cmd.voltageSet(0)
        print(self.ch, "STOP Issued")

    # Method for IR compensation to achieve constant speed output regardless of load.
    def motor_update(self, current):
        if self.v_cmd.isAttached() and self.mtr_current.isAttached():
            # Calculate voltage drop over motor and umbilical
            v_drop = current * (self.res_umb + self.res_mot)
            # if abs(current) > 0.01:
            #     print(self.ch, round(current,4))
            # Calculate voltage for desired speed. i.e. commanded speed: 3V, calculated voltage drop: 7V, set P5 to 10V
            v_at_p5 = v_drop + self.sv_last_cmd_speed * self.cmd_mag
            # Convert required voltage at P5 to +-10V input command signal
            new_voltage_input = v_at_p5 / self.cmd_mag
            # Set the Voltage Output Phidget to the new voltage. If the command speed is 0V, don't update voltage
            # because the motors will drift
            if -10 < new_voltage_input < 10 and self.sv_last_cmd_speed != 0:
                self.v_cmd.voltageSet(new_voltage_input)
            # Update state variable
            self.sv_last_cmd_voltage = new_voltage_input


class EnableMotors(qtc.QObject):
    def __init__(self, motor1: Motor, motor2: Motor, sw_bias: Phidget, sw_bus: Phidget, sw_enable: Phidget):
        qtc.QObject.__init__(self)
        self.motor1 = motor1
        self.motor2 = motor2  # TIMC fix this naming
        self.sw_bias = sw_bias
        self.sw_bus = sw_bus
        self.sw_enable = sw_enable

    def enable_motor(self):
        self.sw_bias.setState(True)
        time.sleep(1)
        self.sw_bus.setState(True)
        time.sleep(0.25)
        self.sw_enable.setState(True)
        self.motor1.sv_motor_enabled = True
        self.motor2.sv_motor_enabled = True

    def disable_motors(self):
        self.sw_enable.setState(False)
        self.sw_bus.setState(False)
        self.motor1.sv_motor_enabled = False
        self.motor2.sv_motor_enabled = False


# Pull in GUI information and pair with code.
class UserWindow(qtw.QMainWindow, Ui_MainWindow):
    count = 0
    over_current = qtc.pyqtSignal()

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setupUi(self)  # Reads in data from Qt Designer ui file

        # Manage the selection options in the serial number combo box
        self.CM = ConfigurationManager(self.serialNumberSelect)

        # Connect the Phidgets which sense motor current
        self.current_non_channel_1A = CurrentFeedback("VoltageInput", "HUB2", 4, 0)
        self.current_channel_side_1B = CurrentFeedback("VoltageInput", "HUB2", 5, 0)
        self.current_pan_2A = CurrentFeedback("VoltageInput", "HUB2", 3, 0)
        self.current_tilt_2B = CurrentFeedback("VoltageInput", "HUB2", 2, 0)

        # Connect signals for updated motor current to the GUI labels
        # TODO channel and non-channel might need to be flipped
        self.current_non_channel_1A.current_updated.connect(lambda current: self.update_gui_current(current, "1A"))
        self.current_channel_side_1B.current_updated.connect(lambda current: self.update_gui_current(current, "1B"))
        self.current_pan_2A.current_updated.connect(lambda current: self.update_gui_current(current, "2A"))
        self.current_tilt_2B.current_updated.connect(lambda current: self.update_gui_current(current, "2B"))

        # Connect to the Phidgets which voltage signal is used to command motor movement
        self.mtrCmd_non_channel_1A = VariableVoltageSource("VoltageOutput", "HUB0", 2, 0)
        self.mtrCmd_channel_side_1B = VariableVoltageSource("VoltageOutput", "HUB0", 3, 0)
        self.mtrCmd_pan_2A = VariableVoltageSource("VoltageOutput", "HUB2", 1, 0)
        self.mtrCmd_tilt_2B = VariableVoltageSource("VoltageOutput", "HUB2", 0, 0)

        self.gamepad = MyGamepadThread(self.over_current)
        self.gamepad.paned_positive.connect(lambda val: self.jog_pan_positive(round(val * self.s_joystick_speed.value() / 10, 1)))
        self.gamepad.paned_negative.connect(lambda val: self.jog_pan_negative(round(val * self.s_joystick_speed.value() / 10, 1)))
        self.gamepad.pan_stopped.connect(lambda: self.stop_pan_jog())
        self.gamepad.tilted_positive.connect(lambda val: self.jog_tilt_positive(round(val * self.s_joystick_speed.value() / 10, 1)))
        self.gamepad.tilted_negative.connect(lambda val: self.jog_tilt_negative(round(val * self.s_joystick_speed.value() / 10, 1)))
        self.gamepad.tilt_stopped.connect(lambda: self.stop_tilt_jog())

        self.gamepad.multi_stopped.connect(lambda: self.stop_multi_jog())
        self.gamepad.multiple_axes_moved.connect(lambda ch_side, non_ch: self.shoulder_and_rotation_move(ch_side, non_ch))
        self.gamepad.start()

        self.gamepad.gamepad_connected.connect(lambda: print("Gamepad connected"))
        self.gamepad.gamepad_inactive.connect(lambda: print("Gamepad disconnected"))

        # Initialize the motors with their v_cmd and current feedback
        self.motor_non_channel = Motor(self.current_non_channel_1A, self.mtrCmd_non_channel_1A, "1A")
        self.motor_channel_side = Motor(self.current_channel_side_1B, self.mtrCmd_channel_side_1B, "1B")
        self.motor_pan = Motor(self.current_pan_2A, self.mtrCmd_pan_2A, "2A")
        self.motor_tilt = Motor(self.current_tilt_2B, self.mtrCmd_tilt_2B, "2B")

        # Pair GUI buttons with their respective jogging methods using the sliders as speed input
        self.b_tilt_positive.pressed.connect(lambda: self.jog_tilt_positive(self.s_tilt_input.value() / 10))
        self.b_tilt_negative.pressed.connect(lambda: self.jog_tilt_negative(self.s_tilt_input.value() / 10))
        self.b_tilt_positive.released.connect(lambda: self.stop_tilt_jog())
        self.b_tilt_negative.released.connect(lambda: self.stop_tilt_jog())
        self.b_pan_positive.pressed.connect(lambda: self.jog_pan_positive(self.s_pan_input.value() / 10))
        self.b_pan_negative.pressed.connect(lambda: self.jog_pan_negative(self.s_pan_input.value() / 10))
        self.b_pan_positive.released.connect(lambda: self.stop_pan_jog())
        self.b_pan_negative.released.connect(lambda: self.stop_pan_jog())
        self.b_extend.pressed.connect(lambda: self.jog_extend(self.s_extend_retract_input.value() / 10))
        self.b_retract.pressed.connect(lambda: self.jog_retract(self.s_extend_retract_input.value() / 10))
        self.b_extend.released.connect(lambda: self.stop_multi_jog())
        self.b_retract.released.connect(lambda: self.stop_multi_jog())
        self.b_rotateCW.pressed.connect(lambda: self.jog_rotation_cw(self.s_rotate_input.value() / 10))
        self.b_rotateCCW.pressed.connect(lambda: self.jog_rotation_ccw(self.s_rotate_input.value() / 10))
        self.b_rotateCW.released.connect(lambda: self.stop_multi_jog())
        self.b_rotateCCW.released.connect(lambda: self.stop_multi_jog())

        self.swLogic = Phidget("DigitalOutput", "HUB0", 1, 0)
        self.swBus = Phidget("DigitalOutput", "HUB0", 5, 3)
        self.swEnable = Phidget("DigitalOutput", "HUB0", 1, 1)

        self.enable = EnableMotors(self.motor_non_channel, self.motor_channel_side, self.swLogic, self.swBus,
                                   self.swEnable)
        self.b_cameraPower.pressed.connect(lambda: self.enable.enable_motor())
        self.b_near.pressed.connect(lambda: self.enable.disable_motors())
        self.b_wide.pressed.connect(lambda: self.over_current.emit())

    # Method to allow for GUI to open first and then attempt to connect to Phidgets
    def open_phidgets(self):
        self.statusbar.showMessage("Opening Phidget Channels")
        for i in Phidget.instance_list:
            i.open()

    # Method to add a + or - and update the motor current.
    def update_gui_current(self, value, label):
        if label == "1A":
            if value < 0:
                self.l_current1A.setText(" - " + "{:.3f}".format(abs(value)) + " (A)")
            else:
                self.l_current1A.setText(" + " + "{:.3f}".format(value) + " (A)")
        elif label == "1B":
            if value < 0:
                self.l_current1B.setText(" - " + "{:.3f}".format(abs(value)) + " (A)")
            else:
                self.l_current1B.setText(" + " + "{:.3f}".format(value) + " (A)")
        elif label == "2A":
            if value < 0:
                self.l_current2A.setText(" - " + "{:.3f}".format(abs(value)) + " (A)")
            else:
                self.l_current2A.setText(" + " + "{:.3f}".format(value) + " (A)")
        elif label == "2B":
            if value < 0:
                self.l_current2B.setText(" - " + "{:.3f}".format(abs(value)) + " (A)")
            else:
                self.l_current2B.setText(" + " + "{:.3f}".format(value) + " (A)")

    # Methods to start/stop jogging of the TILT axis
    def jog_tilt_positive(self, speed):
        self.motor_tilt.jog_cw(speed)

    def jog_tilt_negative(self, speed):
        self.motor_tilt.jog_ccw(speed)

    def stop_tilt_jog(self):
        self.motor_tilt.stop_jog()

    # Methods to start/stop jogging of the PAN axis
    def jog_pan_positive(self, speed):
        self.motor_pan.jog_ccw(speed)

    def jog_pan_negative(self, speed):
        self.motor_pan.jog_cw(speed)

    def stop_pan_jog(self):
        self.motor_pan.stop_jog()

    # Methods to start/stop jogging of the SHOULDER axis
    def jog_extend(self, speed):
        self.motor_channel_side.jog_cw(speed)
        self.motor_non_channel.jog_ccw(speed)

    def jog_retract(self, speed):
        self.motor_channel_side.jog_ccw(speed)
        self.motor_non_channel.jog_cw(speed)

    def stop_multi_jog(self):
        self.motor_channel_side.stop_jog()
        self.motor_non_channel.stop_jog()

    # Methods to start/stop jogging of the ROTATION axis
    def jog_rotation_cw(self, speed):
        self.motor_channel_side.jog_ccw(speed)
        self.motor_non_channel.jog_ccw(speed)

    def jog_rotation_ccw(self, speed):
        self.motor_channel_side.jog_cw(speed)
        self.motor_non_channel.jog_cw(speed)


    # Takes input from the gamepad where the motors can have different commanded speeds
    def shoulder_and_rotation_move(self, channel_side, non_channel):
        if channel_side < 0:
            self.motor_channel_side.jog_ccw(abs(channel_side) * self.s_joystick_speed.value() / 10)
        elif channel_side > 0:
            self.motor_channel_side.jog_cw(channel_side * self.s_joystick_speed.value() / 10)
        else:
            self.motor_channel_side.stop_jog()

        if non_channel < 0:
            self.motor_non_channel.jog_ccw(abs(non_channel) * self.s_joystick_speed.value() / 10)
        elif non_channel > 0:
            self.motor_non_channel.jog_cw(non_channel * self.s_joystick_speed.value() / 10)
        else:
            self.motor_non_channel.stop_jog()

if __name__ == '__main__':
    app = qtw.QApplication([])
    # Create the main window
    mw = UserWindow()
    mw.show()
    mw.open_phidgets()
    # Create a thread to receive input from the first gamepad (0) plugged into the computer
    # my_handler = GamepadInput(0)
    # my_gamepad_thread = GamepadThread(my_handler)

    # Connect a signal from the gamepad thread to update the status bar in the main window
    # my_handler.message_mode.connect(mw.updateLightMode)
    app.exec_()
