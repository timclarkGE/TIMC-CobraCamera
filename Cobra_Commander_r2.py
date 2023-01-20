from Cobra_Commander_GUIr2 import Ui_MainWindow
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

import webbrowser

import xml.etree.ElementTree as ET
import sys
import win32pipe, win32file, pywintypes

from XInput import *
import time
import math
import os
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


# Class to contain methods for serial control of the camera
class CameraControl(qtc.QObject):
    # USB to Serial Converter used to send commands to camera P/N: TTL232RG-VSW5V0
    TTL232RG_VendorID = str(0x0403)
    TTL232RG_ProductID = str(0x6001)
    wclient = r'\\.\pipe\vhclient' if sys.platform == 'win32' else '/tmp/vhclient'

    serial_connected = qtc.pyqtSignal(bool)
    message_created = qtc.pyqtSignal(str)
    conflicted_users = qtc.pyqtSignal(bool)

    cmd_packets = {
        "zoom_tele": b'\x81\x01\x04\x07\x26\xFF',
        "zoom_wide": b'\x81\x01\x04\x07\x36\xFF',
        "zoom_stop": b'\x81\x01\x04\x07\x00\xFF',
        "focus_near": b'\x81\x01\x04\x08\x03\xFF',
        "focus_far": b'\x81\x01\x04\x08\x02\xFF',
        "focus_stop": b'\x81\x01\x04\x08\x00\xFF',
        "focus_auto": b'\x81\x01\x04\x38\x02\xFF',
        "focus_manual": b'\x81\x01\x04\x38\x03\xFF',
        "full_auto": [b'\x81\x01\x04\x39\x00\xFF', b'\x81\x01\x04\x3E\x02\xFF'],
        "inc_exp_comp": b'\x81\x01\x04\x0E\x02\xFF',
        "dec_exp_comp": b'\x81\x01\x04\x0E\x03\xFF',
        "exp_reset": b'\x81\x01\x04\x0E\x00\xFF',
        "bright_mode": b'\x81\x01\x04\x39\x0D\xFF',
        "bright_up": b'\x81\x01\x04\x0D\x02\xFF',
        "bright_down": b'\x81\x01\x04\x0D\x03\xFF',
        "bright_reset": b'\x81\x01\x04\x0D\x00\xFF',
        "reset_cam": b'\x81\x01\x04\x19\x03\xFF',
    }

    def __init__(self):
        super(CameraControl, self).__init__()
        self.serial_port = serial.Serial()
        self.serial_port.baudrate = 9600
        self.serial_port.timeout = 1
        self.port_name = "N/A"
        self.status_check_timer = qtc.QTimer()
        self.status_check_timer.timeout.connect(self.check_serial_status)
        self.status_check_timer.start(2000)
        self.last_check = False
        self.conn_address = None

    # Virtual Here based method to start using a USB device connected to virtual here server
    def useDevice(self, address):
        devices = self.writeAndReadServer("USE," + address)
        return devices

    # Virtual Here based method to query the network for visible Virtual Here servers
    def getClientState(self):
        state = self.writeAndReadServer("get client state")
        return state

    # Shutdown the connection with a USB device
    def stopDevice(self, address):
        state = self.writeAndReadServer("STOP USING," + address)
        return state

    # Main API call for Virtual Here which executes text based commands
    def writeAndReadServer(self, IPCcommand):
        IPCcommand = IPCcommand.encode() + b'\n'
        try:
            handle = win32file.CreateFile(self.wclient, win32file.GENERIC_READ | win32file.GENERIC_WRITE, 0, None, win32file.OPEN_EXISTING,
                                          win32file.FILE_ATTRIBUTE_NORMAL, None)
            res = win32pipe.SetNamedPipeHandleState(handle, win32pipe.PIPE_READMODE_MESSAGE, None, None)
            exitcode, buf = win32pipe.TransactNamedPipe(handle, IPCcommand, 15000, None)
            buf = buf.decode()

            return buf, exitcode
        except:
            self.message_created.emit("Virtual Here is not running on motion controller laptop")
            return False

    # Slot: Send commands to camera through the serial connection
    def send_serial_cmd(self, cmd):
        # Some actions require multiple serial commands that are in the form of a list
        if self.serial_port.is_open:
            try:
                if isinstance(self.cmd_packets[cmd], list):
                    for i in range(len(self.cmd_packets[cmd])):
                        self.serial_port.write(self.cmd_packets[cmd][i])
                # Single serial command is adequate for camera action
                else:
                    self.serial_port.write(self.cmd_packets[cmd])
            except serial.SerialException:
                pass

    def report_serial_error(self):
        self.message_created.emit("Error with serial port: " + self.port_name)
        self.serial_connected.emit(False)
        try:
            self.serial_port.close()
        except serial.SerialException:
            pass

    # Slot: Starts a serial connection with an ip address string as input
    def start_serial(self, ip):
        state = self.getClientState()
        # State will be False if virtual here is not running
        if state:
            root = ET.fromstring(state[0])
            usb_host = "0.0.0.0."
            serial_id = "N/A"

            # Attempt to close down any prior ports that were opened
            try:
                self.serial_port.close()
            except:
                pass

            # If there are no servers to check, exit the method
            if len(root) == 0:
                self.message_created.emit("No Virtual Here Servers Found -- Check Network")
                return
            else:
                connections = root.findall("./server/connection")
                for child in connections:
                    if child.get("ip") == ip:
                        serial_id = child.get("serverSerial")
                        usb_host = child.get("hostname")
                devices = root.findall("./server/device")
                for child in devices:
                    if child.get("serverSerial") == serial_id:
                        address = child.get("address")
                        id_vendor = child.get("idVendor")
                        id_product = child.get("idProduct")
                        device_state = child.get("state")

                        # Check if the properties of the serial device connected matches the USB to Serial properties
                        if id_vendor == self.TTL232RG_VendorID and id_product == self.TTL232RG_ProductID:
                            self.conn_address = usb_host + "." + address
                            # Device is ready for binding: state 1
                            if device_state == '1':
                                result = self.useDevice(self.conn_address)
                                if result[0] != "OK":
                                    self.message_created.emit("Error when Virtual Here called: USE")
                                else:
                                    self.message_created.emit("Initializing serial connection for camera control")
                                    time.sleep(3)  # Client software needs time for initialization
                                    state = self.getClientState()
                                    root = ET.fromstring(state[0])
                                    if len(root) == 0:
                                        self.message_created.emit("No Virtual Here Servers Found -- Check Network")
                                        return
                                    else:
                                        devices = root.findall("./server/device")
                                        for child in devices:
                                            # Check if the device now reports state 3 which denotes a successful attempt to connect
                                            if child.get("serverSerial") == serial_id and child.get("state") == '3':
                                                self.port_name = child.get("comPortName")
                                                self.message_created.emit("Virtual Here connected to serial adapter at: " + self.port_name)
                        # Device state 3 is for connection already existing
                        if device_state == '3':
                            self.port_name = child.get("comPortName")
                            # Sometimes the port will be in use by another user on another computer and the port name will be nothing
                            if self.port_name:
                                self.message_created.emit("Virtual Here already connected to port: " + self.port_name)

            # Give the user time to read status messages
            time.sleep(2)
            try:
                self.serial_port.port = self.port_name
                self.serial_port.open()
                if self.serial_port.is_open:
                    self.message_created.emit("Successful serial connection with camera on port: " + self.port_name)
            except serial.SerialException:
                if self.port_name:
                    self.message_created.emit("Error attempting to use serial port: " + self.port_name)
                else:
                    self.message_created.emit("This Cobra Commander is already in use")
                    self.conflicted_users.emit(True)

            # Emit true or false if the serial is connected
            self.serial_connected.emit(self.serial_port.is_open)

        # Close the serial connection to the camera

    def stop_serial(self):
        self.port_name = "N/A"
        try:
            self.serial_port.close()
            self.message_created.emit("Closed serial port: " + self.port_name)
            self.stopDevice(self.conn_address)
            self.conn_address = None
        except:
            pass

    # Check serial connection and emit signal if state change
    def check_serial_status(self):
        state_check = False
        is_open = self.serial_port.is_open
        try:
            self.serial_port.inWaiting()
            can_write = True
        except serial.SerialException:
            can_write = False

        if is_open and can_write:
            state_check = True

        if state_check != self.last_check:
            if state_check:
                self.serial_connected.emit(True)
            else:
                self.report_serial_error()

        self.last_check = state_check


# Worker function to facilitate using the gamepad to control the motor motion as well as camera functions such as zoom.
class GamepadControls(qtc.QObject):
    # Signals
    gamepad_inactive = qtc.pyqtSignal()
    gamepad_connected = qtc.pyqtSignal()
    refresh_wait = 50  # ms of time between polling of gamepad state
    double_click_time = 0.5  # s of time between consecutive clicks of focus far or focus near which toggles auto and manual focus

    multi_stopped = qtc.pyqtSignal()
    multiple_axes_moved = qtc.pyqtSignal(float, float)
    paned_positive = qtc.pyqtSignal(float)
    paned_negative = qtc.pyqtSignal(float)
    pan_stopped = qtc.pyqtSignal()
    tilted_positive = qtc.pyqtSignal(float)
    tilted_negative = qtc.pyqtSignal(float)
    tilt_stopped = qtc.pyqtSignal()
    light_state_updated = qtc.pyqtSignal(list)  # NW, NE, SE, SW
    intensity_state_updated = qtc.pyqtSignal(float)

    zoomed_tele = qtc.pyqtSignal()
    zoomed_wide = qtc.pyqtSignal()
    zoom_stopped = qtc.pyqtSignal()
    focused_near = qtc.pyqtSignal()
    focused_far = qtc.pyqtSignal()
    focus_stopped = qtc.pyqtSignal()
    focus_state_updated = qtc.pyqtSignal()
    camera_power_state_updated = qtc.pyqtSignal()

    message_created = qtc.pyqtSignal(str)

    # def __init__(self, index_speed_signal, scan_speed_signal, enabled_index, enabled_scan):
    def __init__(self):
        super(GamepadControls, self).__init__()
        self.timer_gamepad_update = qtc.QTimer()
        self.connected_last = tuple((False, False, False, False))

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

        self.s_last_zoom_click = time.time()
        self.s_last_focus_click = time.time()

        self.btn_state_last = None
        self.thumb_state_last = None
        self.trigger_state_last = None

    def init_loop(self):
        # Set a time which runs the main loop which gathers data about the gamepad
        self.timer_gamepad_update.timeout.connect(self.main_loop)
        self.timer_gamepad_update.start(self.refresh_wait)

    # Slot: rumble the gamepad when the main thread detects an over current condition.
    def rumble_for_over_current(self, rumble):
        try:
            connection = get_connected()
            # Don't rumble if the gamepad is not connected
            if connection.count(True) == 1:
                index = connection.index(True)
                if rumble:
                    set_vibration(index, 50000, 50000)
                else:
                    set_vibration(index, 0, 0)
        except:
            pass

    def main_loop(self):
        try:
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

                    # Check for camera zoom tele command
                    if btn_state["START"] != self.btn_state_last["START"]:
                        if btn_state["START"]:
                            if self.check_zoom_double_click():
                                self.camera_power_state_updated.emit()
                            else:
                                self.zoomed_tele.emit()
                        else:
                            self.zoom_stopped.emit()

                    # Check for camera zoom wide command
                    if btn_state["BACK"] != self.btn_state_last["BACK"]:
                        if btn_state["BACK"]:
                            if self.check_zoom_double_click():
                                self.camera_power_state_updated.emit()
                            else:
                                self.zoomed_wide.emit()
                        else:
                            self.zoom_stopped.emit()

                    # Check for camera focus near command
                    if btn_state["LEFT_SHOULDER"] != self.btn_state_last["LEFT_SHOULDER"]:
                        if btn_state["LEFT_SHOULDER"]:
                            if self.check_focus_double_click():
                                self.focus_state_updated.emit()
                            else:
                                self.focused_near.emit()
                        else:
                            self.focus_stopped.emit()

                    # Check for camera focus far command
                    if btn_state["RIGHT_SHOULDER"] != self.btn_state_last["RIGHT_SHOULDER"]:
                        if btn_state["RIGHT_SHOULDER"]:
                            if self.check_focus_double_click():
                                self.focus_state_updated.emit()
                            else:
                                self.focused_far.emit()
                        else:
                            self.focus_stopped.emit()

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
                                self.message_created.emit("Entered Vector Mode")
                                set_vibration(index, 20000, 20000)
                                self.light_state = 1
                                self.deactivate_shoulder_and_rotation_axes()
                            elif self.light_state == 1:
                                self.message_created.emit("Exited light mode")
                                set_vibration(index, 0, 0)
                                self.light_state = 0
                            elif self.light_state == 2:
                                self.message_created.emit("Exiting light intensity mode to light vector mode")
                                set_vibration(index, 20000, 20000)
                                self.light_state = 1
                                # If the user exited with the joystick at a non-zero position, than force set point value to zero
                                self.intensity_state_updated.emit(0)
                        # Long press enters light intensity mode
                        elif 1.5 < (time.time() - self.s_last_left_button_click) < 6:
                            self.light_state = 2
                            self.message_created.emit("Entered Light Intensity Mode")
                            set_vibration(index, 30000, 30000)
                            self.deactivate_shoulder_and_rotation_axes()
                            #TIMC

                    # If no light state active, set state variables for shoulder and rotation axes
                    if not self.light_state:
                        self.process_if_move("shoulder", y_init_left, y_final_left, y_pcnfz_left, left_stick_y)
                        self.process_if_move("rotation", x_init_left, x_final_left, x_pcnfz_left, left_stick_x)
                    # Light vector mode
                    elif self.light_state == 1:
                        # Update the brightness of the LEDs if there is an initial, final, or position change of the left joystick
                        if x_init_left or y_init_left or x_final_left or y_final_left or x_pcnfz_left or y_pcnfz_left:
                            self.throw_light(left_stick_x, left_stick_y)
                    elif self.light_state == 2:
                        if y_init_left or y_final_left or y_pcnfz_left:
                            self.intensity_state_updated.emit(left_stick_y)

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
                    self.deactivate_shoulder_and_rotation_axes()
                    self.deactivate_pan_tilt_axes()
                else:
                    self.gamepad_connected.emit()

            self.connected_last = query_connections
        except:
            pass

    # Check if user double clicks a zoom button or clicks both zoom buttons close in succession
    def check_zoom_double_click(self):
        t = time.time()
        check = t - self.s_last_zoom_click
        self.s_last_zoom_click = t
        # Check if the time from the last click registers as a toggle of auto/manual focus
        if check < self.double_click_time:
            return True
        else:
            return False

    # Check if user double clicks a focus button or clicks both focus buttons close in succession
    def check_focus_double_click(self):
        t = time.time()
        check = t - self.s_last_focus_click
        self.s_last_focus_click = t
        # Check if the time from the last click registers as a toggle of auto/manual focus
        if check < self.double_click_time:
            # Brief sleep ensures camera will attempt to autofocus after exiting manual focus mode
            time.sleep(0.25)
            return True
        else:
            return False

    # Calculates the percentage of each LED to be on based on the left joystick position
    def throw_light(self, x, y):
        # Area of light square is 1, 1x1 square
        a = 0.5  # 1/2 length of side
        area = [0, 0, 0, 0]
        res = 200  # Resolution for area calculation (for loop only takes integers)
        dxdy = 0

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

        # Calculate the final area percentages
        for i in range(len(area)):
            area[i] = area[i] / dxdy

        # Emit the signal with the four area percentages
        self.light_state_updated.emit(area)

    def deactivate_shoulder_and_rotation_axes(self):
        self.multi_stopped.emit()
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

    def deactivate_pan_tilt_axes(self):
        self.pan_stopped.emit()
        self.s_pan_positive_initial = False
        self.s_pan_positive_final = False
        self.s_pan_negative_initial = False
        self.s_pan_negative_final = False
        self.s_pan_stationary = True
        self.s_pan_speed_changed = False

        self.tilt_stopped.emit()
        self.s_tilt_positive_initial = False
        self.s_tilt_positive_final = False
        self.s_tilt_negative_initial = False
        self.s_tilt_negative_final = False
        self.s_tilt_stationary = True
        self.s_tilt_speed_changed = False

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
        self.exit()
        self.gamepad_inactive.emit()
        self.connected_last = tuple((False, False, False, False))
        self.btn_state_last = None
        self.thumb_state_last = None
        self.trigger_state_last = None


# Phidget class for opening and saving data about phidgets channels connected
class Phidget(VoltageOutput, VoltageInput, DigitalOutput, DigitalInput, TemperatureSensor):
    num_of_phidgets = 0
    connected_phidgets = 0
    connected_phidgets_mask = 0
    instance_list = []
    data_interval = 150  # ms

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

        Phidget.connected_phidgets_mask += 0b1 << Phidget.num_of_phidgets
        Phidget.num_of_phidgets += 1
        Phidget.instance_list.append(self)

        self.connected = 0
        self.sn = sn
        self.applySerialNumber()
        self.id = Phidget.num_of_phidgets - 1  # The first phidget connected is ID = 0
        self.setHubPort(hub_port)
        self.setChannel(ch)
        self.setOnAttachHandler(self.onAttach)
        self.setOnDetachHandler(self.onDetach)
        self.setOnErrorHandler(self.onError)
        self.phidgetType = phidget_type

    def increment_connection(self):
        self.connected = 1
        Phidget.connected_phidgets |= (0b1 << self.id)

    def decrement_connection(self):
        self.connected = 0
        temp = ~(0b1 << self.id)
        Phidget.connected_phidgets &= temp

    # Set an instance variable representing phidget channel is connected and update class variable
    #   onAttach is called after a Phidget is closed and then opened again
    def onAttach(self, trash):
        self.increment_connection()
        # Set the data interval and trigger for the phidgets which sense voltage for calculating current
        if self.phidgetType == "VoltageInput":
            self.setDataInterval(Phidget.data_interval)
            # Set the voltage change trigger: 0.004 is 1mA
            # self.setVoltageChangeTrigger(0.002)

    # Set a instance variable representing phidget channel is not connected and update class variable
    #   Detach only happens when the phidget is physically disconnected or powered off
    def onDetach(self, trash):
        self.decrement_connection()

    # Respond with state of attachment: 0 not connected, 1 connected. Redundant compared to getAttached()
    def isAttached(self):
        try:
            return self.getAttached()
        except PhidgetException as e:
            print("Exception 0A, " + str(self.id) + ": " + str(hex(e.code)) + " " + str(self) + " " + str(time.time()))
            return -1

    def onError(self, trash, error_code, error_description):
        print("Phidget Error Code:" + str(hex(error_code)))
        pass

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
    high_current_warning = 0.0  # A

    def __init__(self, phidget_type, sn, hub_port, ch, over_curr_signal, under_powered_signal, enabled):
        super(CurrentFeedback, self).__init__(phidget_type, sn, hub_port, ch, self.onUpdate)
        qtc.QObject.__init__(self)
        self.high_current_state = False
        self.over_curr_signal = over_curr_signal
        self.under_powered_signal = under_powered_signal
        self.enabled = enabled
        self.enabled.connect(lambda state: self.update_enabled_state(state))
        self.axis_enabled = False

    def onUpdate(self, trash, voltage):
        current = CurrentFeedback.trans_con * voltage
        # The voltage will be greater than 10V or less than -10V when the amplifier is not enabled and current is 0A
        if not self.axis_enabled:
            self.current_updated.emit(0)
            if self.high_current_state:
                self.over_curr_signal.emit(False)
                self.high_current_state = False
        else:
            self.current_updated.emit(current)

            # If the current is higher than the set point
            if abs(current) > self.high_current_warning:
                # And the state variable has not yet been set
                if not self.high_current_state:
                    self.over_curr_signal.emit(True)
                    self.high_current_state = True
            # Current is less than set point, change state to normal current if not yet set state variable
            elif self.high_current_state:
                self.over_curr_signal.emit(False)
                self.high_current_state = False

    # Slot: Update the high current warning from the main GUI
    def update_high_current(self, high_current):
        # High current setting should be between 1 and 0A, but comes in as 0 to 99
        high_current = high_current / 100
        if 0 <= high_current < 1:
            CurrentFeedback.high_current_warning = high_current

    # Change the variable to start/stop the broadcasting of zero current
    def update_enabled_state(self, state):
        self.axis_enabled = state


# Class for Voltage Output phidget which can control motor speed and LED intensity
class VariableVoltageSource(Phidget):
    def __init__(self, phidget_type, sn, hub_port, ch):
        super(VariableVoltageSource, self).__init__(phidget_type, sn, hub_port, ch)

    def voltageSet(self, value):
        try:
            self.setVoltage(value)
        except PhidgetException as e:
            print("Exception 4, " + str(self.id) + ": " + str(hex(e.code)) + " " + str(self) + " " + str(time.time()))


# Added functionality to Phidget class for toggle and latching of switch
class ToggleSwitch(Phidget, qtc.QObject):
    switch_updated = qtc.pyqtSignal(int)

    def __init__(self, phidget_type, sn, hub_port, ch):
        super(ToggleSwitch, self).__init__(phidget_type, sn, hub_port, ch)
        qtc.QObject.__init__(self)

    # Toggle the state of the switch, latch output
    def toggle(self):
        try:
            if not self.connected:
                self.switch_updated.emit(-1)
                return -1
            elif self.getState():
                self.switch_updated.emit(0)
                self.setState(False)
                return self.getState()
            elif not self.getState():
                self.switch_updated.emit(1)
                self.setState(True)
                return self.getState()
        except PhidgetException as e:
            print("Exception 0, " + str(self.id) + ": " + str(hex(e.code)) + " " + str(self) + " " + str(time.time()))
            self.switch_updated.emit(-1)
            return -1

    def switch_on(self):
        try:
            if not self.connected:
                self.switch_updated.emit(-1)
                return -1
            else:
                self.setState(True)
                self.switch_updated.emit(1)
                return self.getState()
        except PhidgetException as e:
            print("Exception 1, " + str(self.id) + ": " + str(hex(e.code)) + " " + str(self) + " " + str(time.time()))
            self.switch_updated.emit(-1)
            return -1

    def switch_off(self):
        try:
            if not self.connected:
                self.switch_updated.emit(-1)
                return -1
            else:
                self.setState(False)
                self.switch_updated.emit(0)
                return self.getState()
        except PhidgetException as e:
            print("Exception 2, " + str(self.id) + ": " + str(hex(e.code)) + " " + str(self) + " " + str(time.time()))
            self.switch_updated.emit(-1)
            return -1

    def grabState(self):
        try:
            return self.getState()
        except PhidgetException as e:
            print("Exception 3, " + str(self.id) + ": " + str(hex(e.code)) + " " + str(self) + " " + str(time.time()))
            self.switch_updated.emit(-1)
            return -1


# Phidget which senses as a digital input and emits a signal
class DigitalSense(Phidget, qtc.QObject):
    sensor_updated = qtc.pyqtSignal(bool)

    def __init__(self, phidget_type, sn, hub_port, ch):
        super(DigitalSense, self).__init__(phidget_type, sn, hub_port, ch, self.onUpdate)
        qtc.QObject.__init__(self)
        self.sensor_state = None

    def onUpdate(self, trash, sensed_signal):
        self.sensor_updated.emit(sensed_signal)
        self.sensor_state = sensed_signal


# Senses the thermocouples attached to the motor amplifiers
class Thermometer(Phidget, qtc.QObject):
    high_temp_warning = qtc.pyqtSignal(bool)
    temperature_updated = qtc.pyqtSignal(str)
    warning_temp = 0  # (F), initialized to something else at program startup

    def __init__(self, phidget_type, sn, hub_port, ch):
        Phidget.__init__(self, phidget_type, sn, hub_port, ch, self.onTempChange)
        qtc.QObject.__init__(self)
        self.highTempFlag = False

    def onTempChange(self, trash, temperature):
        f = (temperature * 9 / 5) + 32
        self.temperature_updated.emit(str(round(f, 1)))
        # Set a high temperature flag when the setpoint is exceeded
        if f > self.warning_temp and self.highTempFlag == False:
            self.high_temp_warning.emit(True)
            self.highTempFlag = True
        # Once the temperature decreases 2 degrees below the setpoint, reset the flag
        elif f < (self.warning_temp - 2) and self.highTempFlag == True:
            self.highTempFlag = False
            self.high_temp_warning.emit(False)

    def update_warning_temperature(self, new_temp):
        Thermometer.warning_temp = new_temp


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

    # Update the LED brightness based on the new position reported by the slider
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

    def set_intensity_percentage(self, percent):
        span = self.sliderMaximum - self.sliderMinimum
        new_sp = int(span * (1 - percent))
        self.slider.setValue(new_sp)


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
        self.onTimerUpdateGroup.timeout.connect(self.gamepadUpdateGroupBrightness)
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

    def updateSetPoint(self, value):
        # Positive increase in light intensity, i.e. negative delta for slider
        if value != 0:
            self.setPoint = -int(value * 20)
        else:
            self.setPoint = 0

    def update_LEDs_from_gamepad(self, intensities):
        if len(intensities) == len(self.LEDs):
            for i in range(len(intensities)):
                self.LEDs[i].set_intensity_percentage(intensities[i])

    def validateButtonColor(self):
        rv = self.enableSwitch.grabState()
        if rv == -1:
            self.enableButton.setStyleSheet("background-color : red")
        elif rv == 0:
            self.enableButton.setStyleSheet("background-color : light grey")


# The state of the controller as defined by three hub serial numbers and LED values
class State:
    def __init__(self):
        self.hubs = []
        self.leds = []
        self.sn = None
        self.legitimate = False

    # Pass into the state the serial numbers and the minimum LED brightness values
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

    # Set the global variables for hub serial numbers and LED minimum brightness values
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

    def __init__(self, file_name):
        super(Configuration, self).__init__()
        self.fileName = file_name
        self.fileObject = None
        self.isActive = False
        self.readFile()

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

        # Send data to create State and store hub and LED data for later if this configuration were to be set active
        self.initializeState(data_hubs, data_leds)

        # Check if active file was read and set the global variables to be related to this configuration
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


# Go between for Phidget control, takes in the combo box as argument
class ConfigurationManager:
    def __init__(self, combo_box: qtw.QComboBox):
        self.comboBox = combo_box
        self.configurations = []  # List of Configuration objects
        self.configDir = "./Cobra_Commander_Config_Files/"
        self.activeInstanceIndex = None
        self.offlineIndex = None
        self.init_config_manager()
        self.comboBox.currentIndexChanged.connect(self.changeConfiguration)

    # Called once at creation of instance
    def init_config_manager(self):
        config_files = self.init_config_files_directory()

        for i in config_files:
            validated_fn = self.validateFile(i)
            # Create a configuration for each valid filename
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

        # If no active instances or too many active, set in offline mode which is the last index
        if len(active_instances) == 0 or len(active_instances) > 1:
            self.activeInstanceIndex = self.offlineIndex
            self.comboBox.setCurrentIndex(self.activeInstanceIndex)
            if len(active_instances):
                for i in range(len(active_instances)):
                    if i > 0:
                        active_instances[i].makeConfigurationInactive()
        # Only one active configuration,
        elif len(active_instances) == 1:
            self.activeInstanceIndex = self.matchInstanceIndex(active_instances[0])
            self.comboBox.setCurrentIndex(self.activeInstanceIndex)

    def open_config_files_directory(self):
        try:
            os.startfile(os.getcwd() + "/" + self.configDir)
        except:
            pass

    def init_config_files_directory(self):
        # Create a configuration for each valid config file
        try:
            config_files = os.listdir(self.configDir)
        except:
            os.mkdir("./" + self.configDir)
            config_files = os.listdir(self.configDir)
            # Open window for user to drop in config files
            os.startfile(os.getcwd() + "/" + self.configDir)
            msg = qtw.QMessageBox()
            msg.setIcon(qtw.QMessageBox.Information)
            msg.setWindowTitle("Information")
            msg.setText(
                "Copy Cobra Camera configuration files to the location:\n(see window that just opened)\n\n" + os.getcwd() + "\\" +
                self.configDir.split("./")[1].split("/")[0])
            msg.exec_()
        return config_files

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
        # Disable the motors if they are running and close the serial connection
        mw.enable_motor.disable_motors()
        mw.stop_serial_requested.emit()

        for i in Phidget.instance_list:
            i.close()
            i.decrement_connection()
        if self.activeInstanceIndex < self.offlineIndex:
            self.configurations[self.activeInstanceIndex].makeConfigurationInactive()
        if new_index != self.offlineIndex:
            self.configurations[new_index].makeConfigurationActive()
        self.activeInstanceIndex = new_index

        for i in Phidget.instance_list:
            i.applySerialNumber()
            i.open()

        # After a configuration change, update the LED settings and validate the button colors
        mw.LightControl.initLighting()
        mw.LightControl.validateButtonColor()
        mw.enable_motor.validate()
        mw.enable_motor.disable_motors()
        mw.validate_camera_power()

    def make_offline(self):
        self.comboBox.setCurrentIndex(self.offlineIndex)

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
    cmd_mag = 9.6  # multiplier used convert +-10V command signal to +-V for motor at P5 terminal on LA-310T, previously 7.2

    def __init__(self, current: CurrentFeedback, v_cmd: VariableVoltageSource, ch):
        qtc.QObject.__init__(self)
        # Initialize state variables, sv
        self.sv_last_cmd_speed = 0  # units V
        self.sv_last_cmd_voltage = 0  # units V
        self.mtr_current = current
        self.v_cmd = v_cmd
        self.ch = ch
        self.mtr_current.current_updated.connect(self.motor_update)
        self.ir_comp_off = False

    def jog_cw(self, speed):
        self.sv_last_cmd_speed = speed

    # Called from GUI or gamepad
    def jog_ccw(self, speed):
        self.sv_last_cmd_speed = -speed

    def stop_jog(self):
        self.sv_last_cmd_speed = 0
        self.v_cmd.voltageSet(0)

    # Method for IR compensation to achieve constant speed output regardless of load.
    def motor_update(self, current):
        if not mw.ir_comp.isChecked():
            if not self.ir_comp_off:
                self.res_umb = 0
                self.res_mot = 0
                self.ir_comp_off = True
        else:
            if self.ir_comp_off:
                self.res_umb = Motor.res_umb
                self.res_mot = Motor.res_mot
                self.ir_comp_off = False
        if self.v_cmd.isAttached() and self.mtr_current.isAttached():
            # Calculate voltage drop over motor and umbilical
            v_drop = current * (self.res_umb + self.res_mot)
            # Calculate voltage for desired speed. i.e. commanded speed: 3V, calculated voltage drop: 7V, set P5 to 10V
            v_at_p5 = v_drop + self.sv_last_cmd_speed * self.cmd_mag
            # Convert required voltage at P5 to +-10V input command signal
            new_voltage_input = v_at_p5 / self.cmd_mag
            # Set the Voltage Output Phidget to the new voltage. If the command speed is 0V, don't update voltage
            # because the motors will drift
            if -10 <= new_voltage_input <= 10 and self.sv_last_cmd_speed != 0:
                self.v_cmd.voltageSet(new_voltage_input)
            # Requested voltage is out of negative command range
            elif -10 > new_voltage_input and self.sv_last_cmd_speed != 0:
                self.v_cmd.voltageSet(-10)
                self.mtr_current.under_powered_signal.emit()
            # Requested voltage is out of positive command range
            elif 10 < new_voltage_input and self.sv_last_cmd_speed != 0:
                self.v_cmd.voltageSet(10)
                self.mtr_current.under_powered_signal.emit()


# Class for motor control related to enabling and disabling the Varedan drives
class EnableMotors(qtc.QObject):
    drive_enabled = qtc.pyqtSignal(int)
    enable_validated = qtc.pyqtSignal(int)

    def __init__(self, sw_bus: ToggleSwitch, sw_enable: ToggleSwitch):
        qtc.QObject.__init__(self)
        self.sw_bus = sw_bus
        self.state_sw_bus = False
        self.sw_enable = sw_enable
        self.state_sw_enable = False
        self.wait_timer = qtc.QTimer()
        self.wait_timer.timeout.connect(self.enable_drive)

    # Start the process to enable the motors, Bus voltage first
    def enable_motors(self):
        # Check the state of the switches to determine if the motors should be disabled or enabled
        self.state_sw_enable = self.sw_enable.grabState()
        self.state_sw_bus = self.sw_bus.grabState()
        # Motors are disabled, then toggle to enabled
        if self.state_sw_enable == 0 and self.state_sw_bus == 0:
            self.enable_bus()
        # Motors are enabled, then toggle to disabled
        elif self.state_sw_enable == 1 and self.state_sw_bus == 1:
            self.disable_motors()
        # Fatal flaw, enter fail safe mode
        else:
            self.enter_fail_safe()

    # Enable the bus, wait 250ms and then send the enable signal
    def enable_bus(self):
        self.state_sw_bus = self.sw_bus.switch_on()
        self.wait_timer.start(250)

    # Set the enable signal as true, check if switch state is correct and emit signal
    def enable_drive(self):
        self.state_sw_enable = self.sw_enable.switch_on()
        self.wait_timer.stop()
        if self.state_sw_enable and self.state_sw_bus:
            self.drive_enabled.emit(True)

    # Disable the motors
    def disable_motors(self):
        self.state_sw_enable = self.sw_enable.switch_off()
        self.state_sw_bus = self.sw_bus.switch_off()
        # Motors are disabled, emit signal to change button color
        if self.state_sw_enable == 0 and self.state_sw_bus == 0:
            self.drive_enabled.emit(False)
        # Fatal flaw, enter fail safe mode
        else:
            self.enter_fail_safe()

    # Fatal error in code logic attempt to shut down the motors
    def enter_fail_safe(self):
        if self.sw_enable.isAttached():
            self.sw_enable.switch_off()
        if self.sw_bus.isAttached():
            self.sw_bus.switch_off()
        self.drive_enabled.emit(-1)

    # Emit signals for GUI update to button color: Green = On (1), Grey = Off (0), Red = No communication (-1)
    def validate(self):
        self.state_sw_enable = self.sw_enable.grabState()
        self.state_sw_bus = self.sw_bus.grabState()
        if self.state_sw_enable == 1 and self.state_sw_bus == 1:
            self.enable_validated.emit(1)
        elif self.state_sw_enable == 0 and self.state_sw_bus == 0:
            self.enable_validated.emit(0)
        else:
            self.enable_validated.emit(-1)


# Simulated joystick as alternate method to control LEDs not using the gamepad
class LightJoystickWidget(qtw.QWidget):
    light_state_updated = qtc.pyqtSignal(list)
    sticky = 0.2  # values to not exceed 1 inclusive, sticky factor

    def __init__(self, parent=None):
        super(LightJoystickWidget, self).__init__(parent)
        self.dim = 100
        self.resize(self.dim, self.dim)
        self.x = self.dim // 2
        self.y = self.dim // 2

    def paintEvent(self, event):
        paint = qtg.QPainter()
        paint.begin(self)
        paint.setRenderHint(qtg.QPainter.Antialiasing)
        paint.setBrush(qtc.Qt.white)
        paint.drawRect(0, 0, 100, 100)
        radius_x = self.dim // 10
        radius_y = self.dim // 10
        center = qtc.QPoint(self.x, self.y)
        paint.setBrush(qtc.Qt.gray)
        paint.drawEllipse(center, radius_x, radius_y)
        paint.end()

    # When the user moves the mouse
    def mouseMoveEvent(self, event):
        # Calculate sticky parameters
        center = self.dim // 2
        high = center * (1 + self.sticky)
        low = center * (1 - self.sticky)

        # Apply sticky behavior if mouse position is close to the center
        if high > event.x() > low and high > event.y() > low:
            self.x = center
            self.y = center
        else:
            self.x, self.y = event.x(), event.y()

        # Filter out mouse positions
        if self.x > self.width():
            self.x = self.width()
        if self.y > self.height():
            self.y = self.width()
        if self.x < 0:
            self.x = 0
        if self.y < 0:
            self.y = 0

        # Apply scaling factor based on group intensity
        span = mw.s_group_intensity.maximum() - mw.s_group_intensity.minimum()
        value = mw.s_group_intensity.value()
        percent = abs((1 - (value / span)))

        north_west_led = int(self.dim - math.sqrt(self.y ** 2 + self.x ** 2)) / 100
        north_east_led = int(self.dim - math.sqrt((self.y ** 2 + (self.width() - self.x) ** 2))) / 100
        south_east_led = int(self.dim - math.sqrt(((self.height() - self.y) ** 2 + (self.width() - self.x) ** 2))) / 100
        south_west_led = int(self.dim - math.sqrt(((self.height() - self.y) ** 2 + self.x ** 2))) / 100

        # Factor the position of the overall intensity slider
        north_west_led = percent + north_west_led - 0.29
        north_east_led = percent + north_east_led - 0.29
        south_east_led = percent + south_east_led - 0.29
        south_west_led = percent + south_west_led - 0.29

        if north_west_led < 0:
            north_west_led = 0
        if north_east_led < 0:
            north_east_led = 0
        if south_east_led < 0:
            south_east_led = 0
        if south_west_led < 0:
            south_west_led = 0

        if north_west_led > 1:
            north_west_led = 1
        if north_east_led > 1:
            north_east_led = 1
        if south_east_led > 1:
            south_east_led = 1
        if south_west_led > 1:
            south_west_led = 1

        self.light_state_updated.emit([north_west_led, north_east_led, south_east_led, south_west_led])
        self.update()


# Pull in GUI information and pair with code.
class UserWindow(qtw.QMainWindow, Ui_MainWindow):
    count = 0
    over_current = qtc.pyqtSignal(bool)
    motors_enabled = qtc.pyqtSignal(bool)
    under_powered = qtc.pyqtSignal()
    focused_manual = qtc.pyqtSignal()
    focused_auto = qtc.pyqtSignal()

    start_serial_requested = qtc.pyqtSignal(str)
    serial_cmd_requested = qtc.pyqtSignal(str)
    stop_serial_requested = qtc.pyqtSignal()

    timer_requested = qtc.pyqtSignal(int)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setupUi(self)  # Reads in data from Qt Designer ui file

        # Manage the selection options in the serial number combo box
        self.CM = ConfigurationManager(self.serialNumberSelect)
        # Initialize the status LED to blink the front panel as a heartbeat
        self.swStatusLED = ToggleSwitch("DigitalOutput", "HUB0", 1, 3)
        self.heartbeat = qtc.QTimer()
        self.heartbeat.timeout.connect(self.beat)
        self.heartbeat.start(1000)
        self.b_heartBeat.pressed.connect(self.myocardial_infarction)
        self.b_heartBeat.released.connect(self.defibrillation)
        self.offline = True

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

        # Switch which enables power to the LED drivers
        self.swEnableLEDs = ToggleSwitch("DigitalOutput", "HUB0", 1, 2)

        # Pair All LED Controls
        self.LightControl = CameraLighting(self.NorthWestLED, self.NorthEastLED, self.SouthEastLED, self.SouthWestLED,
                                           self.s_group_intensity, self.b_setMinBrightnessLED,
                                           self.b_resetMinimumBrightnessLED, self.b_enableLEDs, self.swEnableLEDs,
                                           self.nw_edit, self.ne_edit, self.se_edit, self.sw_edit, self.apply_min_sp)
        self.LightControl.updated_minimum_brightness.connect(self.CM.updateConfigurationForLEDs)

        self.simulated_joystick = LightJoystickWidget(self.lightWidget)
        self.simulated_joystick.light_state_updated.connect(lambda val: self.LightControl.update_LEDs_from_gamepad(val))

        # Initialize switch for camera power, link button press to power toggle, link button color to signal
        self.swCameraPWR = ToggleSwitch("DigitalOutput", "HUB0", 5, 1)
        self.b_cameraPower.clicked.connect(self.swCameraPWR.toggle)
        self.swCameraPWR.switch_updated.connect(lambda val: self.update_camera_power_status(val))

        self.swLogic = ToggleSwitch("DigitalOutput", "HUB0", 1, 0)
        self.swBus = ToggleSwitch("DigitalOutput", "HUB0", 5, 3)
        self.swEnable = ToggleSwitch("DigitalOutput", "HUB0", 1, 1)

        self.enable_motor = EnableMotors(self.swBus, self.swEnable)
        self.enable_motor.drive_enabled.connect(lambda val: self.update_enable_button(val))
        self.enable_motor.enable_validated.connect(lambda val: self.update_enable_button(val))
        self.b_enable.pressed.connect(lambda: self.enable_motor.enable_motors())

        # Connect the Phidgets which sense motor current
        self.current_non_channel_1A = CurrentFeedback("VoltageInput", "HUB2", 4, 0, self.over_current, self.under_powered,
                                                      self.enable_motor.drive_enabled)
        self.current_channel_side_1B = CurrentFeedback("VoltageInput", "HUB2", 5, 0, self.over_current, self.under_powered,
                                                       self.enable_motor.drive_enabled)
        self.current_pan_2A = CurrentFeedback("VoltageInput", "HUB2", 3, 0, self.over_current, self.under_powered,
                                              self.enable_motor.drive_enabled)
        self.current_tilt_2B = CurrentFeedback("VoltageInput", "HUB2", 2, 0, self.over_current, self.under_powered,
                                               self.enable_motor.drive_enabled)
        self.under_powered.connect(lambda: self.statusbar.showMessage("Motor(s) Underpowered Warning", 3000))

        # Setup timers that will blink the current labels when there is high current
        self.ch_1A_blink_current_warning = qtc.QTimer()
        self.ch_1A_blink_current_warning.timeout.connect(lambda val=("1A", "N/A"): self.blink_current_label(val[0], val[1]))
        self.current_non_channel_1A.over_curr_signal.connect(lambda val: self.blink_current_label("1A", val))
        self.ch_1B_blink_current_warning = qtc.QTimer()
        self.ch_1B_blink_current_warning.timeout.connect(lambda val=("1B", "N/A"): self.blink_current_label(val[0], val[1]))
        self.current_channel_side_1B.over_curr_signal.connect(lambda val: self.blink_current_label("1B", val))
        self.ch_2A_blink_current_warning = qtc.QTimer()
        self.ch_2A_blink_current_warning.timeout.connect(lambda val=("2A", "N/A"): self.blink_current_label(val[0], val[1]))
        self.current_pan_2A.over_curr_signal.connect(lambda val: self.blink_current_label("2A", val))
        self.ch_2B_blink_current_warning = qtc.QTimer()
        self.ch_2B_blink_current_warning.timeout.connect(lambda val=("2B", "N/A"): self.blink_current_label(val[0], val[1]))
        self.current_tilt_2B.over_curr_signal.connect(lambda val: self.blink_current_label("2B", val))

        # Connect signals for updated motor current to the GUI labels
        self.current_non_channel_1A.current_updated.connect(lambda current: self.update_gui_current(current, "1A"))
        self.current_channel_side_1B.current_updated.connect(lambda current: self.update_gui_current(current, "1B"))
        self.current_pan_2A.current_updated.connect(lambda current: self.update_gui_current(current, "2A"))
        self.current_tilt_2B.current_updated.connect(lambda current: self.update_gui_current(current, "2B"))
        self.s_high_motor_current.valueChanged.connect(self.process_high_current_slider_change)
        self.s_high_motor_current.setValue(40)  # Set motor current set point to 0.4A

        # Connect to the Phidgets which voltage signal is used to command motor movement
        self.mtrCmd_non_channel_1A = VariableVoltageSource("VoltageOutput", "HUB0", 2, 0)
        self.mtrCmd_channel_side_1B = VariableVoltageSource("VoltageOutput", "HUB0", 3, 0)
        self.mtrCmd_pan_2A = VariableVoltageSource("VoltageOutput", "HUB2", 1, 0)
        self.mtrCmd_tilt_2B = VariableVoltageSource("VoltageOutput", "HUB2", 0, 0)

        # Create a working class camera_control and assign it to a thread
        self.camera_control = CameraControl()
        self.camera_control_thread = qtc.QThread()
        self.camera_control.moveToThread(self.camera_control_thread)
        self.camera_control_thread.start()

        # Connect signals and slots related to camera control through serial
        self.camera_control.serial_connected.connect(lambda val: self.process_serial_status(val))
        self.start_serial_requested.connect(self.camera_control.start_serial)
        self.serial_cmd_requested.connect(self.camera_control.send_serial_cmd)
        self.stop_serial_requested.connect(self.camera_control.stop_serial)
        self.b_reset_serial.pressed.connect(self.init_serial_connection)
        self.camera_control.message_created.connect(lambda msg: self.statusbar.showMessage(msg, 3000))
        self.camera_control.conflicted_users.connect(self.deconflict_multiple_users)

        # Dictionary of button press signals for camera functions. Dictionary key to be passed in emit signal
        self.camera_signals_dictionary = {
            "zoom_tele": self.b_tele.pressed,
            "zoom_wide": self.b_wide.pressed,
            "zoom_stop": [self.b_tele.released, self.b_wide.released],
            "focus_near": self.b_near.pressed,
            "focus_far": self.b_far.pressed,
            "focus_stop": [self.b_near.released, self.b_far.released],
            "focus_auto": self.focused_auto,
            "focus_manual": self.focused_manual,
            "full_auto": self.b_activate_full_auto_exposure_mode.pressed,
            "inc_exp_comp": self.b_exp_comp_increase.pressed,
            "dec_exp_comp": self.b_exp_comp_decrease.pressed,
            "exp_reset": self.b_exp_comp_reset.pressed,
            "bright_mode": self.b_activate_bright_mode.pressed,
            "bright_up": self.b_bright_mode_increase.pressed,
            "bright_down": self.b_bright_mode_decrease.pressed,
            "bright_reset": self.b_bright_mode_reset.pressed,
            "reset_cam": self.b_reset_camera.pressed,
        }

        # Iterate through the signal dictionary and connect the signals to the slot in the serial worker class
        for key in self.camera_signals_dictionary:
            if isinstance(self.camera_signals_dictionary[key], list):
                for i in range(len(self.camera_signals_dictionary[key])):
                    self.camera_signals_dictionary[key][i].connect(lambda val=key: self.serial_cmd_requested.emit(val))
            else:
                self.camera_signals_dictionary[key].connect(lambda val=key: self.serial_cmd_requested.emit(val))

        self.b_manualSelect.clicked.connect(self.process_manual_select)

        # Create a working class gamepad_control and assign it to a thread
        self.gamepad_control = GamepadControls()
        self.gamepad_control_thread = qtc.QThread()
        self.gamepad_control.moveToThread(self.gamepad_control_thread)
        self.gamepad_control_thread.start()
        # Initiate my main loop after calling moteToThread so the event loops are separated
        self.gamepad_control.init_loop()

        self.gamepad_control.paned_positive.connect(
            lambda val: self.jog_pan_positive(round(val * self.s_joystick_speed_pt.value() / 20, 1)))
        self.gamepad_control.paned_negative.connect(
            lambda val: self.jog_pan_negative(round(val * self.s_joystick_speed_pt.value() / 20, 1)))
        self.gamepad_control.pan_stopped.connect(lambda: self.stop_pan_jog())
        self.gamepad_control.tilted_positive.connect(
            lambda val: self.jog_tilt_positive(round(val * self.s_joystick_speed_pt.value() / 20, 1)))
        self.gamepad_control.tilted_negative.connect(
            lambda val: self.jog_tilt_negative(round(val * self.s_joystick_speed_pt.value() / 20, 1)))
        self.gamepad_control.tilt_stopped.connect(lambda: self.stop_tilt_jog())

        self.gamepad_control.multi_stopped.connect(lambda: self.stop_multi_jog())
        self.gamepad_control.multiple_axes_moved.connect(lambda ch_side, non_ch: self.shoulder_and_rotation_move(ch_side, non_ch))

        self.gamepad_control.gamepad_connected.connect(lambda: self.gamepad_connection.setStyleSheet("background-color : green"))
        self.gamepad_control.gamepad_inactive.connect(lambda: self.gamepad_connection.setStyleSheet("background-color : light grey"))
        self.gamepad_control.light_state_updated.connect(lambda val: self.LightControl.update_LEDs_from_gamepad(val))
        self.gamepad_control.intensity_state_updated.connect(lambda val: self.LightControl.updateSetPoint(val))
        self.gamepad_control.zoomed_tele.connect(self.b_tele.pressed)
        self.gamepad_control.zoom_stopped.connect(self.b_tele.released)
        self.gamepad_control.zoomed_wide.connect(self.b_wide.pressed)
        self.gamepad_control.zoom_stopped.connect(self.b_wide.released)
        self.gamepad_control.focused_near.connect(self.b_near.pressed)
        self.gamepad_control.focus_stopped.connect(self.b_near.released)
        self.gamepad_control.focused_far.connect(self.b_far.pressed)
        self.gamepad_control.focus_stopped.connect(self.b_far.released)
        self.gamepad_control.focus_state_updated.connect(self.process_manual_select)
        self.gamepad_control.camera_power_state_updated.connect(self.swCameraPWR.toggle)

        # When there is an over current situation, rumble the gamepad
        self.over_current.connect(self.gamepad_control.rumble_for_over_current)
        self.gamepad_control.message_created.connect(lambda msg: self.statusbar.showMessage(msg, 3000))

        # Initialize the motors with their v_cmd and current feedback
        self.motor_non_channel = Motor(self.current_non_channel_1A, self.mtrCmd_non_channel_1A, "1A")
        self.motor_channel_side = Motor(self.current_channel_side_1B, self.mtrCmd_channel_side_1B, "1B")
        self.motor_pan = Motor(self.current_pan_2A, self.mtrCmd_pan_2A, "2A")
        self.motor_tilt = Motor(self.current_tilt_2B, self.mtrCmd_tilt_2B, "2B")

        # Pair GUI buttons with their respective jogging methods using the sliders as speed input
        self.b_tilt_positive.pressed.connect(lambda: self.jog_tilt_positive(self.s_tilt_input.value() / 20))
        self.b_tilt_negative.pressed.connect(lambda: self.jog_tilt_negative(self.s_tilt_input.value() / 20))
        self.b_tilt_positive.released.connect(lambda: self.stop_tilt_jog())
        self.b_tilt_negative.released.connect(lambda: self.stop_tilt_jog())
        self.b_pan_positive.pressed.connect(lambda: self.jog_pan_positive(self.s_pan_input.value() / 20))
        self.b_pan_negative.pressed.connect(lambda: self.jog_pan_negative(self.s_pan_input.value() / 20))
        self.b_pan_positive.released.connect(lambda: self.stop_pan_jog())
        self.b_pan_negative.released.connect(lambda: self.stop_pan_jog())
        self.b_extend.pressed.connect(lambda: self.jog_extend(self.s_extend_retract_input.value() / 20))
        self.b_retract.pressed.connect(lambda: self.jog_retract(self.s_extend_retract_input.value() / 20))
        self.b_extend.released.connect(lambda: self.stop_multi_jog())
        self.b_retract.released.connect(lambda: self.stop_multi_jog())
        self.b_rotateCW.pressed.connect(lambda: self.jog_rotation_cw(self.s_rotate_input.value() / 20))
        self.b_rotateCCW.pressed.connect(lambda: self.jog_rotation_ccw(self.s_rotate_input.value() / 20))
        self.b_rotateCW.released.connect(lambda: self.stop_multi_jog())
        self.b_rotateCCW.released.connect(lambda: self.stop_multi_jog())

        # Load the LED settings from the configuration file
        self.LightControl.initLighting()

        self.cabinet_power = DigitalSense("DigitalInput", "HUB0", 0, 0)
        self.cabinet_power.sensor_updated.connect(lambda val: self.update_cabinet_power_status(val))

        self.drive1fault = DigitalSense("DigitalInput", "HUB1", 0, 0)
        self.drive1fault.sensor_updated.connect(lambda val: self.update_fault1_status(val))
        self.b_drive1_fault.pressed.connect(self.reset_stage1)
        self.drive2fault = DigitalSense("DigitalInput", "HUB1", 5, 0)
        self.drive2fault.sensor_updated.connect(lambda val: self.update_fault2_status(val))
        self.b_drive2_fault.pressed.connect(self.reset_stage1)

        # Pair Thermocouples with Label on GUI (Phidget Channel Qty: 2)
        self.temp_drive1 = Thermometer("TemperatureSensor", "HUB0", 4, 1)
        self.temp_drive1.temperature_updated.connect(lambda val: self.l_tempDrive1.setText(val))
        self.temp_drive1.temperature_updated.connect(lambda val: self.l_tempDrive1_e.setText(val))
        self.temp_drive1.high_temp_warning.connect(lambda val, drive=1: self.high_temp_shutdown(val, drive))
        self.temp_drive2 = Thermometer("TemperatureSensor", "HUB0", 4, 2)
        self.temp_drive2.temperature_updated.connect(lambda val: self.l_tempDrive2.setText(val))
        self.temp_drive2.temperature_updated.connect(lambda val: self.l_tempDrive2_e.setText(val))
        self.temp_drive2.high_temp_warning.connect(lambda val, drive=2: self.high_temp_shutdown(val, drive))
        self.high_temp1_warning_blinking = qtc.QTimer()
        self.high_temp1_warning_blinking.timeout.connect(self.blink_drive1_temperature)
        self.high_temp2_warning_blinking = qtc.QTimer()
        self.high_temp2_warning_blinking.timeout.connect(self.blink_drive2_temperature)

        # Connect movements of the temperature slider to update the temperature setpoint which causes an error
        self.s_trip_temp.valueChanged.connect(lambda val: self.temp_drive1.update_warning_temperature(val))
        self.s_trip_temp.setValue(130)  # Set default value to 130F

        self.b_open_config_directory.clicked.connect(self.CM.open_config_files_directory)


    def closeEvent(self, trash):
        self.camera_control.stop_serial()

    # Called when a user tries to connect to a commander that is already in use. This method automatically activates offline mode
    def deconflict_multiple_users(self):
        conflict_sn = self.CM.comboBox.currentText()
        if ".cc" in conflict_sn:
            conflict_sn = conflict_sn.split(".cc")[0]
        if not isinstance(conflict_sn, str):
            conflict_sn = "<Error>"

        self.CM.make_offline()
        multiple_users = qtw.QMessageBox()
        multiple_users.setWindowTitle("Error: Requested Commander Already In Use")
        message = "Entering Offline Mode, the Commander being requested: " + conflict_sn + " is on the network but it is in use"
        multiple_users.setText(message)
        multiple_users.exec_()
        multiple_users.raise_()


    def validate_camera_power(self):
        rv = self.swCameraPWR.grabState()
        if rv == -1:
            self.b_cameraPower.setStyleSheet("background-color : red")
        elif rv == 0:
            self.b_cameraPower.setStyleSheet("background-color : light grey")

    # Method to allow for GUI to open first and then attempt to connect to Phidgets
    def open_phidgets(self):
        self.statusbar.showMessage("Opening Phidget Channels")
        for i in Phidget.instance_list:
            i.open()

    # Change the background of the enable button based on the input
    def update_enable_button(self, value):
        if value == 1:
            self.b_enable.setStyleSheet("background-color : green")
        elif value == 0:
            self.b_enable.setStyleSheet("background-color : light grey")
        elif value == -1:
            self.b_enable.setStyleSheet("background-color : red")

    # Method which is called regularly to flash the front LED on the cabinet
    def beat(self):
        if not ((~Phidget.connected_phidgets) & Phidget.connected_phidgets_mask):
            if self.swStatusLED.isAttached() and self.cabinet_power.sensor_state:
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
                self.b_enable.setEnabled(True)
                self.offline = False
                self.init_serial_connection()




        # All Phidgets are not attached, display error code
        else:
            self.statusbar.showMessage(
                "Phidget Error Code (L->F): " + format(Phidget.connected_phidgets, "0" + str(Phidget.num_of_phidgets) + "b"))
            if not self.offline:
                self.b_heartBeat.setStyleSheet("background-color : red")
                # Check Motor
                self.enable_motor.validate()
                self.enable_motor.disable_motors()
                # Check Camera Power
                self.validate_camera_power()
                # Check LED Power
                self.LightControl.validateButtonColor()
            self.offline = True

    def init_serial_connection(self):
        # Get IP address of a phidget channel on SBC.
        usb_to_serial_ip_address = self.swCameraPWR.getServerPeerName().split(":")[0]
        # Emit the signal which tells the serial thread to start a serial connection
        self.start_serial_requested.emit(usb_to_serial_ip_address)
        # If the default hostname is used for the SBC, multiple SBC's will not be able to be on the network at the same time.
        if "phidgetsbc" in self.swCameraPWR.getServerHostname():
            webbrowser.open_new(usb_to_serial_ip_address)
            rename_hostname_alert = qtw.QMessageBox()
            rename_hostname_alert.setWindowTitle("Warning: SBC Hostname is Default")
            message = "Open web browser if not already with url:\n" + usb_to_serial_ip_address + "/cgi-bin/status-system.sh\n"
            message += "Username = admin\n"
            message += "Password = Isi99word\n"
            message += "Click on \"System\" and rename the Hostname to the serial number\n"
            rename_hostname_alert.setText(message)
            rename_hostname_alert.exec_()
            rename_hostname_alert.raise_()

    # Called when user presses and holds the heartbeat button to identify Cobra Commander
    def myocardial_infarction(self):
        self.b_heartBeat.setStyleSheet("background-color : green")
        self.heartbeat.stop()
        self.swStatusLED.switch_on()

    # Called when user releases heartbeat button
    def defibrillation(self):
        self.b_heartBeat.setStyleSheet("background-color : light grey")
        self.heartbeat.start(1000)

    # Method to update the status of the cabinet power status
    def update_cabinet_power_status(self, power_status):
        if power_status:
            self.l_cabinet_power.setText("ON")
            self.l_cabinet_power.setStyleSheet("background-color : green")
        else:
            self.l_cabinet_power.setText("OFF")
            self.l_cabinet_power.setStyleSheet("background-color : light grey")

    # Method to update the status of the drive 1 fault
    def update_fault1_status(self, fault1):
        if fault1:
            self.b_drive1_fault.setStyleSheet("background-color : red")
        else:
            self.b_drive1_fault.setStyleSheet("background-color : light grey")

    # Method to update the status of the drive 2 fault
    def update_fault2_status(self, fault2):
        if fault2:
            self.b_drive2_fault.setStyleSheet("background-color : red")
        else:
            self.b_drive2_fault.setStyleSheet("background-color : light grey")

    # Change the color of the camera power button
    def update_camera_power_status(self, status):
        if status == 0:
            self.b_cameraPower.setStyleSheet("background-color : light grey")
        elif status == 1:
            self.b_cameraPower.setStyleSheet("background-color : green")
        elif status == -1:
            self.b_cameraPower.setStyleSheet("background-color : red")

    # Process the user selecting the manual or auto focus
    def process_manual_select(self):
        if self.b_near.isEnabled():
            self.b_near.setEnabled(False)
            self.b_far.setEnabled(False)
            self.focused_auto.emit()
        else:
            self.b_near.setEnabled(True)
            self.b_far.setEnabled(True)
            self.focused_manual.emit()

    # Resetting the Varedan Amplifiers is a three step process: Step 1
    def reset_stage1(self):
        self.enable_motor.disable_motors()
        self.enable_gui_buttons_related_to_motion(False)
        text = "Waiting for bus voltage to dissipate"
        self.statusbar.showMessage(text)
        # Continue the reset after waiting for the bus voltage to dissipate
        qtc.QTimer.singleShot(3000, self.reset_stage2)

    def reset_stage2(self):
        self.swLogic.switch_on()
        text = "Resetting Drives"
        self.statusbar.showMessage(text)
        qtc.QTimer.singleShot(1000, self.reset_stage3)

    def reset_stage3(self):
        self.swLogic.switch_off()
        self.enable_gui_buttons_related_to_motion(True)
        text = "Reset Complete"
        self.statusbar.showMessage(text, 3000)

    # Set GUI button enabled state related to axes motion
    def enable_gui_buttons_related_to_motion(self, value):
        if value:
            state = True
        else:
            state = False
        self.b_enable.setEnabled(state)
        self.b_extend.setEnabled(state)
        self.b_retract.setEnabled(state)
        self.b_rotateCW.setEnabled(state)
        self.b_rotateCCW.setEnabled(state)
        self.b_pan_positive.setEnabled(state)
        self.b_pan_negative.setEnabled(state)
        self.b_tilt_positive.setEnabled(state)
        self.b_tilt_negative.setEnabled(state)
        self.b_drive1_fault.setEnabled(state)
        self.b_drive2_fault.setEnabled(state)

    def process_serial_status(self, status):
        if status:
            update = True
        else:
            update = False
        self.b_manualSelect.setEnabled(update)
        self.b_near.setEnabled(update)
        self.b_far.setEnabled(update)
        self.b_tele.setEnabled(update)
        self.b_wide.setEnabled(update)
        self.b_activate_bright_mode.setEnabled(update)
        self.b_bright_mode_reset.setEnabled(update)
        self.b_bright_mode_increase.setEnabled(update)
        self.b_bright_mode_decrease.setEnabled(update)
        self.b_activate_full_auto_exposure_mode.setEnabled(update)
        self.b_exp_comp_reset.setEnabled(update)
        self.b_exp_comp_increase.setEnabled(update)
        self.b_exp_comp_decrease.setEnabled(update)
        self.b_reset_camera.setEnabled(update)

    # Method which is called when one of the drive temperatures has exceeded the setpoint
    def high_temp_shutdown(self, fault, drive):
        if fault:
            self.enable_motor.disable_motors()
            self.enable_gui_buttons_related_to_motion(False)
            if drive == 1:
                self.high_temp1_warning_blinking.start(1000)
            elif drive == 2:
                self.high_temp2_warning_blinking.start(1000)
        else:
            if drive == 1:
                self.high_temp1_warning_blinking.stop()
                self.l_tempDrive1.setStyleSheet("background-color : light grey")
            elif drive == 2:
                self.high_temp2_warning_blinking.stop()
                self.l_tempDrive2.setStyleSheet("background-color : light grey")
            self.enable_gui_buttons_related_to_motion(True)

    # When there is an over-current condition, blink the corresponding GUI label
    def blink_current_label(self, ch, blink):
        # Check the channel requesting the action
        if ch == "1A":
            # The parameter blink is "N/A" when the timer has timed out
            if blink == "N/A":
                if int(time.time()) % 2:
                    self.l_current1A.setStyleSheet("background-color : light grey")
                else:
                    self.l_current1A.setStyleSheet("background-color : red")

            # The parameter blink is true when there is an over-current warning
            elif blink:
                self.ch_1A_blink_current_warning.start(1000)
                self.l_current1A.setStyleSheet("background-color : red")
            # The parameter blink is false representing a condition change where there is no more over-current
            else:
                self.ch_1A_blink_current_warning.stop()
                self.l_current1A.setStyleSheet("background-color : light grey")
        elif ch == "1B":
            # The parameter blink is "N/A" when the timer has timed out
            if blink == "N/A":
                if int(time.time()) % 2:
                    self.l_current1B.setStyleSheet("background-color : light grey")
                else:
                    self.l_current1B.setStyleSheet("background-color : red")

            # The parameter blink is true when there is an over-current warning
            elif blink:
                self.ch_1B_blink_current_warning.start(1000)
                self.l_current1B.setStyleSheet("background-color : red")
            # The parameter blink is false representing a condition change where there is no more overcurrent
            else:
                self.ch_1B_blink_current_warning.stop()
                self.l_current1B.setStyleSheet("background-color : light grey")
        elif ch == "2A":
            # The parameter blink is "N/A" when the timer has timed out
            if blink == "N/A":
                if int(time.time()) % 2:
                    self.l_current2A.setStyleSheet("background-color : light grey")
                else:
                    self.l_current2A.setStyleSheet("background-color : red")

            # The parameter blink is true when there is an over-current warning
            elif blink:
                self.ch_2A_blink_current_warning.start(1000)
                self.l_current2A.setStyleSheet("background-color : red")
            # The parameter blink is false representing a condition change where there is no more over-current
            else:
                self.ch_2A_blink_current_warning.stop()
                self.l_current2A.setStyleSheet("background-color : light grey")
        elif ch == "2B":
            # The parameter blink is "N/A" when the timer has timed out
            if blink == "N/A":
                if int(time.time()) % 2:
                    self.l_current2B.setStyleSheet("background-color : light grey")
                else:
                    self.l_current2B.setStyleSheet("background-color : red")

            # The parameter blink is true when there is an over-current warning
            elif blink:
                self.ch_2B_blink_current_warning.start(1000)
                self.l_current2B.setStyleSheet("background-color : red")
            # The parameter blink is false representing a condition change where there is no more over-current
            else:
                self.ch_2B_blink_current_warning.stop()
                self.l_current2B.setStyleSheet("background-color : light grey")


    def blink_drive1_temperature(self):
        if int(time.time()) % 2:
            self.l_tempDrive1.setStyleSheet("background-color : light grey")
        else:
            self.l_tempDrive1.setStyleSheet("background-color : red")

    def blink_drive2_temperature(self):
        if int(time.time()) % 2:
            self.l_tempDrive2.setStyleSheet("background-color : light grey")
        else:
            self.l_tempDrive2.setStyleSheet("background-color : red")

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

    # Update the label for the high current warning set point for the motors
    def process_high_current_slider_change(self, slider_value):
        text = str(slider_value / 100)
        self.l_high_motor_current.setText(text)
        CurrentFeedback.high_current_warning = float(text)

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
        print(speed)

    def jog_retract(self, speed):
        self.motor_channel_side.jog_ccw(speed)
        self.motor_non_channel.jog_cw(speed)
        print(speed)

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
            self.motor_channel_side.jog_ccw(abs(channel_side) * self.s_joystick_speed_sr.value() / 20)
        elif channel_side > 0:
            self.motor_channel_side.jog_cw(channel_side * self.s_joystick_speed_sr.value() / 20)
        else:
            self.motor_channel_side.stop_jog()

        if non_channel < 0:
            self.motor_non_channel.jog_ccw(abs(non_channel) * self.s_joystick_speed_sr.value() / 20)
        elif non_channel > 0:
            self.motor_non_channel.jog_cw(non_channel * self.s_joystick_speed_sr.value() / 20)
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
