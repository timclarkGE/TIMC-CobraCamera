import time

from Phidget22.Devices.VoltageOutput import *
from Phidget22.Devices.VoltageInput import *
from Phidget22.Devices.DigitalOutput import *
from Phidget22.Devices.DigitalInput import *
from Phidget22.Devices.TemperatureSensor import *
from Phidget22.Net import *
from Phidget22.Devices.Log import *
from Phidget22.LogLevel import *

Net.enableServerDiscovery(PhidgetServerType.PHIDGETSERVER_DEVICEREMOTE)

# Actual voltage at P5 is cmd_voltage*7.2
CMD_VOLTAGE = 0 # Negative voltage moves shoulder up
DATA_INTERVAL = 25
CMD_CH = CMD_VOLTAGE
CMD_NON = CMD_VOLTAGE
MTR_RES = 84


# def onUpdate_tilt(trash, voltage):
#     # Calculate voltage drop 1V/0.25A
#     current = voltage * .25
#     v_drop = current * 9.8
#     # Calculate the required increase in
#     voltage_at_p5 = CMD_VOLTAGE * 7.2
#
#     # Voltage at P5 should be low so add more command voltage
#     new_voltage = voltage_at_p5 + v_drop
#     new_command_voltage = new_voltage / 7.2
#     print("New CMD Voltage", new_command_voltage)
#     command_tilt.setVoltage(new_command_voltage)

def onUpdate_ch_side(trash, voltage):
    current = voltage * .25
    # Calculate voltage drop over the umbilical and the motor windings
    v_drop = current * 9.8 + current * MTR_RES
    # Calculate the required voltage at P5 to achieve commanded voltage
    new_voltage = v_drop + CMD_CH * 7.2
    # Convert new voltage to new command voltage set point
    set_point = new_voltage/7.2

    print("New CMD Voltage CH", set_point)
    if 10 >= set_point >= -10:
        command_ch_side.setVoltage(set_point)


def onUpdate_non_ch_side(trash, voltage):
    current = voltage * .25
    # Calculate voltage drop over the umbilical and the motor windings
    v_drop = current * 9.8 + current * MTR_RES
    # Calculate the required voltage at P5 to achieve commanded voltage
    new_voltage = v_drop + CMD_NON * 7.2
    # Convert new voltage to new command voltage set point
    set_point = new_voltage / 7.2

    print("New CMD Voltage NON CH", set_point)
    if 10 >= set_point >= -10:
        command_non_ch_side.setVoltage(set_point)


# # Open current_tilt sensor tilt
# current_tilt = VoltageInput()
# current_tilt.setDeviceSerialNumber(671924)
# current_tilt.setHubPort(2)
# current_tilt.setChannel(0)
# current_tilt.setIsHubPortDevice(False)
# current_tilt.openWaitForAttachment(5000)
# current_tilt.setDataInterval(DATA_INTERVAL)

# Open current_channel_side
current_ch_side = VoltageInput()
current_ch_side.setDeviceSerialNumber(671924)
current_ch_side.setHubPort(5)
current_ch_side.setChannel(0)
current_ch_side.setIsHubPortDevice(False)
current_ch_side.openWaitForAttachment(5000)
current_ch_side.setDataInterval(DATA_INTERVAL)

# Open current_non_channel_side
current_non_ch_side = VoltageInput()
current_non_ch_side.setDeviceSerialNumber(671924)
current_non_ch_side.setHubPort(4)
current_non_ch_side.setChannel(0)
current_non_ch_side.setIsHubPortDevice(False)
current_non_ch_side.openWaitForAttachment(5000)
current_non_ch_side.setDataInterval(DATA_INTERVAL)

# Open Logic Enable
logic_enable = DigitalOutput()
logic_enable.setDeviceSerialNumber(512922)
logic_enable.setHubPort(1)
logic_enable.setChannel(0)
logic_enable.setIsHubPortDevice(False)
logic_enable.openWaitForAttachment(5000)

# Open Bus Enable
bus_enable = DigitalOutput()
bus_enable.setDeviceSerialNumber(512922)
bus_enable.setHubPort(5)
bus_enable.setChannel(3)
bus_enable.setIsHubPortDevice(False)
bus_enable.openWaitForAttachment(5000)

# Open Amp Enable
amp_enable = DigitalOutput()
amp_enable.setDeviceSerialNumber(512922)
amp_enable.setHubPort(1)
amp_enable.setChannel(1)
amp_enable.setIsHubPortDevice(False)
amp_enable.openWaitForAttachment(5000)

# Open Motor Command for ch_side
command_ch_side = VoltageOutput()
command_ch_side.setDeviceSerialNumber(512922)
command_ch_side.setHubPort(3)
command_ch_side.setChannel(0)
command_ch_side.setIsHubPortDevice(False)
command_ch_side.openWaitForAttachment(5000)

# Open Motor Command for non_ch_side
command_non_ch_side = VoltageOutput()
command_non_ch_side.setDeviceSerialNumber(512922)
command_non_ch_side.setHubPort(2)
command_non_ch_side.setChannel(0)
command_non_ch_side.setIsHubPortDevice(False)
command_non_ch_side.openWaitForAttachment(5000)

# Initiate Startup Sequence
logic_enable.setState(True)
time.sleep(1)
bus_enable.setState(True)
time.sleep(1)
amp_enable.setState(True)

# command_tilt.setVoltage(CMD_VOLTAGE)
command_non_ch_side.setVoltage(CMD_NON)
command_ch_side.setVoltage(CMD_CH)
current_non_ch_side.setOnVoltageChangeHandler(onUpdate_non_ch_side)
current_ch_side.setOnVoltageChangeHandler(onUpdate_ch_side)
# current_tilt.setOnVoltageChangeHandler(onUpdate_tilt)
time.sleep(10)
current_non_ch_side.setOnVoltageChangeHandler(None)
current_ch_side.setOnVoltageChangeHandler(None)

# Initiate Shutdown Sequence
amp_enable.setState(False)
bus_enable.setState(False)
time.sleep(5)
logic_enable.setState(False)
