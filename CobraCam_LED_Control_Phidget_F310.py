# timothy.clark@ge.com
from XInput import *  # Logitech F310 Gamepad module
import tkinter as tk  # GUI module
from Phidget22.Devices.VoltageOutput import *
from Phidget22.Devices.DigitalOutput import *
from Phidget22.Net import *
import os

LIGHT_SQ_DIM = 2
CUR_MAX = 0.458

# Line required to look for Phidget devices on the network
Net.enableServerDiscovery(PhidgetServerType.PHIDGETSERVER_DEVICEREMOTE)

# List to hold Voltage Output Phidgets 0,1,2,3: NE, NW, SW, SE
LED = []
bias = [0, 0, 0, 0]

for i in range(4):
    LED.append(VoltageOutput())
    LED[i].setDeviceSerialNumber(539524)
    LED[i].setIsHubPortDevice(False)
    LED[i].setHubPort(i)
    LED[i].setChannel(0)
    LED[i].openWaitForAttachment(1000)
    LED[i].setVoltageOutputRange(VoltageOutputRange.VOLTAGE_OUTPUT_RANGE_5V)

switch = DigitalOutput()
switch.setDeviceSerialNumber(539524)
switch.setIsHubPortDevice(False)
switch.setHubPort(4)
switch.setChannel(0)
switch.openWaitForAttachment(1000)


def set_LED_current(percent):
    max_current = CUR_MAX * (intensity_var.get() / 100)
    for i in range(len(percent)):
        voltage = -5.8692 * (max_current * percent[i]) + 4.1429 + bias[i]
        if full_on.get():
            LED[i].setVoltage(0)
        else:
            LED[i].setVoltage(voltage)
    switch.setState(True)

def set_LED_bias(input):
    bias[0] = bias_NE_var.get()
    bias[1] = bias_NW_var.get()
    bias[2] = bias_SW_var.get()
    bias[3] = bias_SE_var.get()

def gui_configure():
    if os.path.isfile('config.txt'):
        with open('config.txt','r') as f:
            intensity_scale.set(float(f.readline()))
            sensitivity_scale.set(float(f.readline()))
            bias_NE.set(float(f.readline()))
            bias_NW.set(float(f.readline()))
            bias_SW.set(float(f.readline()))
            bias_SE.set(float(f.readline()))

def on_closing():
    switch.setState(False)
    with open('config.txt', 'w') as f:
        f.write(str(intensity_scale.get())+'\n')
        f.write(str(sensitivity_scale.get())+'\n')
        f.write(str(bias_NE.get())+'\n')
        f.write(str(bias_NW.get())+'\n')
        f.write(str(bias_SW.get())+'\n')
        f.write(str(bias_SE.get())+'\n')
    root.destroy()


def led_update():
    # Poll left joystick location: Logitech F310
    x = get_thumb_values(get_state(0))[0][0]
    y = get_thumb_values(get_state(0))[0][1]

    # Calculate each vertex corner of 2x2 square moved per left joystick
    c1 = [x + (LIGHT_SQ_DIM - sensitivity_var.get()) / 2, y + (LIGHT_SQ_DIM - sensitivity_var.get()) / 2]  # NE Corner
    c2 = [x - (LIGHT_SQ_DIM - sensitivity_var.get()) / 2, y + (LIGHT_SQ_DIM - sensitivity_var.get()) / 2]  # NW Corner
    c3 = [x - (LIGHT_SQ_DIM - sensitivity_var.get()) / 2, y - (LIGHT_SQ_DIM - sensitivity_var.get()) / 2]  # SW Corner
    canvas.coords(joystick, 200, 200, 200 + x * 100, 200 - y * 100)  # Depict joystick movement

    dxdy = 0
    area = [0, 0, 0, 0]
    res = 200
    # Integrate light square to calculate percentage of light in each quadrant
    for y in range(int(c2[1] * res), int(c3[1] * res), -1):
        for x in range(int(c2[0] * res), int(c1[0] * res)):

            if y > 0 < x:
                area[0] += 1
            elif y >= 0 >= x:
                area[1] += 1
            elif y < 0 >= x:
                area[2] += 1
            elif y < 0 < x:
                area[3] += 1
            dxdy += 1

    for i in range(len(area)):
        if full_on.get():
            area[i] = 1
        else:
            area[i] = round(area[i] / dxdy, 2)

    set_LED_current(area)

    # Calculate hex color (0-255) for each sub-rectangle: Quadrant 1 through Quadrant 4
    q1c = round(area[0] * 255)
    q2c = round(area[1] * 255)
    q3c = round(area[2] * 255)
    q4c = round(area[3] * 255)

    # Calculate HEX color in RGB for each quadrant. #000000 = BLACK, #FFFFFF = WHITE
    q1c_hex = str('#' + hex(q1c).split('x')[1].zfill(2) + hex(q1c).split('x')[1].zfill(2) +
                  hex(q1c).split('x')[1].zfill(2))
    q2c_hex = str('#' + hex(q2c).split('x')[1].zfill(2) + hex(q2c).split('x')[1].zfill(2) +
                  hex(q2c).split('x')[1].zfill(2))
    q3c_hex = str('#' + hex(q3c).split('x')[1].zfill(2) + hex(q3c).split('x')[1].zfill(2) +
                  hex(q3c).split('x')[1].zfill(2))
    q4c_hex = str('#' + hex(q4c).split('x')[1].zfill(2) + hex(q4c).split('x')[1].zfill(2) +
                  hex(q4c).split('x')[1].zfill(2))

    # Set LED intensity color, WHITE = Full Intensity, BLACK = Off
    canvas.itemconfig(led_ne, fill=q1c_hex)
    canvas.itemconfig(led_nw, fill=q2c_hex)
    canvas.itemconfig(led_sw, fill=q3c_hex)
    canvas.itemconfig(led_se, fill=q4c_hex)


    root.after(10, led_update)  # Every 10 milliseconds check for new events from F310 gamepad


root = tk.Tk()
root.title("Cobra Camera Light Simulation")
canvas = tk.Canvas(root, width=400, height=400, bg="white")
canvas.create_oval(100, 100, 300, 300)  # Outline of camera lens, not modified
canvas.create_rectangle(25, 25, 375, 375)  # Outline of camera body, not modified
led_nw = canvas.create_oval(50, 50, 100, 100)  # Depiction of LED NW, color modified by left joystick
led_se = canvas.create_oval(300, 300, 350, 350)  # Depiction of LED SE, color modified by left joystick
led_ne = canvas.create_oval(300, 50, 350, 100)  # Depiction of LED NE, color modified by left joystick
led_sw = canvas.create_oval(50, 300, 100, 350)  # Depiction of LED SW, color modified by left joystick
joystick = canvas.create_line(200, 200, 200, 200)

intensity_var = tk.DoubleVar()
intensity_scale = tk.Scale(root, orient=tk.HORIZONTAL, variable=intensity_var,
                           label="LED Intensity (% Max LED Current)", length=200)

sensitivity_var = tk.DoubleVar()
sensitivity_scale = tk.Scale(root, orient=tk.HORIZONTAL, variable=sensitivity_var, label="Joystick Sensitivity",
                             length=200, resolution=0.1)
sensitivity_scale.configure(from_=-1.5, to_=1.5)

bias_NE_var = tk.DoubleVar()
bias_NE = tk.Scale(root, orient=tk.HORIZONTAL, variable=bias_NE_var, label="    Bias NE LED", resolution=0.01,
                   command=set_LED_bias)
bias_NE.configure(from_=-0.5, to_=0.5)

bias_NW_var = tk.DoubleVar()
bias_NW = tk.Scale(root, orient=tk.HORIZONTAL, variable=bias_NW_var, label="    Bias NW LED", resolution=0.01,
                   command=set_LED_bias)
bias_NW.configure(from_=-0.5, to_=0.5)

bias_SW_var = tk.DoubleVar()
bias_SW = tk.Scale(root, orient=tk.HORIZONTAL, variable=bias_SW_var, label="    Bias SW LED", resolution=0.01,
                   command=set_LED_bias)
bias_SW.configure(from_=-0.5, to_=0.5)

bias_SE_var = tk.DoubleVar()
bias_SE = tk.Scale(root, orient=tk.HORIZONTAL, variable=bias_SE_var, label="    Bias SE LED", resolution=0.01,
                   command=set_LED_bias)
bias_SE.configure(from_=-0.5, to_=0.5)

full_on = tk.IntVar()
self_destruct = tk.Checkbutton(root, text="Self Destruct (Seriously use CAUTION)", variable = full_on, onvalue=1)

canvas.pack()  # Assemble all widgets into GUI window
intensity_scale.pack()
sensitivity_scale.pack()
bias_NE.pack()
bias_NW.pack()
bias_SW.pack()
bias_SE.pack()
self_destruct.pack()

gui_configure()
led_update()  # Call first instance of function to update LED color

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()  # Run TkInter GUI
