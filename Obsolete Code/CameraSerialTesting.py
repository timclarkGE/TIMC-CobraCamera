import time
import struct

import serial
import tkinter as tk
import subprocess
import os
import sys



zoom_in = b'\x81\x01\x04\x07\x26\xFF'
zoom_out = b'\x81\x01\x04\x07\x36\xFF'
zoom_stop = b'\x81\x01\x04\x07\x00\xFF'
menu = b'\x81\x01\x70\x01\x26\xFF'
menu_up = b'\x81\x01\x70\x01\x21\xFF'
menu_down = b'\x81\x01\x70\x01\x22\xFF'
auto_exposure = b'\x81\x01\x04\x39\x00\xFF'
manual_exposure = b'\x81\x01\x04\x39\x03\xFF'
increase_exposure_compensation = b'\x81\x01\x04\x0E\x02\xFF'
decrease_exposure_compensation = b'\x81\x01\x04\x0E\x03\xFF'
exposure_compensation_on = b'\x81\x01\x04\x3E\x02\xFF'
exposure_compensation_off = b'\x81\x01\x04\x3E\x03\xFF'
reset_exposure_compensation = b'\x81\x01\x04\x0E\x00\xFF'   # was b'\x81\x01\x04\x3E\x00\xFF'
is_power_on = b'\x81\x09\x04\x00\xFF'
focus_far = b'\x81\x01\x04\x08\x02\xFF'
focus_near = b'\x81\x01\x04\x08\x03\xFF'
focus_stop = b'\x81\x01\x04\x08\x00\xFF'
focus_toggle = b'\x81\x01\x04\x38\x10\xFF'
focus_auto = b'\x81\x01\x04\x38\x02\xFF'
focus_manual = b'\x81\x01\x04\x38\x03\xFF'
wide_dynamic_range_on = b'\x81\x01\x04\x3D\x02\xFF'
wide_dynamic_range_off = b'\x81\x01\x04\x3D\x03\xFF'
blc_on = b'\x81\x01\x04\x33\x02\xFF'
blc_off = b'\x81\x01\x04\x33\x03\xFF'
enter = b'\x81\x01\x70\x01\x26\xFF'
bright_mode = b'\x81\x01\x04\x39\x0D\xFF'
bright_up = b'\x81\x01\x04\x0D\x02\xFF'
bright_down = b'\x81\x01\x04\x0D\x03\xFF'
full_auto = b'\x81\x01\x04\x39\x00\xFF'

exposure_state = False

# p = subprocess.run('./vhui64.exe -t list', capture_output=True, text=True, shell=True)
# p = subprocess.Popen("./vhui64.exe -t list")
# print(p.stdout)
# process = subprocess.Popen(['vhui64.exe', '-t', 'list'])
# proc_stdout = process.communicate()
# print(proc_stdout)
# subprocess.call(["vhui64.exe", "-t", "list"])

# This opens a window with virtual here text that I want to capture
# process = subprocess.Popen(["vhui64.exe", "-t", "LIST"], stdout= subprocess.PIPE)
# stdout,stderr = process.communicate()
# print(stdout,stderr)

# process = subprocess.Popen(["vhui64.exe", "-t", "list", "-r", "out.txt"], stdout= subprocess.PIPE)
# stdout,stderr = process.communicate()
# print(stdout,stderr)
#
# f = open("out.txt", "r")
# data = f.read()
# iter = data.count("-->")
#
# temp = data.split("--> ")[1]
# data = temp.split("\n")[0]
# address = data.split("(")[1]
# address = address.split(")")[0]
# command = "USE," + address
# subprocess.Popen(["vhui64.exe", "-t", command], stdout= subprocess.PIPE)
# time.sleep(2)
ser = serial.Serial('COM4', 9600)

# vhui64.exe -t "LIST" -r "out.txt"

# This does nothing
# process = subprocess.Popen(["vhui64.exe", "-t", "list"], shell=True)
# stdout,stderr = process.communicate()
# print(stdout,stderr)


# This does nothing
# p1 = subprocess.Popen(["vhui64.exe", "-t", "list"], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
# out,err = p1.communicate()
# print(out,err)

# This does nothing
# subprocess.run(['vhui64.exe','-t','list'])
#
# # This does nothing
# p = subprocess.run('vhui64.exe -t list', shell= True, capture_output=True, text=True)
# l = subprocess.run()


def enter_pressed():
    ser.write(enter)

def func_zoom_in(event):
    print("Ready to Zoom In")
    ser.write(zoom_in)


def func_zoom_out(event):
    print("Ready to Zoom Out")
    ser.write(zoom_out)


def func_zoom_stop(event):
    print("Ready to Stop Zoom")
    ser.write(zoom_stop)


def func_set_auto_exposure():
    print("Ready to Set Auto Exposure")
    ser.write(auto_exposure)


def func_set_manual_exposure():
    print("Ready to Set Manual Exposure")
    ser.write(manual_exposure)


def func_increase_exposure():
    print("Ready to Increase Exposure")
    ser.write(increase_exposure_compensation)


def func_decrease_exposure():
    print("Ready to Decrease Exposure")
    ser.write(decrease_exposure_compensation)


def func_toggle_exposure_comp():
    global exposure_state
    if exposure_state:
        # Change to false
        print("Ready to turn OFF exposure compensation")
        exposure_state = False
        ser.write(exposure_compensation_off)
    else:
        # Change to True
        print("Ready to turn ON exposure compensation")
        exposure_state = True
        ser.write(exposure_compensation_on)


def func_reset_exposure_comp():
    print("Ready to Reset Exposure Comp")
    ser.write(reset_exposure_compensation)

def func_menu():
    print("Ready to show menu")
    ser.write(menu)


def func_focus_near(event):
    print("Ready to Focus Near")
    ser.write(focus_near)


def func_focus_far(event):
    print("Ready to Focus Far")
    ser.write(focus_far)


def func_focus_stop(event):
    print("Ready to Focus STOP")
    ser.write(focus_stop)

def func_toggle_af():
    print("Ready to Toggle Focus")
    ser.write(focus_toggle)

def func_inquiry():
    # print("Ready to Ask Question")
    # # print("Response before question:", ser.readline())
    ser.write(is_power_on)
    time.sleep(.05)
    while ser.inWaiting():
        print(ser.read().hex())

def func_turn_auto_focus_on():
    print("Set Auto Focus On")
    ser.write(focus_auto)

def func_turn_manual_focus_on():
    print("Set Manual Focus On")
    ser.write(focus_manual)

def func_wdr_on():
    print("Turing on WDR")
    ser.write(wide_dynamic_range_on)

def func_wdr_off():
    print("Turing off WDR")
    ser.write(wide_dynamic_range_off)

def func_blc_on():
    print("Turing on BLC")
    ser.write(blc_on)
    sys.exit()

def func_blc_off():
    print("Turing off BLC")
    ser.write(blc_off)

def func_bright_mode_on():
    ser.write(bright_mode)

def func_full_auto_mode_on():
    ser.write(full_auto)

def func_bright_mode_up():
    ser.write(bright_up)

def func_bright_mode_down():
    ser.write(bright_down)

main_frame = tk.Tk()
bt_enter = tk.Button(main_frame,text="enter", command=enter_pressed)
bt_zoom_in = tk.Button(main_frame, text="zoom in")
bt_zoom_out = tk.Button(main_frame, text="zoom out")
bt_focus_near = tk.Button(main_frame, text="focus near")
bt_focus_far = tk.Button(main_frame, text="focus far")
bt_toggle_af = tk.Button(main_frame, text="toggle AF", command=func_toggle_af)
bt_auto_exposure = tk.Button(main_frame, text="auto exposure", command=func_set_auto_exposure)
bt_manaul_exposure = tk.Button(main_frame, text="manual exposure", command=func_set_manual_exposure)
bt_increase_exposure = tk.Button(main_frame, text="Increase exposure", command=func_increase_exposure)
bt_decrease_exposure = tk.Button(main_frame, text="Decrease exposure", command=func_decrease_exposure)
bt_toggle_exposure_compensation = tk.Button(main_frame, text="toggle exposure comp", command=func_toggle_exposure_comp)
bt_reset_exposure_compensation = tk.Button(main_frame, text="reset exposure comp", command=func_reset_exposure_comp)
bt_menu = tk.Button(main_frame, text="menu", command=func_menu)
bt_inquiry = tk.Button(main_frame, text="?", command=func_inquiry)
bt_auto_focus_on = tk.Button(main_frame, text = "AUTO FOCUS", command=func_turn_auto_focus_on)
bt_manual_focus_on = tk.Button(main_frame, text = "Manual FOCUS", command=func_turn_manual_focus_on)
bt_wdr_on = tk.Button(main_frame, text = "WDR ON", command=func_wdr_on)
bt_wdr_off = tk.Button(main_frame, text = "WDR OFF", command=func_wdr_off)
bt_blc_on = tk.Button(main_frame, text = "BLC ON", command=func_blc_on)
bt_blc_off = tk.Button(main_frame, text = "BLC OFF", command=func_blc_off)
bt_full_auto_on = tk.Button(main_frame, text = "Full AUTO", command=func_full_auto_mode_on)
bt_bright_mode_on = tk.Button(main_frame, text="BRIGHTMODE", command=func_bright_mode_on)
bt_bright_mode_up = tk.Button(main_frame, text="BRIGHTMODE UP", command=func_bright_mode_up)
bt_bright_mode_down = tk.Button(main_frame, text="BRIGHTMODE DOWN", command=func_bright_mode_down)

bt_zoom_in.bind("<ButtonPress-1>", func_zoom_in)
bt_zoom_out.bind("<ButtonPress-1>", func_zoom_out)
bt_zoom_in.bind("<ButtonRelease-1>", func_zoom_stop)
bt_zoom_out.bind("<ButtonRelease-1>", func_zoom_stop)

bt_focus_near.bind("<ButtonPress-1>", func_focus_near)
bt_focus_far.bind("<ButtonPress-1>", func_focus_far)
bt_focus_near.bind("<ButtonRelease-1>", func_focus_stop)
bt_focus_far.bind("<ButtonRelease-1>", func_focus_stop)

bt_enter.pack()
bt_zoom_in.pack()
bt_zoom_out.pack()
bt_focus_near.pack()
bt_focus_far.pack()
bt_toggle_af.pack()
bt_auto_exposure.pack()
bt_manaul_exposure.pack()
bt_increase_exposure.pack()
bt_decrease_exposure.pack()
bt_toggle_exposure_compensation.pack()
bt_reset_exposure_compensation.pack()
bt_menu.pack()
bt_inquiry.pack()
bt_auto_focus_on.pack()
bt_manual_focus_on.pack()
bt_wdr_on.pack()
bt_wdr_off.pack()
bt_blc_on.pack()
bt_blc_off.pack()
bt_full_auto_on.pack()
bt_bright_mode_on.pack()
bt_bright_mode_up.pack()
bt_bright_mode_down.pack()


main_frame.mainloop()
