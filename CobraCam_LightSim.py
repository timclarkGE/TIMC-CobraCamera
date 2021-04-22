# timothy.clark@ge.com
from XInput import *  # Logitech F310 Gamepad module
import tkinter as tk  # GUI module


def led_update():
    # Get movement from joystick: Logitech F310
    events = get_events()
    for event in events:
        if event.type == EVENT_STICK_MOVED:
            if event.stick == LEFT:
                x = event.x
                y = event.y

                # Calculate each vertex of 2x2 square moved per left joystick
                q1 = [x + 1, y + 1]
                q2 = [x - 1, y + 1]
                q3 = [x - 1, y - 1]
                q4 = [x + 1, y - 1]
                canvas.coords(joystick, 200, 200, 200 + x * 100, 200 - y * 100)  # Depict joystick movement

                # Calculate hex color (0-255) for each sub-rectangle: Quadrant 1 through Quadrant 4
                if q1[0] >= 0 and q1[1] >= 0:
                    q1_percent = abs(round((q1[0] * q1[1]) / 4.0, 2))
                    q1c = round(q1_percent * 255)
                if q2[0] <= 0 and q2[1] >= 0:
                    q2_percent = abs(round((q2[0] * q2[1]) / 4.0, 2))
                    q2c = round(q2_percent * 255)
                if q3[0] <= 0 and q3[1] <= 0:
                    q3_percent = abs(round((q3[0] * q3[1]) / 4.0, 2))
                    q3c = round(q3_percent * 255)
                if q4[0] >= 0 and q4[1] <= 0:
                    q4_percent = abs(round((q4[0] * q4[1]) / 4.0, 2))
                    q4c = round(q4_percent * 255)
                print("LED Intensity NE, NW, SW, SE (%):", q1_percent, q2_percent, q3_percent, q4_percent)

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
canvas.pack()  # Assemble all widgets into GUI window
led_update()  # Call first instance of function to update LED color
root.mainloop()  # Run TkInter GUI
