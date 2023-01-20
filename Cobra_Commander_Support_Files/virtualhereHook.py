import subprocess
import platform  # for distinguishing 'Windows', 'Linux', 'Darwin'
import psutil
import sys
import time
from os import listdir


def checkVHClient():
    if '64 bit' in platform.python_compiler():

        running = "vhui64.exe" in (i.name() for i in psutil.process_iter())
        if (not running):
            print("Starting VirtualHere Client x64")
            process = sys.prefix + '\\vhui64.exe'
            rc = subprocess.Popen(process)
            time.sleep(2)
        else:
            print("Connecting to VirtualHere Server")
    if '32 bit' in platform.python_compiler():
        running = "vhui32.exe" in (i.name() for i in psutil.process_iter())
        if (not running):
            print("Starting VirtualHere Client x32")
            process = sys.prefix + '\\vhui32.exe'
            rc = subprocess.Popen(process)
            time.sleep(2)
        else:
            print("Connecting to VirtualHere Server")


def checkBonjour():
    if platform.system() == 'Windows':
        if '64 bit' in platform.python_compiler():
            installer = "\Bonjour64.msi"
        else:
            installer = "\Bonjour.msi"
    state = subprocess.run('sc query "Bonjour Service"', capture_output=True, text=True)
    state = state.stdout.split('\n')
    if (state[3] == ''):
        print("Bonjour Not Installed -- Installing Passively --")
        command = str("msiexec /i " + sys.prefix + installer + " /quiet")
        rc = subprocess.run(command)
        if (rc.returncode == 0):
            print("Install Completely Successfully")
        else:
            print("Install Error -- See Below For Details --")
            print(rc)
    else:
        state = state[3].split(':')
        state = state[1].split(' ')
        if (state[1] == '1'):
            print("Starting Bonjour Service")
            rc = subprocess.run('sc start "Bonjour Service"')

        if (state[1] == '4'):
            print("Bonjour Running")


def checkFTDIDriver():
    # Find the actual driver files. This will tell us if we need to reinstall the drivers. There are other places to see if the drivers are running, but that will only be true if the device is connected.
    driverList = listdir("C:\Windows\System32\DriverStore\FileRepository")
    foundBus = False
    foundport = False
    for driver in driverList:
        driverName = driver.split(".inf_")[0]
        if driverName == "ftdibus":
            foundBus = True
        if driverName == "ftdiport":
            foundport = True
    if (foundBus and foundport):
        print("Serial Drivers Installed")
    else:
        # install the drivers
        print("Follow the prompts to install the serial drivers. ")
        command = sys.prefix + "\\CDM212364_Setup.exe"
        rc = subprocess.run(command)

        driverList = listdir("C:\Windows\System32\DriverStore\FileRepository")
        foundBus = False
        foundport = False
        for driver in driverList:
            driverName = driver.split(".inf_")[0]
            if driverName == "ftdibus":
                foundBus = True
            if driverName == "ftdiport":
                foundport = True
        if (foundBus and foundport):
            print("Serial Drivers Installed")
        else:
            input("Drivers not installed press enter to exit the start sequence............ ")
            sys.exit()


if __name__ == "__main__":
    checkBonjour()
    checkVHClient()
    checkFTDIDriver()
    print("Pre-Flight Checks Passed -- Starting Main UI")
