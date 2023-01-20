import vhutil
from Phidget22.Phidget import *
from Phidget22.Devices.DigitalInput import *
from Phidget22.Devices.DigitalOutput import *
from Phidget22.Devices.VoltageOutput import *
from Phidget22.Net import *
import xml.etree.ElementTree as ET
import time

# USB to Serial Converter used to send commands to camera P/N: TTL232RG-VSW5V0
TTL232RG_VendorID = str(0x0403)
TTL232RG_ProductID = str(0x6001)


# Temporarily connect to SBC VINT hub channel in order find the IP address of the SBC and then disconnect from channel
def connectToPhidgets(serial):
    Net.enableServerDiscovery(PhidgetServerType.PHIDGETSERVER_DEVICEREMOTE)
    ch = DigitalOutput()
    ch.setDeviceSerialNumber(serial)
    ch.setIsHubPortDevice(True)
    ch.setHubPort(1)
    ch.setChannel(0)
    ch.openWaitForAttachment(5000)

    serverPeerName = ch.getServerPeerName()
    ip = serverPeerName.split(':')[0]

    ch.close()
    return ip


# Function which utilizes the virtual here API to find and connect to a USB to Serial device
def connectToSerial(ip):
    port = "N/A"

    # Query the IP addresses of all virtual here servers on the network
    state = getClientState()
    root = ET.fromstring(state[0])
    host = "0.0.0.0."
    if len(root) == 0:
        print("No Servers Found -- Check Network")
        sys.exit()
    else:
        connections = root.findall("./server/connection")
        for child in connections:
            if child.get("ip") == ip:
                host = child.get("serverName")
                usb_host = child.get("hostname")
        devices = root.findall("./server/device")
        for child in devices:
            if child.get("serverName") == host:
                address = child.get("address")
                idVendor = child.get("idVendor")
                idProduct = child.get("idProduct")
                deviceState = child.get("state")
                # Check if the properties of the serial device connected matches the USB to Serial properties
                if idVendor == TTL232RG_VendorID and idProduct == TTL232RG_ProductID:
                    connAddress = usb_host + "." + address
                    # Device is ready for binding: state 1
                    if deviceState == '1':
                        result = useDevice(connAddress)
                        if result[0] != "OK":
                            print("Error Connecting to Serial Device")
                        else:
                            time.sleep(3)  # Client software needs time for initialization
                            state = getClientState()
                            root = ET.fromstring(state[0])
                            if len(root) == 0:
                                print("No Servers Found -- Check Network")
                                sys.exit()
                            else:
                                devices = root.findall("./server/device")
                                for child in devices:
                                    if child.get("serverName") == host and child.get("state") == '3':
                                        port = child.get("comPortName")
                                        print("SUCCESSFULLY connected to serial adapter. Port is: " + port)
                # Device state 3 is for connection already existing
                if deviceState == '3':
                    port = child.get("comPortName")
                    print("Already connected to port: " + port)

    return port


def useDevice(address):
    devices = vhutil.writeAndReadServer("USE," + address)
    return devices

def getClientState():
    state = vhutil.writeAndReadServer("get client state")
    return state



def intializeSerialConnection(serial_number):
    phidgetIP = connectToPhidgets(512922)
    connectToSerial(phidgetIP)


if __name__ == "__main__":
    phidgetIP = connectToPhidgets(512922)
    print("Port:", connectToSerial(phidgetIP))
    print("____________________Reached main script__________________")
