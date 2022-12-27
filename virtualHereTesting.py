import vhutil
from Phidget22.Phidget import *
from Phidget22.Devices.DigitalInput import *
from Phidget22.Devices.DigitalOutput import *
from Phidget22.Devices.VoltageOutput import *
from Phidget22.Net import *
import xml.etree.ElementTree as ET
import time


def getIPs():
    state = vhutil.writeAndReadServer("get client state")
    root = ET.fromstring(state[0])
    if (len(root) == 0):
        print("No Servers Found -- Check Network")
        sys.exit()
    else:
        connections = root.findall("./server/connection")
        IPs = []
        for child in connections:
            ip = child.get("ip")
            IPs.append(ip)
        return IPs


def selectIP(availableIPs, phidgetIP):
    for ip in availableIPs:
        if ip == phidgetIP:
            return ip
        else:
            continue

    return


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
    print("ServerPeerName: " + str(ip))

    ch.close()
    return ip


def connectToSerial(ip):
    state = vhutil.writeAndReadServer("get client state")
    root = ET.fromstring(state[0])
    host = "0.0.0.0."
    if (len(root) == 0):
        print("No Servers Found -- Check Network")
        sys.exit()
    else:
        connections = root.findall("./server/connection")
        for child in connections:
            if (child.get("ip") == ip):
                connState = child.get("state")
                host = child.get("serverName")
        devices = root.findall("./server/device")
        for child in devices:
            if (child.get("serverName") == host):
                address = child.get("address")
                # TODO determine and check against product ids.
                idVendor = child.get("idVendor")
                idProduct = child.get("idProduct")
                deviceState = child.get("state")
                if idVendor == 1027 and idProduct == 24577:
                    connAddress = host + "." + address
                    print("Connecting To:" + connAddress)
                    # if(deviceState == '1' and connState == '3'):
                    if (deviceState == '1'):
                        result = useDevice(connAddress)
                        if (result[0] != "OK"):
                            # TODO: write a useful error message
                            print("SOMETHING WENT WRONG!!!! RUN AWAY!!! AAAAAAAAAAAAAAAAAAAAAAAAA!!!!!!!!!")
                        else:
                            time.sleep(3) # Client software needs time for initialization
                            state = vhutil.writeAndReadServer("get client state")
                            root = ET.fromstring(state[0])
                            if (len(root) == 0):
                                print("No Servers Found -- Check Network")
                                sys.exit()
                            else:
                                devices = root.findall("./server/device")
                                for child in devices:
                                    if (child.get("serverName") == host and child.get("state") == '3'):
                                        port = child.get("comPortName")
                                        print("SUCCESSFULLY connected to serial adapater. Port is: " + port)
                # Device state 3 is for connection already existing
                # Device state 3 is for connection already existing
                if (deviceState == '3'):
                    port = child.get("comPortName")
                    print("Already connected to port: " + port)
                    pass


def useDevice(address):
    devices = vhutil.writeAndReadServer("USE," + address)
    return devices


if __name__ == "__main__":
    phidgetIP = connectToPhidgets(512922)
    availableIPs = getIPs()
    ip = selectIP(availableIPs, phidgetIP)
    connectToSerial(ip)
    print("____________________Reached main script__________________")