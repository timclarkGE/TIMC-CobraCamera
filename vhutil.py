import xml.etree.ElementTree as ET
from base64 import decodebytes
import ipaddress
import colorama
import sys
import os
import getpass
import platform
import getopt
import time

if sys.platform == 'win32':
    import win32pipe, win32file, pywintypes

IPCcommands = {
    '-a': 'auto use device port,{}',
    '-s': 'stop using,{}',
    '-u': 'use,{}',
}
shortopts = 'hpAa:u:s:x:'

helptext = []
helptext.append('Usage: vhutil.py [-h -A -p -u <devnum> | -s <devnum> | -a <devnum> | -x <IPCcommand>]')
helptext.append('\tNo argument shows the list, -h shows this help.')
helptext.append('\t-u or -s uses or stops using a device. -a toggles Auto Use (indicated by **).')
helptext.append('\t is chosen from the white numbers on the left).')
helptext.append('\t-A shows the devices\' addresses, -p asks for a password to be input.')
helptext.append('\tvhutil.py -x "" is equivalent to vhui64 -t "".')
helptext.append('\tTry vhutil.py -x "help"')

wclient = r'\\.\pipe\vhclient' if sys.platform == 'win32' else '/tmp/vhclient'
rclient = r'\\.\pipe\vhclient' if sys.platform == 'win32' else '/tmp/vhclient_response'


class color:
    LM = colorama.Fore.LIGHTMAGENTA_EX
    MA = colorama.Fore.MAGENTA
    CY = colorama.Fore.CYAN
    LC = colorama.Fore.LIGHTCYAN_EX
    BL = colorama.Fore.BLUE
    GR = colorama.Fore.GREEN
    LG = colorama.Fore.LIGHTGREEN_EX
    YE = colorama.Fore.YELLOW
    RE = colorama.Fore.RED
    WH = colorama.Fore.WHITE
    LW = colorama.Fore.LIGHTWHITE_EX
    BRIGHT = colorama.Style.BRIGHT
    UNDERLINE = '\033[4m'
    END = colorama.Style.RESET_ALL


class ClientStateElement():
    def __init__(self, elementTree):
        if elementTree.attrib:
            for k, v in elementTree.items():
                # print(k, v)
                if 'boundConnectionIp' in k and v:
                    ip = decodebytes(v.encode())
                    ip = ipaddress.ip_address(ip)
                    ip = str(ip)
                    # ip = re.findall(r'[0-9]+(?:\.[0-9]+){3}', ip)
                    # command = 'host {}'.format(ip)
                    # proc = subprocess.Popen(command, text = True, shell = True, stdout = subprocess.PIPE)
                    # host = proc.stdout.read()
                    # if 'pointer' in host:
                    #    host = host.split('pointer ')[1].replace('.\n', '')
                    # else:
                    #    host = ip
                    setattr(self, k, ip)
                elif 'clientId' == k and v:
                    splitv = v.split()  # in case clientId is just 'Username (Username)'
                    if len(splitv) > 1 and splitv[0] in splitv[1]:
                        setattr(self, k, splitv[0])
                elif k in ['idVendor', 'idProduct'] and v:
                    v = int(v)
                    setattr(self, k, "{0:0{1}x}".format(v, 4))
                elif v and '=' == v[-1]:
                    setattr(self, k, decodebytes(v.encode()))
                else:
                    setattr(self, k, v)
        else:
            for i, child in enumerate(elementTree):
                tag = child.tag
                if len([a.tag for a in elementTree if tag in a.tag]) > len({a.tag for a in elementTree if tag in a.tag}):
                    tag = child.tag + '_{}'.format(i)
                setattr(self, tag, ClientStateElement(child))


class Opts():
    showAddress = False
    srvnum = 'undefined'
    IPCcommand = 'nodevcommand'
    rerunargs = []

    def __init__(self, argv):
        try:
            self.opts, args = getopt.getopt(argv, shortopts)
        except getopt.GetoptError as err:
            print(err)
            sys.exit(2)
        for opt, arg in self.opts:
            if opt == '-h':
                print('\n'.join(helptext))
                exit()
            if opt in IPCcommands:  # interact with a dev
                if arg.isdecimal():
                    self.srvnum = int(list(arg)[0]) - 1
                    self.devnum = list(arg)[1:]
                    self.devnum = int(''.join(self.devnum))
                    self.IPCcommand = IPCcommands.get(opt, 'bad flag')
            elif opt == '-A':
                self.showAddress = True
                self.rerunargs.append(opt)
            elif opt == '-x':
                self.srvnum = opt
                self.IPCcommand = arg
        if '-p' in [o for o, a in self.opts] and 'use' in self.IPCcommand:
            devpass = getpass.getpass()
            self.IPCcommand = 'use,{},' + devpass


def isClientRunning():
    if sys.platform == 'win32':
        try:
            isrunning = len(win32file.FindFilesW(wclient)) > 0  # throws an error if the file's not there
        except pywintypes.error as err:
            isrunning = False
    elif sys.platform == 'linux':
        isrunning = os.path.exists(wclient)
    return isrunning


def writeAndReadServer(IPCcommand):
    # print(IPCcommand)
    if sys.platform == 'win32':
        IPCcommand = IPCcommand.encode() + b'\n'
        handle = win32file.CreateFile(wclient, win32file.GENERIC_READ | win32file.GENERIC_WRITE,
                                      0, None, win32file.OPEN_EXISTING, win32file.FILE_ATTRIBUTE_NORMAL, None)
        res = win32pipe.SetNamedPipeHandleState(handle, win32pipe.PIPE_READMODE_MESSAGE, None, None)
        exitcode, buf = win32pipe.TransactNamedPipe(handle, IPCcommand, 15000, None)
        buf = buf.decode()
    elif sys.platform == 'linux':
        with open(wclient, 'w') as client:
            client.write(IPCcommand + '\n')
        with open(rclient) as client:
            buf = client.read()
        exitcode = ord(buf[-1:])
        buf = buf[:-1]
    return buf, exitcode


def getClientState():
    xml, exitcode = writeAndReadServer('get client state')
    # print(exitcode)
    clientState = ET.fromstring(xml)
    clientState = ClientStateElement(clientState)
    return clientState


def makeServerLine(connection):
    # for k in connection.__dict__.items(): print(*k)
    line = f'  {color.LM}Server: {connection.serverName}{color.END} ({connection.hostname}.{connection.port})'
    return line


def makeDeviceLine(nickname, device, hostname, srvnum, devnum, showAddress):
    # for k in device.__dict__.items(): print(*k)
    num = int('{0}{1}'.format(srvnum + 1, devnum))
    line = f'    {num:3} {color.GR}{nickname}'
    autoUse = ''
    if hasattr(device, 'autoUse'):
        autoUse = '' if device.autoUse == 'not-set' else '**'
    if showAddress:
        line = f'{line} {color.YE}({hostname}.{device.address})'
    if device.state == '3':
        if platform.node() == device.boundClientHostname:
            line = f'{color.BRIGHT}{line} {color.LW}(In use by you){color.END}'.replace(color.GR, color.LG)
        else:
            line = f'{line} {color.WH}(In use by: {color.LC}{device.clientId} at {device.boundClientHostname}{color.END})'
    return line + color.END + autoUse


def main(argv):
    #### Parse the args
    opts = Opts(argv)
    #### Parse the state of the server
    clientstate = getClientState()
    vhlist = ['VirtualHere devices:']
    alldevices = []
    for i, (key, server) in enumerate(clientstate.__dict__.items()):
        vhlist.append(makeServerLine(server.connection))
        devices = {k: v for k, v in server.__dict__.items() if 'device' in k}
        sorted_devices = {}
        for k, v in devices.items():
            v.nickname = v.product if not v.nickname else v.nickname
            sorted_devices[v.nickname] = v
        sorted_devices = sorted(sorted_devices.items())
        alldevices.append(sorted_devices)
        for j, (nickname, device) in enumerate(sorted_devices):
            vhlist.append(makeDeviceLine(nickname, device, server.connection.hostname, i, j, opts.showAddress))

    #### Do the action
    if opts.srvnum == '-x':
        IPCresp, exitcode = writeAndReadServer(opts.IPCcommand)
        print(IPCresp)
        exit()
    if opts.srvnum != 'undefined':
        srvstr = 'server_{}'.format(opts.srvnum) if len(clientstate.__dict__) > 1 else 'server'
        thisdev = alldevices[opts.srvnum][opts.devnum][1]

        thisaddress = getattr(clientstate, srvstr)
        thisaddress = '{}.{}'.format(thisaddress.connection.hostname, thisdev.address)
        if opts.IPCcommand != 'nodevcommand':
            opts.IPCcommand = opts.IPCcommand.format(thisaddress)
            IPCresp, exitcode = writeAndReadServer(opts.IPCcommand)
            if 'FAILED' in IPCresp and '-u' in [o for o, a in opts.opts] and thisdev.state == '1':
                # If it fails to be "used", but is unused (state == 1), it's probably PW protected
                opts.rerunargs = argv
                opts.rerunargs.append('-p')
                if opts.rerunargs.count('-p') > 3:
                    opts.rerunargs = [e for e in opts.rerunargs if e in ['-A']]
                    print('Three incorrect passwords given, skipping.')
            elif 'OK' not in IPCresp:
                print(r'{}Warning: The command "{}" returned "{}".{}'.format(color.RE, opts.IPCcommand, IPCresp, color.END))
        main(opts.rerunargs)
    else:
        vhlist.append('\nUsage: vhutil.py [-h -A -p -u <devnum> | -s <devnum> | -a <devnum> | -x <IPCcommand>]')
        vhlist.append('\tNo argument shows the list, -h shows help.')
        print('\n'.join(vhlist))


if '__main__' == __name__:
    colorama.init()
    if isClientRunning():
        main(sys.argv[1:])
    else:
        print('Please start a VH client')
    colorama.deinit()