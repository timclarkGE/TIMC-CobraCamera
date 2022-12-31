import sys

if sys.platform == 'win32':
    import win32pipe, win32file, pywintypes

wclient = r'\\.\pipe\vhclient' if sys.platform == 'win32' else '/tmp/vhclient'
rclient = r'\\.\pipe\vhclient' if sys.platform == 'win32' else '/tmp/vhclient_response'

def writeAndReadServer(IPCcommand):
    if sys.platform == 'win32':
        IPCcommand = IPCcommand.encode() + b'\n'
        handle = win32file.CreateFile(wclient, win32file.GENERIC_READ | win32file.GENERIC_WRITE,
            0, None, win32file.OPEN_EXISTING, win32file.FILE_ATTRIBUTE_NORMAL, None)
        res = win32pipe.SetNamedPipeHandleState(handle, win32pipe.PIPE_READMODE_MESSAGE, None, None)
        exitcode, buf = win32pipe.TransactNamedPipe(handle, IPCcommand, 15000, None)
        buf = buf.decode()
    elif sys.platform == 'linux':
        with open(wclient, 'w') as client:
            client.write(IPCcommand+'\n')
        with open(rclient) as client:
            buf = client.read()
        exitcode = ord(buf[-1:])
        buf = buf[:-1]
    return buf, exitcode