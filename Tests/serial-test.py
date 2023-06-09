import serial
import numpy as np
import struct
import time
import os
import sys
import glob

BUFFER_FORMAT = '4siii'
BUFFER_SIZE = 16
COM_PORT = None
COM_PORT_RPI_USB = '/dev/ttyACM0'
COM_PORT_LAPTOP = 'COM9'
COM_PORT_RPI_GPIO = '/dev/ttyS0'

def os_setup():
    global COM_PORT
    if sys.platform.startswith("linux"):  # could be "linux", "linux2", "linux3", ...
        os.environ['TZ'] = 'America/New_York'
        time.tzset()
        COM_PORT = COM_PORT_RPI_GPIO
    elif os.name == "nt":
        os.system('tzutil /s "Eastern Standard Time"')
        COM_PORT = COM_PORT_LAPTOP

def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def make_test_points():
  n = 100
  x = np.linspace(0, 1000, n)
  y = np.linspace(0, 2000, n)
  z = np.linspace(0, 3000, n)
  pts = np.transpose([x,y,z])
  return pts


def get_timestamp():
    return time.strftime('%X')

def log_Tx(buffer):
    decoded_buffer = struct.unpack(BUFFER_FORMAT, buffer)
    cmd = decoded_buffer[0].decode('utf-8')
    timestamp = get_timestamp()
    l1 = decoded_buffer[1]
    l2 = decoded_buffer[2]
    l3 = decoded_buffer[3]
    print("{:}:\tSending Buffer ({:}, {:}, {:}, {:})\n\t\tEncoded as {:}".format(timestamp, cmd, l1, l2, l3, buffer))

def log_Rx(reply):
    print('\t\tReceived:  ', end='')
    print(reply)


def main():
    os_setup()
    print(serial_ports())
    ser = serial.Serial(port=COM_PORT, baudrate=115200, timeout=1)
    time.sleep(3.0)
    points = make_test_points()
    points = points.astype(int)
    n = len(points)
    i = 0
    while True:
        p = points[i]
        cmd = "GOTO" #or "SET " (include space for buffer size)
        buffer = struct.pack(BUFFER_FORMAT, cmd.encode('utf-8'), p[0], p[1], p[2])
        log_Tx(buffer)
        bytes_written = ser.write(buffer)
        time.sleep(0.100)
        reply = ser.read(16)
        log_Rx(reply)


        #print(decoded_buffer)
        i = (i+1)%n
        if i == 5:
            break
    ser.close()
    return 0

if __name__ == "__main__":
    main()