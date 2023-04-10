import serial
import numpy as np
import struct
import time
import os
import sys
import glob
import numpy as np
import math
#import matplotlib.pyplot as plt


BUFFER_FORMAT = '<4siii'
BUFFER_SIZE = 16
COM_PORT_RPI_USB = '/dev/ttyACM0'
COM_PORT_LAPTOP = 'COM9'
COM_PORT_RPI_GPIO = '/dev/ttyAMA0' #/dev/ttyS0


##################### SYSTEM SETUP ###############################
def os_setup():
    if sys.platform.startswith("linux"):  # could be "linux", "linux2", "linux3", ...
        os.environ['TZ'] = 'America/New_York'
        time.tzset()
    elif os.name == "nt":
        os.system('tzutil /s "Eastern Standard Time"')

def get_serial():
    if sys.platform.startswith("linux"):  # could be "linux", "linux2", "linux3", ...
        return COM_PORT_RPI_GPIO
    elif os.name == "nt":
        return COM_PORT_LAPTOP    

##################### SERIAL ###############################

def serial_ports():
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
    #print("{}:\tSending Buffer {}".format(timestamp, buffer))

def log_Rx(reply):
    print('\t\tReceived:  ', end='')
    print(reply)

##################### TRJAECTORY GENERATION ###############################

def triangular_helix_trajectory(params, t):
    # params: (X0, l, w, h0, hf, n_loops) in mm
    # t: 0 <= t <= 1
    # returns: (x(t),y(t),z(t)) in mm
    assert(0 <= t and t <= 1)
    X0 = np.array(params[0])
    l = params[1]
    w = params[2]
    h = params[3]
    n_loops = params[4]


    s_loop = l + w + (l**2 + w**2)**(0.5)
    
    loop_number = math.floor(t * n_loops)
    l1_portion = l / s_loop
    l2_portion = w / s_loop
    l3_portion = (l**2 + w**2)**0.5 / s_loop
    loop_progress = ((t * n_loops )- loop_number)

    z = (h/2) + (h/2)*np.sin(2*math.pi * t - math.pi/2)
    if (0 <= loop_progress < l1_portion):
        l1_progress = loop_progress
        X = X0 + [(l1_progress / l1_portion) * l, 0, z]
    elif (l1_portion <= loop_progress < l1_portion + l2_portion):
        l2_progress = loop_progress - l1_portion
        X = X0 + [l, 0, 0] + [- (l2_progress / l2_portion)*l, (l2_progress / l2_portion) * w, z]
    elif (l1_portion + l2_portion < loop_progress <= l1_portion + l2_portion + l3_portion):
        l3_progress = loop_progress - l2_portion - l1_portion
        X = X0 + [0, w, 0] + [0, -(l3_progress / l3_portion) * w, z]
    else:
        X = [0, 0, 0]
        print("loop progress out of bounds")
    x = X[0]
    y = X[1]
    z = X[2]

    return [x,y,z]

def helix_trajectory(params, t):
    # params: (radius, h0, hf, n_loops) in mm
    # t: 0 <= t <= 1
    # returns: (x(t),y(t),z(t)) in mm
    assert(0 <= t and t <= 1)
    radius = params[0]
    h0 = params[1]
    hf = params[2]
    loops = params[3]

    x = radius * np.cos(loops * 2 * np.pi * t)
    y = radius * np.sin(loops * 2 * np.pi * t)

    if t < 0.5:
        z = h0 + (hf - h0) * (2 * t)
    else:
        z = hf - (hf - h0) * 2 * (t - 0.5)

    return [x,y,z]

def line_trajectory(params, t):
    # params: (p0, pF)
    # t: 0 <= t <= 1
    assert(0 <= t and t <= 1)

    p0, pf = params
    x0, y0, z0 = p0
    xf, yf, zf = pf

    x = x0 + (xf - x0) * t
    y = y0 + (yf - y0) * t
    z = z0 + (zf - z0) * t

    return [x, y, z]

def calculate_arc_length(trajectory, params, sample_pts=10000):
    # trajectory: (x,y,z) = traj(params, t) in mm
    # params: (,)
    #  0 <= t <= 1
    # return: s in mm
    s = 0
    for i in range(sample_pts - 1):
        t = i / sample_pts
        pt_a = trajectory(params, t)
        t = (i + 1) / sample_pts
        pt_b = trajectory(params, t)
        s += math.dist(pt_a, pt_b)
    return s

def interpolate(velocity, timestep, trajectory, params):
    # velocity: mm/s
    # timestep: s
    # trajectory: (x,y,z) = traj(params, t) in mm
    # params: (,)

    # discretize curve using 10000 sample pts to calculate curve length
    s = calculate_arc_length(trajectory, params, sample_pts = 10000)
    T = s / velocity
    print("Total Trajectory Time: {}s".format(T))
    intervals = round(T / timestep)


    pts = []
    for i in range(intervals + 1):
        t = i / (intervals + 1)
        pts.append(trajectory(params, t))
    return pts

##################### KINEMATICS ###############################

def point2length(pt_A, pt_B, pt_C, points):
  lengths = []
  for p in points:
    l1 = math.dist(p, pt_A)
    l2 = math.dist(p, pt_B)
    l3 = math.dist(p, pt_C)
    lengths.append([l1, l2, l3])
  return lengths

def point2rad(pt_A, pt_B, pt_C, points, radius):
  thetas = []
  for p in points:
    l1 = math.dist(p, pt_A)
    l2 = math.dist(p, pt_B)
    l3 = math.dist(p, pt_C)
    t1 = l1/radius
    t2 = l2/radius
    t3 = l3/radius
    thetas.append([t1, t2, t3])
  return thetas


############################### Debug/Plotting ####################
def plot_helix():
    ROOM_HEIGHT = 2200 #mm
    ROOM_WIDTH = 1300 #mm
    ROOM_LENGTH = 2000 #mm
    r = 0.3 * min(ROOM_WIDTH, ROOM_LENGTH)
    n_loops = 5
    h0 = 0.2 * ROOM_HEIGHT
    hf = 0.8 * ROOM_HEIGHT
    params = (r, h0, hf, n_loops)
    ax = plt.figure().add_subplot(projection='3d')

    # Prepare arrays x, y, z
    n = 100
    t = np.linspace(0, 1, n)
    z = np.zeros((n,))
    x = np.zeros((n,))
    y = np.zeros((n,))

    for i in range(n):
        tp = t[i]
        xp, yp, zp = helix_trajectory(params, tp)
        x[i] = xp
        y[i] = yp
        z[i] = zp

    ax.plot(x, y, z, label='parametric curve')
    ax.legend()

    plt.show()

#plot_helix()
