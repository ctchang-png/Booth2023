import numpy as np
from utils import *
from socket import htonl


yes_set = {"Y", "y", "yes", "YES"}
no_set = {"N", "n", "no", "NO"}

ROOM_HEIGHT = 2200 #mm
ROOM_WIDTH = 1300 #mm
ROOM_LENGTH = 2000 #mm


POINT_A = [ROOM_LENGTH, 0, ROOM_HEIGHT]
POINT_B = [0, 0, ROOM_HEIGHT]
POINT_C = [0, ROOM_WIDTH, ROOM_HEIGHT]

R_SPOOL = 20 #mm

COM_PORT = get_serial()

velocity = 200
timestep = 0.100

## helix ##
r = 0.3 * min(ROOM_WIDTH, ROOM_LENGTH)
n_loops = 5
h0 = 0.2 * ROOM_HEIGHT
hf = 0.8 * ROOM_HEIGHT
helix_params = (r, h0, hf, n_loops)

## triangular helix ##
X0 = [0.1*ROOM_LENGTH, 0.1*ROOM_WIDTH, 0.1*ROOM_HEIGHT]
l = 0.8*ROOM_LENGTH
w = 0.8*ROOM_WIDTH
h = 0.8*ROOM_HEIGHT
n_loops = 2
t_helix_params = (X0, l, w, h, n_loops)

# we'll need to add offsets/transforms to center this in the room
# I also need to define a room origin and mounting locations for pulleys

def calibrate(ser):
    Calibration_Codes = {"OFF": "OFF ", 
                         "TENSION": "TNSN", 
                         "SET": "SET "}
    print("Initiating Calibration")

    ## Free Motors and pull to origin
    reply = input("TURN OFF MOTORS? [Y/N]: ")
    if reply in yes_set:

        buffer = struct.pack(BUFFER_FORMAT, 
                             Calibration_Codes["OFF"].encode('utf-8'), 0, 0, 0)

        ser.flushInput()
        ser.write(buffer)
        print("Motors Deactivating")
        print(ser.read())
        print("Pull Island To Origin Location...")
        print()
    elif reply in no_set:
        print("Aborting")
        calibrate(ser)

    ## 
    reply = input("TENSION MOTORS? [Y/N]: ")
    if reply in yes_set:
        buffer = struct.pack(BUFFER_FORMAT, 
                             Calibration_Codes["TENSION"].encode('utf-8'), 0, 0, 0)
        ser.flushInput()
        ser.write(buffer)
        print("Tensioning Motors")
        ser.read()
        print("Motors Locked")
        print()
    elif reply in no_set:
        print("Aborting")
        calibrate(ser)
    
    reply = input("SET ENCODERS? [Y/N]: ")
    if reply in yes_set:
        l1 = int(np.linalg.norm(POINT_A))
        l2 = int(np.linalg.norm(POINT_B))
        l3 = int(np.linalg.norm(POINT_C))
        buffer = struct.pack(BUFFER_FORMAT, 
                             (Calibration_Codes["SET"]).encode('utf-8'),
                             l1, l2, l3)
        ser.flushInput()
        ser.write(buffer)
        print("Setting Encoders to ({}, {}, {})".format(l1, l2, l3))
        print()
    elif reply in no_set:
        calibrate(ser)

    print("Calibration Complete")
    return
           
    return 0

def move_manual(ser):
    print("Manual mode")
    print("<X> <Y> <Z> (in mm)")
    while True:
        reply = input("> ")
        L = reply.split()
        if len(L) != 3:
            continue
        try:
           x, y, z = map(int, L)
        except ValueError:
            continue
        lengths = point2length(POINT_A, POINT_B, POINT_C, [(x, y, z)])[0]
        lengths = map(int, lengths)
        goto(*lengths, ser)
        

def goto(l0, l1, l2, ser):
    
    cmd = "GOTO" #or "SET " (include space for buffer size)
    buffer = struct.pack(BUFFER_FORMAT, cmd.encode('utf-8'), l0, l1, l2)
    signal = ser.read(1)
    bytes_written = ser.write(buffer)
    reply = ser.read(16)


def move_helix(ser):
    helix_points = interpolate(velocity, timestep, helix_trajectory, helix_params)
    pf = helix_points[0]
    p0 = [0, 0, 0]
    rapid_points = interpolate(velocity, timestep, line_trajectory, (p0, pf))
    rapid_lengths = np.array(point2length(POINT_A, POINT_B, POINT_C, rapid_points))
    rapid_lengths = rapid_lengths.astype(int)
    helix_lengths = np.array(point2length(POINT_A, POINT_B, POINT_C, helix_points))
    helix_lengths = helix_lengths.astype(int)

    lengths = []
    lengths.extend(rapid_lengths)
    lengths.extend(helix_lengths)
    n_rapid = len(rapid_lengths)
    n_helix = len(helix_lengths)
    n = len(lengths)
    i = 0
    for i in range(n_rapid):
        l = rapid_lengths[i]
    
    print("Rapid Segment Complete")

    while True:
        l = helix_lengths[i]
        if i == n_helix-1:
            print("Helix Segment Complete")
        
        goto(*l, ser)

        i = (i+1)%n_helix

def move_triangular_helix(ser):
    t_helix_points = interpolate(velocity, timestep, triangular_helix_trajectory, t_helix_params)
    pf = t_helix_points[0]
    p0 = [0, 0, 0]
    rapid_points = interpolate(velocity, timestep, line_trajectory, (p0, pf))
    rapid_lengths = np.array(point2length(POINT_A, POINT_B, POINT_C, rapid_points))
    rapid_lengths = rapid_lengths.astype(int)
    t_helix_lengths = np.array(point2length(POINT_A, POINT_B, POINT_C, t_helix_points))
    t_helix_lengths = t_helix_lengths.astype(int)

    lengths = []
    lengths.extend(rapid_lengths)
    lengths.extend(t_helix_lengths)
    n_rapid = len(rapid_lengths)
    n_helix = len(t_helix_lengths)
    n = len(lengths)
    i = 0
    for i in range(n_rapid):
        l = rapid_lengths[i]
    
    print("Rapid Segment Complete")

    while True:
        l = t_helix_lengths[i]
        if i == n_helix-1:
            print("Helix Segment Complete")
        
        goto(*l, ser)

        i = (i+1)%n_helix

def main():
    os_setup()
    print("Detected COM Ports: ", end="")
    print(serial_ports())
    print("Using COM PORT: {}".format(COM_PORT))
    ser = serial.Serial(port=COM_PORT, baudrate=115200, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE)
    ser.flushInput()
    ser.flushOutput()
    time.sleep(3.0)

    reply = input("Run Calibration? [Y/N]: ")
    if reply in yes_set: 
        calibrate(ser)
    
    reply = input("Select Mode: [M]anual / [H]elix: ")
    if reply[0].upper() == 'M':
        move_manual(ser)
    elif reply[0].upper() == 'H':
        move_helix(ser)
    elif reply[0].upper() == 'T':
        move_triangular_helix(ser)

    ser.close()
    return 0

if __name__ == "__main__":
    main()