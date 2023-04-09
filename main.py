import numpy as np
from utils import *
from socket import htonl

ROOM_HEIGHT = 2200 #mm
ROOM_WIDTH = 1300 #mm
ROOM_LENGTH = 2000 #mm


POINT_A = [ROOM_LENGTH//2, -ROOM_WIDTH//2, ROOM_HEIGHT]
POINT_B = [-ROOM_LENGTH//2, -ROOM_WIDTH//2, ROOM_HEIGHT]
POINT_C = [-ROOM_LENGTH//2, ROOM_WIDTH//2, ROOM_HEIGHT]

R_SPOOL = 20 #mm

COM_PORT = get_serial()

velocity = 100
timestep = 0.100
r = 0.3 * min(ROOM_WIDTH, ROOM_LENGTH)
n_loops = 5
h0 = 0.2 * ROOM_HEIGHT
hf = 0.8 * ROOM_HEIGHT
params = (r, h0, hf, n_loops)
# we'll need to add offsets/transforms to center this in the room
# I also need to define a room origin and mounting locations for pulleys

def calibrate(ser):
    Calibration_Codes = {"OFF": "OFF ", 
                         "TENSION": "TNSN", 
                         "SET": "SET "}
    yes_set = {"Y", "y", "yes", "YES"}
    no_set = {"N", "n", "no", "NO"}
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
        buffer = struct.pack(BUFFER_FORMAT, 
                             (Calibration_Codes["SET"]).encode('utf-8'),
                             int(np.linalg.norm(POINT_A)),
                             int(np.linalg.norm(POINT_B)),
                             int(np.linalg.norm(POINT_C)))
        ser.flushInput()
        ser.write(buffer)
        print("Setting Encoders")
        print()
    elif reply in no_set:
        calibrate(ser)

    print("Calibration Complete")
    return
           
    return 0

def main():
    os_setup()
    print("Detected COM Ports: ", end="")
    print(serial_ports())
    print("Using COM PORT: {}".format(COM_PORT))
    ser = serial.Serial(port=COM_PORT, baudrate=115200, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE)
    ser.flushInput()
    ser.flushOutput()
    time.sleep(3.0)

    calibrate(ser)

    helix_points = interpolate(velocity, timestep, helix_trajectory, params)
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
    while True:
        l = lengths[i]
        cmd = "GOTO" #or "SET " (include space for buffer size)
        buffer = struct.pack(BUFFER_FORMAT, cmd.encode('utf-8'), l[0], l[1], l[2])
        #print("Waiting for Pull request")
        if i == n_rapid:
            print("Rapid Segment Complete")
        if i == n_helix:
            print("Helix Segment Complete")
        signal = ser.read(1)
        #log_Tx(buffer)
        bytes_written = ser.write(buffer)
        #time.sleep(0.050)
        reply = ser.read(16)
        #log_Rx(reply)

        i = (i+1)%n
    ser.close()
    return 0

if __name__ == "__main__":
    main()