import numpy as np
from utils import *
from socket import htonl

COM_PORT = get_serial()

velocity = 300
timestep = 0.300
r = 0.3 * min(ROOM_WIDTH, ROOM_LENGTH)
n_loops = 2
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
        ser.write(buffer)
        print("Motors Deactivating")
        ser.read()
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
                             (Calibration_Codes["SET"]).encode('utf-8'), 0, 0, 0)
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
    ser.read()

    calibrate(ser)


    points = interpolate(velocity, timestep, helix_trajectory, params)
    lengths = np.array(point2length(POINT_A, POINT_B, POINT_C, points, R_SPOOL))
    lengths = lengths.astype(int)
    n = len(points)
    i = 0
    while True:
        l = lengths[i] - lengths[0]
        cmd = "GOTO" #or "SET " (include space for buffer size)
        buffer = struct.pack(BUFFER_FORMAT, cmd.encode('utf-8'), l[0], l[1], l[2])
        print("Waiting for Pull request")
        signal = ser.read(1)
        log_Tx(buffer)
        bytes_written = ser.write(buffer)
        #time.sleep(0.050)
        reply = ser.read(16)
        log_Rx(reply)

        i = (i+1)%n
    ser.close()
    return 0

if __name__ == "__main__":
    main()