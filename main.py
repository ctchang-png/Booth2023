import numpy as np
from utils import *
from socket import htonl

COM_PORT = get_serial()

velocity = 30
timestep = 0.300
r = 0.3 * min(ROOM_WIDTH, ROOM_LENGTH)
n_loops = 2
h0 = 0.2 * ROOM_HEIGHT
hf = 0.8 * ROOM_HEIGHT
params = (r, h0, hf, n_loops)
# we'll need to add offsets/transforms to center this in the room
# I also need to define a room origin and mounting locations for pulleys



def main():
    os_setup()
    print("Detected COM Ports: ", end="")
    print(serial_ports())
    print("Using COM PORT: {}".format(COM_PORT))
    ser = serial.Serial(port=COM_PORT, baudrate=115200)
    ser.flushInput()
    ser.flushOutput()
    time.sleep(3.0)
    points = interpolate(velocity, timestep, helix_trajectory, params)
    lengths = np.array(point2length(POINT_A, POINT_B, POINT_C, points, R_SPOOL))
    lengths = lengths.astype(int)
    n = len(points)
    i = 0
    while True:
        l = lengths[i]
        l = [0, 0, 0]
        cmd = "GOTO" #or "SET " (include space for buffer size)
        buffer = struct.pack(BUFFER_FORMAT, cmd.encode('utf-8'), l[0], l[1], l[2])
        #buffer = "{} {} {} {}".format(cmd, l[0], l[1], l[2])
        log_Tx(buffer)
        bytes_written = ser.write(buffer.encode())
        time.sleep(0.050)
        reply = ser.read(100)
        log_Rx(reply)

        i = (i+1)%n
        if i == 10:
            break
    ser.close()
    return 0

if __name__ == "__main__":
    main()