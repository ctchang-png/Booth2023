import numpy as np
from utils import *

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
    print(serial_ports())
    ser = serial.Serial(port=COM_PORT, baudrate=115200, timeout=1)
    time.sleep(3.0)
    points = interpolate(velocity, timestep, helix_trajectory, params)
    lengths = np.array(point2length(POINT_A, POINT_B, POINT_C, points, R_SPOOL))
    lengths = lengths.astype(int)
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