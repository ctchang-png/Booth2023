import numpy as np
import math

ROOM_HEIGHT = 5000 #mm
ROOM_WIDTH = 1300 #mm
ROOM_LENGTH = 2000 #mm

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
    intervals = round(T / timestep)


    pts = []
    for i in range(intervals + 1):
        t = i / (intervals + 1)
        pts.append(trajectory(params, t))
    return pts

def main():
    velocity = 30
    timestep = 0.300
    r = 0.3 * min(ROOM_WIDTH, ROOM_LENGTH)
    n_loops = 2
    h0 = 0.2 * ROOM_HEIGHT
    hf = 0.8 * ROOM_HEIGHT
    params = (r, h0, hf, n_loops)
    # we'll need to add offsets/transforms to center this in the room
    # I also need to define a room origin and mounting locations for pulleys
    pts = interpolate(velocity, timestep, helix_trajectory, params)
    n = len(pts)
    i = 0
    while True:
        p = pts[i]
        print(p)
        i = (i + 1)%n

        if i == 5:
            break
    return 0

if __name__ == "__main__":
    main()