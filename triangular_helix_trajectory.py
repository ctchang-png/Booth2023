from utils import *
import matplotlib.pyplot as plt

ROOM_HEIGHT = 2200 #mm
ROOM_WIDTH = 1300 #mm
ROOM_LENGTH = 2000 #mm

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
    l1_portion = (l / s_loop)
    l2_portion = l1_portion + (w / s_loop)
    l3_portion = 1
    loop_progress = ((t * n_loops )- loop_number)

    print(t)
    print(loop_number)
    print(loop_progress)

    z = (h/2)*np.sin(2*math.pi * t - math.pi/2)
    if (0 <= loop_progress < l1_portion):
        l1_progress = loop_progress
        X = X0 + [(l1_progress / l1_portion) * l, 0, z]
    elif (l1_portion <= loop_progress < l2_portion):
        l2_progress = loop_progress - l1_portion
        X = X0 + [l, 0, 0] + [- (l2_progress / l2_portion)*l, (l2_progress / l2_portion) * w, z]
    elif (l2_portion < loop_progress <= l3_portion):
        l3_progress = loop_progress - l2_portion - l1_portion
        X = X0 + [0, w, 0] + [0, -(l3_progress / l3_portion) * w, z]
    else:
        X = [0, 0, 0]
        print("loop progress out of bounds")

    x = X[0]
    y = X[1]
    z = X[2]

    return [x,y,z]


def main():
    # x, y, z
    # ROOM_LENGTH, ROOM_WIDTH, ROOM_HEIGHT
    X0 = [0.1*ROOM_LENGTH, 0.1*ROOM_WIDTH, 0.1*ROOM_HEIGHT]
    l = 0.8*ROOM_LENGTH
    w = 0.8*ROOM_WIDTH
    h = 0.8*ROOM_HEIGHT
    n_loops = 5

    params = (X0, l, w, h, n_loops)
    ax = plt.figure().add_subplot(projection='3d')

    # Prepare arrays x, y, z
    n = 1000
    t = np.linspace(0, 1, n)
    z = np.zeros((n,))
    x = np.zeros((n,))
    y = np.zeros((n,))

    for i in range(n):
        tp = t[i]
        xp, yp, zp = triangular_helix_trajectory(params, tp)
        x[i] = xp
        y[i] = yp
        z[i] = zp
    print(x[0:5])
    print(y[0:5])
    print(z[0:5])
    ax.plot(x, y, z, label='parametric curve')
    ax.legend()

    plt.show()

main()
