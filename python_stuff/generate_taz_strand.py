# enoch omale
import numpy as np
import math
import matplotlib.pyplot as plt
import random


def newHair(root, direction, step, length):
    points = []

    norm = np.linalg.norm(direction)
    if norm:
        d = direction / norm
    else:
        d = np.array([1, 0, 0])
    for i in range(length):
        points.append(root + i * step * d)
    return (points)


def generateRoots(r=1, n=6, mode=0):
    """
    generates a pattern that should lie on the surphace of a sphere centered at 0 with radius r.
    n determines the density.  
    """
    roots = []
    if (mode == 0):
        for i in range(1, n):
            radius = i * r / (n)
            theta = 0
            while theta <= 2*math.pi:
                roots.append(np.array(
                    [radius * math.sin(theta), math.sqrt(r ** 2 - radius ** 2), radius * math.cos(theta)]))
                theta += 2 * math.pi / n
    elif (mode == 1):
        step = 4/n
        a = 1/n
        theta = 0
        rad = a * theta ** 0.7
        while rad < 0.95*r:
            x = rad * math.cos(theta)
            z = rad * math.sin(theta)
            y = math.sqrt(r ** 2 - rad ** 2)
            rt = np.array([x, y, z])
            if (math.isnan(x)):
                print(f"x: {x}, y: {y}, z: {z}\nr:{rad}, theta: {theta}")
            roots.append(rt)
            print(rt)
            theta += step
            rad = a * theta ** 0.7

    return roots


def generateHair(radius, density, spread="radial", mode=0):
    # roots = generateRoots(radius, density, mode)
    roots = [np.array([0,5,0])]
    # n = len(roots)
    # print
    hair = []
    for base in roots:
        if spread == "radial":
            direction = base
        elif spread == "horizontal":
            direction = np.array([1, 0, 0])
        elif spread == "up":
            direction = np.array([0, 1, 0])
        hair.append(newHair(base, direction, 1, 30))
    # for base in roots:
    #     if spread == "radial":
    #         direction = base
    #     elif spread == "horizontal":
    #         direction = np.array([-1, 0, 0])
    #     elif spread == "up":
    #         direction = np.array([0, 1, 0])
    #     hair.append(newHair(base, direction, 1, 40)) ##enoch length!!
    return hair


print("generating hairs...")
all_the_hairs = generateHair(1, 15, spread='horizontal', mode=1)
# print(all_the_hairs)

# # visualising hair distribution you can comment this section out if you don't need to see it.
# x = []
# z = []
# i = []
# j = []
# a = []
# b = []
# c = []
# d = []

# for hair in all_the_hairs:
#     x.append(hair[0][0])
#     z.append(hair[0][2])
#     a.append(hair[-1][0])
#     b.append(hair[-1][2])
#     i.append(hair[0][0])
#     j.append(hair[0][1])
#     c.append(hair[-1][0])
#     d.append(hair[-1][1])

# data = [(i, j, c, d), (x, z, a, b)]
# angle = ["side", "top"]
# fig = plt.figure()
# rows = 1
# columns = 2

# for i in range(2):
#     fig.add_subplot(rows, columns, i+1)
#     plt.scatter(data[i][0], data[i][1])
#     plt.scatter(data[i][2], data[i][3])
#     plt.title(f"hair distribution ({angle[i]} view).")
# plt.show()

# creating file
# f = open("demofile2.txt", "a")
# f.write("Now the file has more content!")
# f.close()
exps = [{
    "center": np.array([0,0,0]),
    "speed": np.array([0,0,0])
}]
"""GENERATING MOVEMENT"""



def generateExperiments(experiments):
    for e in range(len(experiments)):
        center = experiments[e]["center"]
        hspeed = experiments[e]["speed"]
        with open(f"..\\meshes\\single_curly_hair.mss", 'w') as F:
            # ## generating vertex/particle data
            # # for i in range(len(all_the_hairs)):
            # #     for j in range(len(all_the_hairs[0])):
            # #         x = float(all_the_hairs[i][j][0])
            # #         y = float(all_the_hairs[i][j][1])
            # #         z = float(all_the_hairs[i][j][2])
            # #         F.write(f"p {x} {y} {z} 0 0 0 2")

            n = 0
            for hair in all_the_hairs:
                size = 0
                for p in hair:
                    P = p + center
                    F.write(f"p {P[0]} {P[1]} {P[2]} 0 0 0 1\n")
                    n += 1
                    size += 1
            v = 0
            while v < n:
                print(f"v={v} @enoch")
                i = math.floor(v / size)
                j = v % size

                if (j):
                    if (v+1) % size:
                        F.write(f"t {v} {v+1} 5000000 4472\n")
                else:
                    hair = all_the_hairs[i][j] + center
                    F.write(f"z {v} {hair[0]} {hair[1]} {hair[2]} 2000\n")
                    F.write(f"t {v} {v+1} 5000000 4472\n")
                v += 1
            F.write(f"c {center[0]} {center[1]} {center[2]}\n")
            F.write(f"h {hspeed[0]} {hspeed[1]} {hspeed[2]}\n")

            F.close()
        # print("done.")
        # print(f"vertex count: {n}")
        # print(f"number of hairs: {len(all_the_hairs)}")
        # print(f"number of points per hair: {len(all_the_hairs[0])}")

# s 28 29 50
# z 0 1 0 0 200
# print(f"hair 1: {all_the_hairs[0]}")

# image = np.array(all_the_hairs)
# plt.title("visualisation of first hair")
# plt.imshow(image)
# plt.show()


generateExperiments(exps)
print(f"number of tests: {len(exps)}")
print("done.")
print(f"vertex count: {len(all_the_hairs[0])*len(all_the_hairs)}")
print(f"number of hairs: {len(all_the_hairs)}")
print(f"number of points per hair: {len(all_the_hairs[0])}")
