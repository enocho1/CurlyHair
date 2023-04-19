import os
#enoch omale
import numpy as np
import math
import matplotlib.pyplot as plt
import random

def burrowWrapper(M, TS="", TE="txt"):
    final_output = []

    def burrow(mainpath, targ_start="", targ_end="txt"):

        # just checking the filetype has a '.' in front
        if(not targ_end.startswith(".")):
            targ_end = "."+targ_end

        # the main bit:loop through files and folders in the mainpath dir
        for filename in os.listdir(mainpath):
            # recursive step if we come across a folder, look into it :)
            if os.path.isdir(os.path.join(mainpath, filename)):
                burrow(os.path.join(mainpath, filename),
                       targ_start, targ_end)
            # not really a base case per se, but it kinda terminates with this else branch
            else:
                #print(filename, targ_start, targ_end)
                # checks for files of our decided format
                if filename.endswith(targ_end) and filename.startswith(targ_start):
                    final_output.append(os.path.join(mainpath, filename))
    burrow(M, TS, TE)
    return(final_output)

def parseDigits(string):
    Output = []
    out = string.split(",")
    for i in range(len(out)):
        if i%4 == 0:
            vec = out[i]
            out[i] = vec.split()
            # try:
            #     Output.append(float(out[i][1]))
            # except:
            #     print("hey enoch")
            #     print(out[i])

            for j in range(len(out[i])):
                dig = out[i][j]
                Output.append(float(dig))
    return Output

def parseResults(loc):
    data = {
        "x":[],
        "y":[]
    }
    lines = []
    with open(loc, "r") as F:
        lines.extend(F.readlines())
        F.close()
    for i in range(len(lines)):
        # every 5th line
        if (i%5 == 0 or i%5 == 1):
            if i%2:
                data["y"].append(parseDigits(lines[i]))
            else:
                ll = parseDigits(lines[i])
                if ll: 
                    data["x"].append(ll)
    data["x"] = np.array(data["x"])
    data["y"] = np.array(data["y"])

    return data





location = "..\\results"
results_files = burrowWrapper(location, "experiment")

dat_container = []
for i in range(len(results_files)):
    l = results_files[i]
    try:
        dat_container.append(parseResults(l))
    except:
        print(f"unable to parse file {i}: {l}")
0
testy = dat_container[76]["y"]
testx = dat_container[76]["x"]

print(f"textx: {testx}")
print(f"textx shape: {testx.shape}")
plt.imshow(testx)
plt.show()
plt.imshow(testy)
plt.show()

