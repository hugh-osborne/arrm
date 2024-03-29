import numpy as np
from numpy import *
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import csv

with open("jointreaction_Un-named analysis._ReactionLoads") as jrfile:

    reader = csv.reader(jrfile, delimiter='\t', quotechar='|')
    i=0
    
    hand_mass = 0.00001
    gravity = 9.80665
    
    fig = plt.figure(figsize=(8,8))
    ax = fig.add_subplot(111, projection='3d')
    alph = 0.1
    next_time = 0.4
    x = 0
    y = 0
    z = 0
    num_rows_read = 0
    for row in reader:
        if i > 15 and float(row[0]) > next_time:
            next_time += 0.01
            start_index = 1
            x += float(row[start_index])/hand_mass
            y += (float(row[start_index+1])/hand_mass)-gravity
            z += float(row[start_index+2])/hand_mass
            ax.quiver(0.0, 0.0 ,0.0, float(row[start_index]) ,(float(row[start_index+2])),(float(row[start_index+1]))-gravity, color = 'red', alpha = alph, lw = 2, arrow_length_ratio = 0.01, normalize=False)
            alph += 0.01
            num_rows_read += 1
            
        i += 1
    
    print((x/num_rows_read),(y/num_rows_read),(z/num_rows_read))
    ax.set_xlim3d(-1.0,1.0)
    ax.set_xlabel("Forward/Backward")
    ax.set_ylim3d(-1.0,1.0)
    ax.set_ylabel("Left/Right")
    ax.set_zlim3d(-1.0,1.0)
    ax.set_zlabel("Up/Down")
    plt.title('force')

    plt.draw()
    plt.show()