import numpy as np
import matplotlib.pyplot as plt
from quat_helpers import *
import bpy
from matplotlib.animation import FuncAnimation

def animate_quaternions(t, x, c):
    q = x[:, 3:]

    c_eci = qarray(q, qsandwich, c)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)

    # Initialize a line plot
    line, = ax.plot([0, c_eci[0, 0]], [0, c_eci[0, 1]], [0, c_eci[0, 2]], 'b-')

    def update(frame):
        # Update the data of the line plot
        line.set_data([0, c_eci[frame, 0]], [0, c_eci[frame, 1]])
        line.set_3d_properties([0, c_eci[frame, 2]])
        return line,

    ani = FuncAnimation(fig, update, frames=range(c_eci.shape[0]), blit=True)
    plt.show()


def trajectory2blender(t, x, fps=30):
    dt = t[1] - t[0]
    frame_step = fps//dt

    #TODO: Finish animation keyframes using Blender quaternions
    for i in range(x.shape[0]):
        bpy.ops.object.empty_add(location=(x[i, 0], x[i, 1], x[i, 2]))
        bpy.context.object.name = 'trajectory_point_' + str(i)
        bpy.context.object.empty_display_type = 'SPHERE'
        bpy.context.object.empty_display_size = 0.1