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

    ani = FuncAnimation(fig, update, frames=range(c_eci.shape[0]), blit=True, interval=10)
    ani.save("attitude_constraints.webp")
    plt.show()


def trajectory2blender(t, x, fps=30):
    q = x[:, 3:]
    dt = t[1] - t[0]
    frame_step = fps//dt

    bpy.ops.wm.open_mainfile(filepath="cubesat_model.blend")

    bpy.data.objects["Cube.003"].rotation_mode = 'QUATERNION'
    for i in range(x.shape[0]):
        bpy.data.objects['Cube.003'].rotation_quaternion = q[i, :]
        bpy.data.objects['Cube.003'].keyframe_insert(data_path="rotation_quaternion", frame=int(i * frame_step))

    bpy.context.scene.frame_end = int(x.shape[0] * frame_step)
    bpy.ops.wm.save_as_mainfile(filepath="animation.blend")