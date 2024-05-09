import numpy as np
import math
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter, MultipleLocator
from mpl_toolkits.mplot3d import Axes3D
from itertools import product, combinations
from numpy import sin, cos

%matplotlib widget 

'''
TO IMPORT THIS TO CURRENT INFRASTRUCTURE:
from plotting2.py import *

once state and control trajectories are generated, plotting() with the details provided below
'''

#SOME FUNCTIONS THAT WILL COME IN HANDY

def rot_mat(q):
    """
    This function finds the rotation matrix for a particular quaternion. 
    Our quaternion formulation uses the Hamilton convention, and represents 
    a rotation from the body frame to ECI frame (local-to-inertial rotation). 
    
    So, the rotation matrix will rotate a body vector to ECI, and its transpose
    will rotate from ECI -> body.
    
    This means that if you want to rotate B_ECI to body frame, you MUST transpose the output of this function.
    
    Inputs:
        # q -- 4 x 1 numpy array, quaternion to find rot matrix of
    Returns:
        # A -- 3 x 3 np matrix to rotate a vector between frames (should rotate body -> ECI based on our quaternion model)
    """
    q = q.reshape((4,1))
    q1 = q[0,0]
    q2 = q[1,0]
    q3 = q[2,0]
    q4 = q[3,0]
    A = np.matrix([[q1**2+q2**2-q3**2-q4**2, 2*(q2*q3-q1*q4), 2*(q2*q4+q1*q3)],
                    [2*(q2*q3+q1*q4), q1**2-q2**2+q3**2-q4**2, 2*(q3*q4-q1*q2)],
                    [2*(q2*q4-q1*q3), 2*(q3*q4+q1*q2), q1**2-q2**2-q3**2+q4**2]])

    return A


def rotate_vector(s, q):
    '''
    This function rotates a vector s by a quaternion q

    Inputs:
        # s -- 3 entry numpy array, vector to be rotated
        # q -- 4 entry numpy array, quaternion to find rot matrix of
    Returns:
        # s_rotated -- 3 entry numpy array that contains rotated s
    '''
    s = [s[0], s[1], s[2]] #to get a 2 or 3u cubesat, simply multiply s[2] by 2 or 3
    s_rot_np = rot_mat(q)@(np.array(s).reshape(3,1))
    s_rotated = [s_rot_np[0].item(), s_rot_np[1].item(), s_rot_np[2].item()]
    return s_rotated


def plot_surface(vecs, q): 
    '''
    This function accepts a list of vectors vecs, quaternion q

    returns plotted surface
    '''
    vecs_rot = [rotate_vector(vec, q) for vec in vecs]
    x = np.array([[vecs_rot[0][0], vecs_rot[1][0]], [vecs_rot[2][0], vecs_rot[3][0]]])
    y = np.array([[vecs_rot[0][1], vecs_rot[1][1]], [vecs_rot[2][1], vecs_rot[3][1]]])
    z = np.array([[vecs_rot[0][2], vecs_rot[1][2]], [vecs_rot[2][2], vecs_rot[3][2]]])
    ax.plot_surface(x, y, z, color=c, zorder=ord)
    ax.view_init(elev=10, azim=45)  # Adjust the elevation and azimuth angles as desired

def plot_cube(dimension, color):
    cube =3

def quat_to_angle(state, camera, sun):
    '''
    This function accepts the sun and camera('s original position) as 3D vectors
    and accepts the state trajectory

    returns an angle trajectory by getting angle from a dot product
    '''
    rotated_camera = np.zeros((len(state[:,0]),3))
    theta = np.zeros(len(state[:,0]))
    dp = np.zeros(len(state[:,0]))
    cosine_theta = np.zeros(len(state[:,0]))
    angle_in_radians = np.zeros(len(state[:,0]))

    for i in range(len(state[:,0])):
        rotated_camera[i,:] = rot_mat(state[i,3:])@(camera)
        dp[i] = np.dot(rotated_camera[i,:], sun)
        cosine_theta[i] = dp[i] / (np.linalg.norm(rotated_camera[i,:],2)*np.linalg.norm(sun,2))
        angle_in_radians[i] = math.acos(cosine_theta[i])
        theta[i] = math.degrees(angle_in_radians[i])
    
    return theta

def power_est(torque):
    ''' 
    This function accepts a torque and outputs a proxy for power (magnitude of torque)
    '''
    return np.linalg.norm(torque,2)

def power_to_energy(powers): 
    '''    
    This function takes trajectory of power and converts to total energy via summing
    '''    
    return sum(powers)

'''
COMMENCE PLOTTING INFRASTRUCTURE




'''


def plot_quat_traj(state):
    plt.figure()
    plt.title("Quaternion Trajectory")
    plt.xlabel('t')
    plt.ylabel('q(t)')
    plt.plot(state[:, 3], label=r'$q_x$', color='blue')
    plt.plot(state[:, 4], label=r'$q_y$', color='purple')
    plt.plot(state[:, 5], label=r'$q_z$', color='red')
    plt.plot(state[:, 6], label=r'$q_w$', color='#00ef00')

    plt.legend(fontsize=10, loc='upper left', shadow=True, frameon=False, ncol=1)
    plt.show()


def plot_control_traj(control):
    ''' 
    This function accepts a control trajectory and plots it + estimates power consumption
    '''
    N = len(control[:,0])
    power_traj = np.zeros(N)
    for i in range(N):
        power_traj[i] = power_est(control[i,:])

    plt.figure()
    plt.title(f"Control Trajectory, Energy Consumption: {power_to_energy(power_traj):.1f}")
    plt.xlabel('t')
    plt.ylabel(r'u(t)')
    plt.plot(state[:, 0], label=r'$u_x$', color='#E57523')
    plt.plot(state[:, 1], label=r'$u_y$', color='#FFC031')
    plt.plot(state[:, 2], label=r'$u_z$', color='#F3DB54')
    plt.plot(power_traj, label=r'Power', color='#B90000')
    
    plt.legend(fontsize=10, loc='lower left', shadow=True, frameon=False, ncol=1)
    plt.show()


def plot_angle_plot(state, camera, sun, angle):
    theta = quat_to_angle(state, camera, sun)

    plt.figure()
    plt.title("Angle Between Camera and Sun Vector")
    plt.xlabel('t')
    plt.ylabel(r'$\theta$(t)')
    plt.plot(theta)
    plt.axhline(angle, color='red', linestyle='--', label='Keep-out angle')
    plt.ylim(angle-15)
    
    plt.legend(fontsize=10, loc='lower left', shadow=True, frameon=False, ncol=1)
    plt.show()


#ANIMATION PLOTTING
def update_box(num):
    ax = plt.gca()
    ax.cla()

    ax.set_xlim3d([-5.0, 5.0])
    ax.set_xlabel('X')

    ax.set_ylim3d([-5.0, 5.0])
    ax.set_ylabel('Y')

    ax.set_zlim3d([-5.0, 5.0])
    ax.set_zlabel('Z')

    q = state[num, 3:7] #change depending on what Xset holds

    camera_base = rotate_vector([2, 0, 0], q) #camera boresight is along the body-frames x-axis
    camera_vec = rotate_vector([5, 0, 0], q)
    ax.quiver(camera_base[0], camera_base[1], camera_base[2], camera_vec[0], camera_vec[1], camera_vec[2], length=1, color='red', label="Camera vector")

    s_left = [[-2, -2, -2], [-2, -2, 2], [-2, 2, -2], [-2, 2, 2]]
    plot_surface(s_left, q, 2, 'black')
    s_right = [[2, -2, -2], [2, -2, 2], [2, 2, -2], [2, 2, 2]]
    plot_surface(s_right, q, 3, 'cyan')
    s_top = [[-2, -2, 2], [-2, 2, 2], [2, -2, 2], [2, 2, 2]]
    plot_surface(s_top, q, 4, 'cyan')
    s_bottom = [[-2, -2, -2], [-2, 2, -2], [2, -2, -2], [2, 2, -2]]
    plot_surface(s_bottom, q, 5, 'cyan')
    s_front = [[-2, 2, -2], [-2, 2, 2], [2, 2, -2], [2, 2, 2]]
    plot_surface(s_front, q, 6, 'cyan')
    s_back = [[-2, -2, -2], [-2, -2, 2], [2, -2, -2], [2, -2, 2]]
    plot_surface(s_back, q, 7, 'cyan')

def animate(state):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim3d([-5.0, 5.0])
    ax.set_xlabel('X')
    ax.set_ylim3d([-5.0, 5.0])
    ax.set_ylabel('Y')
    ax.set_zlim3d([-5.0, 5.0])
    ax.set_zlabel('Z')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])

    ax.grid(True)

    cube_ani = animation.FuncAnimation(fig, update_box, 90, interval=90, blit=False,repeat_delay=1000 )
    cube_ani.save("test.gif")



''' 
THE ALMIGHTY PLOTTER
'''

def plotter(state, control, camera, sun, angle, quat_traj = False, control_traj = False, angle_plot = False, anim = False):
    ''' 
    plotter accepts:
        - state trajectory of shape Nx7, with entries [wx,wy,wz,qx,qy,qz,qw] for each time step
        - control trajectory of shape Nx3, with entries [ux,uy,uz] for each time step
        - angle (for keep-out angle plot)
        - boolean for each plot you want (quat_traj = True, control_traj = True, angle_plot = True, anim = True)
    
    returns: plots of choosing

    example call: plotter(state, control, np.array([0, 0, 1]), np.array([0.75, 0.433, 0.5]), 40, quat_traj = True, control_traj = True, angle_plot = True, anim = True)
    '''

    if quat_traj:
        plot_quat_traj(state)
    if control_traj:
        plot_control_traj(control)
    if angle_plot:
        plot_angle_plot(state, camera, sun, angle)
    if anim:
        animate(state)
    