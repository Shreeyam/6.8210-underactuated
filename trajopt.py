import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from plotting import *
from quat_helpers import *
from pydrake.all import LeafSystem, Simulator, DiagramBuilder, LogVectorOutput, MathematicalProgram, Solve, MakeSemidefiniteRelaxation, Variables
import os
os.environ["MOSEKLM_LICENSE_FILE"] = "mosek.lic"

# Helper functions
def qdot_matrix(q):
    qw, qx, qy, qz = q
    return np.array([
        [-qx, -qy, -qz],
        [qw, -qz, qy],
        [qz, qw, -qx],
        [-qy, qx, qw]
    ])

def euler_eqn(w, u, J, Jinv):
    return Jinv @ (u - np.cross(w, J @ w))

def qdot(q, w):
    return 0.5 * qdot_matrix(q) @ w

def qspace(q):
    return np.array([q[:, 4]/q[:, 3], q[:, 5]/q[:, 3], q[:, 6]/q[:, 3]])

# Dynamics equations

# x = [wx, wy, wz, qw, qx, qy, qz]
def satellite_continuous_dynamics(x, u, J, Jinv):
    w = x[:3]
    q = x[3:]

    w_dot = euler_eqn(w, u, J, Jinv)
    q_dot = qdot(q, w)

    return np.concatenate([w_dot, q_dot])

# Implicit Euler integration for use with solver
def satellite_discrete_dynamics(x, x_next, u, time_step, J, Jinv):
    x_dot = satellite_continuous_dynamics(x_next, u, J, Jinv)
    residuals = x_next - x - time_step * x_dot
    
    return residuals

def generate_linear_quaternion_trajectory(x0, xn, N):
    # Generate a linear quaternion trajectory
    q0 = x0[3:]
    qn = xn[3:]

    # Linearly interpolate
    q = np.zeros((N + 1, 4))
    for i in range(N + 1):
        q[i] = (1 - i/N) * q0 + (i/N) * qn

    # Normalize quaternion to one
    q /= np.linalg.norm(q, axis=1)[:, np.newaxis]

    return np.concatenate([np.zeros((N + 1, 3)), q], axis=1)

def optimize_trajectory(
        x0, 
        xn, 
        dt, 
        N, 
        J, 
        abstol=1e-3, 
        tau_max=None, 
        attitude_constraint=False, 
        avoidance_angle=40, 
        c=None, 
        s=None,
        relaxation=False,
        use_sparsity=False,
        x_init=None):
    prog = MathematicalProgram()

    # 3 angular rates, 4 quaternions
    state = prog.NewContinuousVariables(N + 1, 7, "state")
    # 3 torques
    torques = prog.NewContinuousVariables(N + 1, 3, "torques")

    if(x_init is not None):
        prog.SetInitialGuess(state, x_init)

    # Precompute Jinv to avoid recalculating it
    Jinv = np.linalg.inv(J)

    # Precompute matrices 
    if(attitude_constraint):
        theta = np.deg2rad(avoidance_angle)
        A = s.dot(c.T) + c.dot(s.T) - (s.T.dot(c) + np.cos(theta)) * np.eye(3)
        b = np.cross(s.T, c.T).T
        d = s.T.dot(c) - np.cos(theta)
        
        Ai = np.vstack([np.hstack([d, b.T]), np.hstack([b, A])])

    # For every time step...
    print("Constructing program...")
    for t in range(N):
        # Torque cost
        prog.AddQuadraticCost(np.eye(3), np.zeros(3), torques[t])
        prog.AddQuadraticCost(np.diag([1, 1, 1, 0, 0, 0, 0]), np.zeros(7), state[t])

        # Dynamics constraints
        residuals = satellite_discrete_dynamics(state[t], state[t+1], torques[t], dt, J, Jinv)
        for residual in residuals:
            prog.AddConstraint(residual <= abstol)
            prog.AddConstraint(residual >= -abstol) 

        # Attitude constraints
        if(attitude_constraint):
            prog.AddConstraint(state[t, 3:].dot(Ai).dot(state[t, 3:]) <= 0)

        # Torque constraints
        if(tau_max is not None):
            prog.AddConstraint(torques[t,0] <= tau_max) 
            prog.AddConstraint(torques[t,1] <= tau_max)
            prog.AddConstraint(torques[t,2] <= tau_max) 

            prog.AddConstraint(torques[t,0] >= -tau_max)
            prog.AddConstraint(torques[t,1] >= -tau_max)
            prog.AddConstraint(torques[t,2] >= -tau_max)


    # Initial and final state constraints
    prog.AddLinearEqualityConstraint(state[0, :], x0)
    prog.AddLinearEqualityConstraint(state[-1, :], xn)

    print("Construction done!")
    if(relaxation):
        print("Now constructing relaxation...")
        # Create variable groups for relaxation
        if(use_sparsity):
            # TODO: Consider making a group generator that takes variable time step groups
            groups = [Variables(np.concatenate((state[t, :], state[t+1, :], torques[t,: ]))) for t in range(N)]
            prog = MakeSemidefiniteRelaxation(prog, variable_groups=groups)
        else:
            prog = MakeSemidefiniteRelaxation(prog)
            
        print(prog)

    print("Solving...")
    result = Solve(prog)
    print("Solved!")
    state_opt = result.GetSolution(state)
    torques_opt = result.GetSolution(torques)

    return (result.is_success(), state_opt, torques_opt)
        

