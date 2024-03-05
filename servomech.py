# servomech.py - servomechanism dynamics
# RMM, 26 Nov 2021

import numpy as np
import control as ct

# Parameter values
servomech_params = {
    'J': 100,             # Moment of inertial of the motor
    'b': 10,              # Angular damping of the arm
    'k': 1,               # Spring constant
    'r': 1,               # Location of spring contact on arm
    'l': 2,               # Distance to the read head
    'eps': 0.01,          # Magnitude of velocity-dependent perturbation
}

# State derivative
def servomech_update(t, x, u, params):
    # Extract the configuration and velocity variables from the state vector
    theta = x[0]                # Angular position of the disk drive arm
    thetadot = x[1]             # Angular velocity of the disk drive arm
    tau = u[0]                  # Torque applied at the base of the arm

    # Get the parameter values
    J, b, k, r = map(params.get, ['J', 'b', 'k', 'r'])

    # Compute the angular acceleration
    dthetadot = 1/J * (
        -b * thetadot - k * r * np.sin(theta) + tau)

    # Return the state update law
    return np.array([thetadot, dthetadot])

# System output
def servomech_output(t, x, u, params):
    l, eps = map(params.get, ['l', 'eps'])
    return np.array([l * u[0] - eps * u[1]])

# System dynamics, without full state as output
servomech_fullstate = ct.NonlinearIOSystem(
    servomech_update, None, name='servomech_fullstate',
    params=servomech_params,
    states=['theta_', 'thetadot_'],
    outputs=['theta', 'thetadot'], inputs=['tau'])

# Sensor subsystem
servomech_sensor = ct.NonlinearIOSystem(
    None, servomech_output, name='servomech_sensor',
    params=servomech_params,
    inputs=['theta', 'thetadot'],
    outputs=['y'])

# Full process dynamics
servomech = ct.interconnect(
    [servomech_fullstate, servomech_sensor],  name='servomech',
    inputs=['tau'], outputs=['y'],
    states=['theta', 'thetadot'],
    params=servomech_params)
