#! /usr/bin/env python3

"""
    # Erik Wirdemark
    # erikwir@kth.se
"""

import math
import numpy as np


def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    """
    Fill in your IK solution here and return the three joint values in q
    """
    l0 = 0.07
    l1 = 0.3
    l2 = 0.35

    r = (x-l0)**2 + y**2
    cosine_q2 = (r - l1**2 - l2**2) / (2 * l1 * l2)
    if cosine_q2 < -1 or cosine_q2 > 1:
        raise ValueError("The point is out of reach")
    q2 = math.acos(cosine_q2)
    q1 = math.atan2(y, x-l0) - math.atan2(l2 * math.sin(q2), l1 + l2 * math.cos(q2))
    q3 = z
    q = [q1, q2, q3]
    

    return q

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """
    X = [x, y, z]
    Epsilon_x = float('inf')
    threshold = 0.001

    while np.linalg.norm(Epsilon_x) > threshold:
        transformations = forward_kinematics(q)
        j = jacobian(q, transformations)
        X_hat = transformations[0] # Identity matrix at index 0
        for T in transformations[1:]:
            X_hat = X_hat @ T

        position_curr = X_hat[0:3, 3]
        rotation_curr = X_hat[0:3, 0:3]

        Epsilon_pos = point - position_curr

        R_err = R @ rotation_curr.T
        Epsilon_rotation = 0.5 * np.array([R_err[2, 1] - R_err[1, 2],
                                R_err[0, 2] - R_err[2, 0],
                                R_err[1, 0] - R_err[0, 1]])

        Epsilon_x = np.concatenate([Epsilon_pos, Epsilon_rotation])

        Epsilon_q = np.linalg.pinv(j) @ Epsilon_x
        q = q - Epsilon_q
    return q


def forward_kinematics(q):
    L = 0.4
    M = 0.39
    q1, q2, q3, q4, q5, q6, q7 = q
    DH_params = []
    DH_params.append([np.pi/2, 0, 0, q1])
    DH_params.append([-np.pi/2, 0, 0, q2])
    DH_params.append([-np.pi/2, L, 0, q3])
    DH_params.append([np.pi/2, 0, 0, q4])
    DH_params.append([np.pi/2, M, 0, q5])
    DH_params.append([-np.pi/2, 0, 0, q6])
    DH_params.append([0, 0, 0, q7])

    transformations = [np.eye(4)]
    for row in DH_params:
        transformations.append(DH_transformation(row))
    return transformations

def jacobian(q, transformations=None):
    if transformations is None:
        transformations = forward_kinematics(q)
    p_e = transformations[-1][0:3, 3]
    for i in range(7):
        z_i = transformations[i][0:3, 2]
        p_i = transformations[i][0:3, 3]
        v = np.cross(z_i, p_e - p_i) # linear velocity
        w = z_i # anfular velocity
        if i == 0:
            J = np.column_stack((v, w))
        else:
            J = np.column_stack((J, np.column_stack((v, w))))
    return J

def DH_transformation(DH_params_row):
    alpha, d, a, theta = DH_params_row
    T = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                  [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                  [0, np.sin(alpha), np.cos(alpha), d],
                  [0, 0, 0, 1]])
    return T
