import numpy as np
from math import *

W_sh = 0.015
L_sh = 0.0325
W_b = W_sh + 2*L_sh
L_b = 0.127


def bodyIK(omega,phi,psi,xm,ym,zm):

    """
    omega = body x rotation pitch
    phi = body  y rotation roll
    psi = body z rotation yaw
    center = center of the robot body 
    """

    """
    Calculate the four Transformation-Matrices for our Legs
    Coordinate axes are consistent with MuJoCo

    Rx=X-Axis Rotation Matrix
    Ry=Y-Axis Rotation Matrix
    Rz=Z-Axis Rotation Matrix
    Rxyz=All Axis Rotation Matrix
    T=Translation Matrix
    Tm=Transformation Matrix
    Tlf,Trf,Tlh,Trh=final Matrix for LeftFore,RightFore,LeftHind and RightHind
    """

    omega = np.deg2rad(omega)
    phi = np.deg2rad(phi)
    psi = np.deg2rad(psi)

    Rx = np.array([
        [1, 0, 0, 0], 
        [0, np.cos(omega), -np.sin(omega), 0],
        [0,np.sin(omega),np.cos(omega),0],
        [0,0,0,1]])

    Ry = np.array([
        [np.cos(phi),0, np.sin(phi), 0], 
        [0, 1, 0, 0],
        [-np.sin(phi),0, np.cos(phi),0],
        [0,0,0,1]])

    Rz = np.array([
        [np.cos(psi),-np.sin(psi), 0,0], 
        [np.sin(psi),np.cos(psi),0,0],
        [0,0,1,0],
        [0,0,0,1]])

    Rxyz=Rx@Ry@Rz

    T = np.array([[0,0,0,xm],[0,0,0,ym],[0,0,0,zm],[0,0,0,0]])
    Tm = T+Rxyz

    Tlf = Tm @ np.array([
        [1,0,0, W_sh/2],
        [0,1,0, -L_b/2],
        [0,0,1, 0],
        [0,0,0,1]])
    
    Trf = Tm @ np.array([
        [1,0,0, -W_sh/2],
        [0,1,0, -L_b/2],
        [0,0,1, 0],
        [0,0,0,1]])

    Tlh = Tm @ np.array([
        [1,0,0, W_sh/2],
        [0,1,0, L_b/2],
        [0,0,1, 0],
        [0,0,0,1]])
    
    Trh = Tm @ np.array([
        [1,0,0, -W_sh/2],
        [0,1,0, L_b/2],
        [0,0,1, 0],
        [0,0,0,1]])

    return (Tlf,Trf,Tlh,Trh,Tm)