#!/usr/bin/env python

import numpy as np
from math import sin, cos, pi, tan, acos

# magnitude(x) takes in a numpy array and returns the scalar magnitude
def magnitude(x):
	return np.linalg.norm(x)

# normalize(x) takes in a numpy array and returns a unit numpy array
def normalize(x):
	mag = magnitude(x)
	if mag==0:
		return x
	return x/mag

# RotInv(x) takes in a rotation array in the form of a numpy array and returns its inverse
def RotInv(x):
	return x.T

# VecToso3(x) takes in a numpy array representing an angular velocity and returns the 3x3 skew-symmetric array
def VecToso3(x):
	w1 = x[0]
	w2 = x[1]
	w3 = x[2]
	return np.array([[0, -w3, w2], [w3, 0,-w1], [-w2, w1, 0]])

# so3ToVec(x) takes in a 3x3 skew-symmetric array of the form numpy array and returns the corresponding 3-vecotr as a numpy array
def so3ToVec(x):
	return np.array([x[2,1], x[0,2], x[1,0]])
	
# AxisAng3(x) takes in a numpy array of the rotation vecotr and returns the unit rotation axis w and the rotation amount theta
def AxisAng3(x):
	theta = magnitude(x)
	omega = x/theta
	return omega, theta

# MatrixExp3(x) takes in a numpy array of exponential coordinates and returns the matrix exponential
def MatrixExp3(x):
	omega,theta = AxisAng3(x)
	w1 = wvec[0]
	w2 = wvec[1]
	w3 = wvec[2]
	return np.array([[cos(th)+w1^2(1-cos(th)), w1*w2*(1-cos(th))-w3*sin(th), w1*w3*(1-cos(th))+w2*sin(th)], [w1*w2*(1-cos(th))+w3*sin(th), cos(th)+w2^2*(1-cos(th)), w2*w3*(1-cos(th))-w1*sin(th)], [w1*w3*(1-cos(th))-w2*sin(th), w2*w3*(1-cos(th))+w1*sin(th), cos(th)+w3^2*(1-cos(th))]])

# MatrixLog3(x) takes in a matrix exponential of the form numpy array and returns a rotation vector as a numpy array
def MatrixLog3(x):
    th = acos((x.trace()-1)/2)
    w1 = 1/(2*sin(th))*(x[2,1] - x[1,2])
    w2 = 1/(2*sin(th))*(x[0,2] - x[2,0])
    w3 = 1/(2*sin(th))*(x[1,0] - x[0,1])
    return np.array([w1, w2, w3])

# RpToTrans(x) takes in a rotation matrix as a numpy array and a postion vector as a numpy array and returns a transformation matrix as a numpy array
def RpToTrans(rot,pos):
    trans = np.concatenate((rot,pos[np.newaxis].T), axis=1)
    c = ([[0,0,0,1]])
    trans = np.concatenate((trans,c),axis=0)
    return np.array(trans)
	
# TransToRp(x) takes in a transformation matrix and returns a rotation matrix and a position vector as numpy arrays
def TransToRp(x):
    rot = x[0:3,0:3]
    pos = x[0:3,3]
    return rot, pos
    
# TransInv(x) takes a transformation matrix as a numpy array and returns its inverse
def TransInv(x):
    rot, pos = TransToRp(x)
    rot_inv = rot.T
    pos_inv = -rot_inv.dot(pos)
    return RpToTrans(rot_inv, pos_inv)

# VecTose3(x) takes in a spatial velocity in the form of 6x1 numpy array and returns the corresponding 4x4 matrix in the form of a numpy array
def VecTose3(x):
    w_mat = VecToso3(x[0:3]);
    pos = x[3:6]
    return RpToTrans(w_mat, pos)

# se3ToVec(x) takes in an element of se(3) as a numpy array and returns the corresponding spatial velocity as a 6-element numpy array    
def se3ToVec(x):
    w_mat, vel = TransToRp(x)
    w = so3ToVec(w_mat)
    return np.concatenate((w,vel),axis=1)

# Adjoint(x) takes in a transformation matrix as a numpy array and returns the 6x6 adjoint representation as a numpy array
def Adjoint(x):
    rot, pos = TransToRp(x)
    top = np.concatenate((rot,np.zeros((3,3))),axis = 1)
    p = VecToso3(pos)
    corner = p.dot(rot)
    bot = np.concatenate((corner,rot),axis = 1)
    return np.concatenate((top,bot))
    
# ScrewToAxis(q,s,h) takes in a point q in R3, a unit axis s in R3 and a screw pitch and returns the screw axis as a numpy array
def ScrewToAxis(q, s, h):
    v = np.cross(-s,q) + h*s
    return np.concatenate((s,v),axis=1)    

# AxisAng6(x) takes a 6 vector of exponential coordinates for rigid body motion and returns the screw axis, S, and the distance traveled, theta
def AxisAng6(x):
    omega,theta = AxisAng3(x[0:3])
    v = x[3:6]/theta
    return np.concatenate((omega,v),axis=1), theta 

# MatrixExp6(x) takes a 6 vector of exponential coordinates and returns T' that is achieved by traveling along S a distance theta
def MatrixExp6(x):
    omega, th = AxisAng3(x[0:3])
    omega_mat = VecToso3(omega)
    v = x[3:6]/th
    I = np.identity(3)
    w1 = omega[0]
    w2 = omega[1]
    w3 = omega[2]
    eot = e_omega_theta(omega_mat,th)
    corner = e_s_theta(omega_mat,th,v)
    top = np.concatenate((eot,corner[np.newaxis].T),axis=1)
    bot = np.array([[0,0,0,1]])
    return np.concatenate((top,bot))
    
def e_omega_theta(omega_mat,th):
    I = np.identity(3)
    return I+sin(th)*omega_mat + omega_mat.dot(omega_mat)*(1-cos(th))
    
def e_s_theta(omega_mat,th,v):
    I = np.identity(3)
    x = (I*th + (1-cos(th))*omega_mat + (th-sin(th))*omega_mat.dot(omega_mat))
    return x.dot(v)

# MatrixLog6(x) takes a transformation matrix and returns the corresponding 6-vector of exponential coordinates    
def MatrixLog6(x):
    rot, pos = TransToRp(x)
    I = np.identity(3)
    if np.all(I==rot):
        omega = np.array([0,0,0])
        v = pos/magnitude(pos)
        theta = magnitude(pos)
    elif (np.trace(rot)==-1):
        theta = pi
        omega = MatrixLog3(rot)
        omega_mat = VecToso3(omega)
        G_inv = 1/theta*I-1/2*omega_mat+(1/theta-1/2*1/tan(theta/2))*omega_mat.dot(omega_mat)
        v = G_inv.dot(pos)
    else:
        theta = acos((np.trace(rot)-1)/2)
        omega_mat = 1/(2*sin(theta))*(rot-rot.T)
        omega = so3ToVec(omega_mat)
        G_inv = 1/theta*I-1/2*omega_mat+(1/theta-1/2*1/tan(theta/2))*omega_mat.dot(omega_mat)
        v = G_inv.dot(pos)        
        
    return np.concatenate((omega,v),axis=1)*theta

def FKinFixed(M, screw_axes, joints):
    T = np.array()
    for axis, joint in screw_axes, joints:
    # what does e^[s]*th actually mean?
        T = T.dot(e)
    return T.dot(M)
    
def FKinBody(M, screw_axes, joints):
    x=1
    












