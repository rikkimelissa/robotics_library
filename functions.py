#!/usr/bin/env python

import numpy as np
from math import sin, cos, pi, tan, acos

'''
magnitude(x) takes in a numpy array and returns the scalar magnitude
Example usage:
x = np.array([3,-1,2])
magnitude(x)
Out: 3.7416573867739413
'''
def magnitude(x):
	return np.linalg.norm(x)

'''
normalize(x) takes in a numpy array and returns a unit numpy array
Example usage:
x = np.array([3,-1,2])
normalize(x)
Out[185]: array([ 0.80178373, -0.26726124,  0.53452248])
'''
def normalize(x):
	mag = magnitude(x)
	if mag==0:
		return x
	return x/mag

''' 
RotInv(x) takes in a rotation array in the form of a numpy array and returns its inverse
Example usage:
R = np.array([[1,0,0],[0,0,-1],[0,1,0]])
RotInv(R)
Out[187]: 
array([[ 1,  0,  0],
       [ 0,  0,  1],
       [ 0, -1,  0]])
'''
def RotInv(x):
	return x.T

'''
VecToso3(x) takes in a numpy array representing an angular velocity and returns the 3x3 skew-symmetric array
Example usage:
w = np.array([3,-1,2])
VecToso3(w)
Out[189]: 
array([[ 0, -2, -1],
       [ 2,  0, -3],
       [ 1,  3,  0]])
'''
def VecToso3(x):
	w1 = x[0]
	w2 = x[1]
	w3 = x[2]
	return np.array([[0, -w3, w2], [w3, 0,-w1], [-w2, w1, 0]])

''' 
so3ToVec(x) takes in a 3x3 skew-symmetric array of the form numpy array and returns the corresponding 3-vecotr as a numpy array
Example usage:
w_hat = np.array([[0,-2,-1],[2,0,-3],[1,3,0]])
so3ToVec(w_hat)
Out[193]: array([ 3, -1,  2])
'''
def so3ToVec(x):
	return np.array([x[2,1], x[0,2], x[1,0]])
	
''' 
AxisAng3(x) takes in a numpy array of the rotation vector and returns the unit rotation axis w and the rotation amount theta
Example usage:
w=np.array([0,0,1])
omega,theta = AxisAng3(w*pi/2)
omega
Out[203]: array([ 0.,  0.,  1.])
theta
Out[204]: 1.5707963267948966
'''
def AxisAng3(x):
    if np.all(x==0):
        theta = 0
        omega = x*0
    else:
	    theta = magnitude(x)
	    omega = x/theta
    return omega,theta

''' 
MatrixExp3(x) takes in a numpy array of exponential coordinates and returns the matrix exponential
Example usage:
w=np.array([0,0,1])
MatrixExp3(w*pi/2)
Out[242]: 
array([[  6.123e-17,  -1.000e+00,   0.000e+00],
       [  1.000e+00,   6.123e-17,   0.000e+00],
       [  0.000e+00,   0.000e+00,   1.000e+00]])
'''
def MatrixExp3(x):
	omega,th = AxisAng3(x)
	w1 = omega[0]
	w2 = omega[1]
	w3 = omega[2]
	return np.array([[cos(th)+w1**2*(1-cos(th)), w1*w2*(1-cos(th))-w3*sin(th), w1*w3*(1-cos(th))+w2*sin(th)], [w1*w2*(1-cos(th))+w3*sin(th), cos(th)+w2**2*(1-cos(th)), w2*w3*(1-cos(th))-w1*sin(th)], [w1*w3*(1-cos(th))-w2*sin(th), w2*w3*(1-cos(th))+w1*sin(th), cos(th)+w3**2*(1-cos(th))]])

''' 
MatrixLog3(x) takes in a matrix exponential of the form numpy array and returns a rotation vector as a numpy array
Example usage:
w = np.array([0,0,1])
a = MatrixExp3(w*pi/2)
 MatrixLog3(a)
Out[259]: array([ 0.   ,  0.   ,  1.571])
'''
def MatrixLog3(x):
    th = acos((x.trace()-1)/2)
    if (sin(th)==0):
        if (R==np.identity(3)):
            th = 0
            w1=w2=w3 = 0
        else:
            th = pi
            w1 = 1/sqrt(2*(1+x[2,2]))*x[0,2]
            w2 = 1/sqrt(2*(1+x[2,2]))*x[1,2]
            w3 = 1/sqrt(2*(1+x[2,2]))*(1+x[2,2])
    else:
        w1 = 1/(2*sin(th))*(x[2,1] - x[1,2])
        w2 = 1/(2*sin(th))*(x[0,2] - x[2,0])
        w3 = 1/(2*sin(th))*(x[1,0] - x[0,1])
    return th*np.array([w1, w2, w3])

''' 
RpToTrans(x) takes in a rotation matrix as a numpy array and a postion vector as a numpy array and returns a transformation matrix as a numpy array
Example usage:
R = np.array([[1,0,0],[0,0,-1],[0,1,0]])
pos = np.array([-3,1,2])
RpToTrans(rot,pos)
Out[262]: 
array([[ 1,  0,  0, -3],
       [ 0,  0, -1,  1],
       [ 0,  1,  0,  2],
       [ 0,  0,  0,  1]])
'''
def RpToTrans(rot,pos):
    trans = np.concatenate((rot,pos[np.newaxis].T), axis=1)
    c = ([[0,0,0,1]])
    trans = np.concatenate((trans,c),axis=0)
    return np.array(trans)
	
''' 
TransToRp(x) takes in a transformation matrix and returns a rotation matrix and a position vector as numpy arrays
Example usage:
R = np.array([[1,0,0],[0,0,-1],[0,1,0]])
pos = np.array([-3,1,2])
T = RpToTrans(rot,pos)
TransToRp(T)
Out[264]: 
(array([[ 1,  0,  0],
       [ 0,  0, -1],
       [ 0,  1,  0]]),
 array([-3,  1,  2]))
'''
def TransToRp(x):
    rot = x[0:3,0:3]
    pos = x[0:3,3]
    return rot, pos
    
''' 
TransInv(x) takes a transformation matrix as a numpy array and returns its inverse
Example usage:
R = np.array([[1,0,0],[0,0,-1],[0,1,0]])
pos = np.array([-3,1,2])
T = RpToTrans(rot,pos)
TransInv(T)
Out[267]: 
array([[ 1,  0,  0,  3],
       [ 0,  0,  1, -2],
       [ 0, -1,  0,  1],
       [ 0,  0,  0,  1]])
'''
def TransInv(x):
    rot, pos = TransToRp(x)
    rot_inv = rot.T
    pos_inv = -rot_inv.dot(pos)
    return RpToTrans(rot_inv, pos_inv)

''' 
VecTose3(x) takes in a spatial velocity in the form of 6x1 numpy array and returns the corresponding 4x4 matrix in the form of a numpy array
Example usage:
S = np.array([0,0,-1,-3,0,-.1])
VecTose3(S)
array([[ 0. ,  1. ,  0. , -3. ],
       [-1. ,  0. , -0. ,  0. ],
       [-0. ,  0. ,  0. , -0.1],
       [ 0. ,  0. ,  0. ,  1. ]])
'''
def VecTose3(x):
    w_mat = VecToso3(x[0:3]);
    pos = x[3:6]
    return RpToTrans(w_mat, pos)

''' 
se3ToVec(x) takes in an element of se(3) as a numpy array and returns the corresponding spatial velocity as a 6-element numpy array
Example usage:
S = np.array([0,0,-1,-3,0,-.1])
s = VecTose3(S)
se3ToVec(s)
Out[273]: array([ 0. ,  0. , -1. , -3. ,  0. , -0.1])
'''    
def se3ToVec(x):
    w_mat, vel = TransToRp(x)
    w = so3ToVec(w_mat)
    return np.concatenate((w,vel),axis=1)

''' 
Adjoint(x) takes in a transformation matrix as a numpy array and returns the 6x6 adjoint representation as a numpy array
Example usage:
R = np.array([[1,0,0],[0,0,-1],[0,1,0]])
pos = np.array([-3,1,2])
T = RpToTrans(rot,pos)
Adjoint(T)
Out[274]: 
array([[ 1.,  0.,  0.,  0.,  0.,  0.],
       [ 0.,  0., -1.,  0.,  0.,  0.],
       [ 0.,  1.,  0.,  0.,  0.,  0.],
       [ 0.,  1.,  2.,  1.,  0.,  0.],
       [ 2.,  3.,  0.,  0.,  0., -1.],
       [-1.,  0.,  3.,  0.,  1.,  0.]])
'''
def Adjoint(x):
    rot, pos = TransToRp(x)
    top = np.concatenate((rot,np.zeros((3,3))),axis = 1)
    p = VecToso3(pos)
    corner = p.dot(rot)
    bot = np.concatenate((corner,rot),axis = 1)
    return np.concatenate((top,bot))
    
''' 
ScrewToAxis(q,s,h) takes in a point q in R3, a unit axis s in R3 and a screw pitch and returns the screw axis as a numpy array
Example usage:
q=np.array([3,0,0])
s=np.array([0,0,1])
h=2
ScrewToAxis(q,s,h)
Out[279]: array([ 0,  0,  1,  0, -3,  2])
'''
def ScrewToAxis(q, s, h):
    v = np.cross(-s,q) + h*s
    return np.concatenate((s,v),axis=1)   
     

''' 
AxisAng6(x) takes a 6 vector of exponential coordinates for rigid body motion and returns the screw axis, S, and the distance traveled, theta
Example usage:
S = np.array([0, 0, 1, 2, 0, 0])
AxisAng6(S*pi/2)
Out[310]: (array([ 0.,  0.,  1.,  2.,  0.,  0.]), 1.5707963267948966)
'''
def AxisAng6(x):
    omega,theta = AxisAng3(x[0:3])
    if theta==0:
        theta=magnitude(x[3:6])
        if theta==0:
            v = x[3:6]
        else:
            v = x[3:6]/theta
    else:
        v = x[3:6]/theta
    return np.concatenate((omega,v),axis=1), theta 

''' 
MatrixExp6(x) takes a 6 vector of exponential coordinates and returns T' that is achieved by traveling along S a distance theta
Example usage:
S=np.array([0,1/sqrt(2),1/sqrt(2),1,2,3])
theta = 1
MatrixExp6(S*theta)
Out[282]: 
array([[  1.1e-16,   1.0e+00,   0.0e+00,  -3.0e+00],
       [ -1.0e+00,   1.1e-16,   0.0e+00,   3.0e+00],
       [  0.0e+00,   0.0e+00,   1.0e+00,  -1.6e-01],
       [  0.0e+00,   0.0e+00,   0.0e+00,   1.0e+00]])
'''
def MatrixExp6(x):
    S, th = AxisAng6(x)
    omega = S[0:3]
    v = S[3:6]
    omega_mat = VecToso3(omega)
#    if th==0:
#        v = x[3:6]/magnitude(x[3:6])
#    else:
#        v = x[3:6]/th
    I = np.identity(3)
    w1 = omega[0]
    w2 = omega[1]
    w3 = omega[2]
    eot = e_omega_theta(omega_mat,th)
    return e_s_theta(eot, omega_mat,th,v)
    
'''
e_omega_theta(omega_mat, th) takes in a skew symmetrix matrix and a distance traveled, theta, and returns the matrix exponential for rotations
Example usage:
omega_mat = np.array([[0,-2,-1],[2,0,-3],[1,3,0]])
th = pi/2
e_omega_theta(omega_mat, th)
Out[314]: 
array([[ -4.,  -5.,   5.],
       [ -1., -12.,  -5.],
       [  7.,   1.,  -9.]])
'''
def e_omega_theta(omega_mat,th):
    I = np.identity(3)
    return I+sin(th)*omega_mat + omega_mat.dot(omega_mat)*(1-cos(th))

'''
e_s_theta(eot, omega_mat, th) takes in a matrix exponential for rotations, a skew symmetrix matrix, a distance traveled theta, and a velocity vector, and returns the matrix exponential for transformations
Example usage:
omega_mat = np.array([[0,-2,-1],[2,0,-3],[1,3,0]])
th = pi/2
eot = e_omega_theta(omega_mat, th)
v = np.array([2, 0, 0])
e_s_theta(eot, omega_mat,th,v)
Out[321]: 
array([[ -4. ,  -5. ,   5. ,  -2.6],
       [ -1. , -12. ,  -5. ,   0.6],
       [  7. ,   1. ,  -9. ,   8.8],
       [  0. ,   0. ,   0. ,   1. ]])
'''
def e_s_theta(eot, omega_mat,th,v):
    I = np.identity(3)
    pos_no_v = (I*th + (1-cos(th))*omega_mat + (th-sin(th))*omega_mat.dot(omega_mat))
    pos = pos_no_v.dot(v)
    top = np.concatenate((eot,pos[np.newaxis].T),axis=1)
    bot = np.array([[0,0,0,1]])
    return np.concatenate((top,bot))
    

''' 
MatrixLog6(x) takes a transformation matrix and returns the corresponding 6-vector of exponential coordinates 
Example usage:  
T = np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,3],[0,0,0,1]]) 
MatrixLog6(T)
Out[284]: array([ 1.6,  0. ,  0. ,  0. ,  2.4,  2.4])
'''
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
        G_inv = (1/theta)*I - .5*omega_mat + (1/theta-.5/tan(theta/2))*omega_mat.dot(omega_mat)
        v = G_inv.dot(pos)
    else:
        theta = acos((np.trace(rot)-1)/2)
        omega_mat = 1/(2*sin(theta))*(rot-rot.T)
        omega = so3ToVec(omega_mat)
        G_inv = (1/theta)*I - .5*omega_mat + (1/theta-.5/tan(theta/2))*omega_mat.dot(omega_mat)
        v = G_inv.dot(pos)                
    return np.concatenate((omega,v),axis=1)*theta

''' 
FKinFixed takes M, the position and orientation of the end-effector frame when the manipulator is at its home position, a list of screw axes in the form of a numpy array matrix expressed in world frame, and a list of joint coordinates as a numpy array, and returns the transformation of the end-effector frame at the given joint angles
Example usage:
L0 = 2
L1 = 1
L2 = 1
L3 = .5
M = np.array([[-1,0,0,0],[0,1,0,L0+L2],[0,0,-1,L1-L3],[0,0,0,1]])
S1 = np.array([0,0,1,L0,0,0])
S2 = np.array([0,0,0,0,1,0])
S3 = np.array([0,0,-1,-L0-L2,0,-.1])
screw_axes = np.vstack((S1,S2,S3))
joints = ([pi/2,3,pi])
np.around(FKinFixed(M,screw_axes,joints),decimals=3)
Out[306]: 
array([[-0. ,  1. ,  0. , -1. ],
       [ 1. ,  0. ,  0. ,  2. ],
       [ 0. ,  0. , -1. ,  0.2],
       [ 0. ,  0. ,  0. ,  1. ]])
'''
def FKinFixed(M, screw_axes, joints):
    T = MatrixExp6(screw_axes[0]*joints[0])
    for S, theta in zip(screw_axes[1:],joints[1:]):
        T = T.dot(MatrixExp6(S*theta))           
    return T.dot(M)

''' 
FKinBody takes M, the position and orientation of the end-effector frame when the manipulator is at its home position, a list of screw axes in the form of a numpy array matrix expressed in body frame, and a list of joint coordinates as a numpy array, and returns the transformation of the end-effector frame at the given joint angles 
Example usage:
L0 = 2
L1 = 1
L2 = 1
L3 = .5
M = np.array([[-1,0,0,0],[0,1,0,L0+L2],[0,0,-1,L1-L3],[0,0,0,1]])
B1 = np.array([0,0,-1,L2,0,0])
B2 = np.array([0,0,0,0,1,0])
B3 = np.array([0,0,1,0,0,.1])
screw_axes = np.vstack((B1,B2,B3))
joints = ([pi/2,3,pi])
np.around(FKinBody(M,screw_axes,joints),decimals=3)
Out[295]: 
array([[-0. ,  1. ,  0. , -1. ],
       [ 1. ,  0. ,  0. ,  2. ],
       [ 0. ,  0. ,  0. ,  1. ]])
'''
def FKinBody(M, screw_axes, joints):
    T = M
    for S, theta in zip(screw_axes, joints):
        T = T.dot(MatrixExp6(S*theta))
    return T

'''    
FixedJacobian takes a set of screw axes and joint angles for a robot's joints expressed in the fixed space frame and returns the space Jacobian.
ss1 = np.array([[0,0,1,0,0,0],[0,0,1,0,-L1,0],[0,0,1,0,-L1-L2,0],[0,0,0,0,0,1]])
js1 = np.array([0,pi/2,0,0])
FixedJacobian(ss1,js1)
Out[207]: 
array([[ 0.,  0.,  0.,  0.],
       [ 0.,  0.,  0.,  0.],
       [ 1.,  1.,  1.,  0.],
       [ 0.,  0.,  2.,  0.],
       [ 0., -1., -1.,  0.],
       [ 0.,  0.,  0.,  1.]])
'''
def FixedJacobian(screw_axes, joints):
    J = np.array([screw_axes[0]])
    for i,S in enumerate(screw_axes[1:]):
        T = MatrixExp6(screw_axes[0]*joints[0])
        for S_next,th_next in zip(screw_axes[1:i+1],joints[1:i+1]):
            T = T.dot(MatrixExp6(S_next*th_next)) 
        V = Adjoint(T).dot(S)
        J = np.concatenate((J,[V]), axis=0)
    return J.T

'''
BodyJacobian takes a set of screw axes and joint angles for a robot's joints expressed in the body space frame and returns the body Jacobian.
ss1 = np.array([[0,0,1,0,0,0],[0,0,1,0,-L1,0],[0,0,1,0,-L1-L2,0],[0,0,0,0,0,1]])
js1 = np.array([0,pi/2,0,0])
BodyJacobian(ss1,js1)
Out[138]: 
array([[ 0.,  0.,  0.,  0.],
       [ 0.,  0.,  0.,  0.],
       [ 1.,  1.,  1.,  0.],
       [-1.,  0.,  0.,  0.],
       [-1., -1., -2.,  0.],
       [ 0.,  0.,  0.,  1.]])
'''
def BodyJacobian(screw_axes, joints):
    n = joints.size
    J = np.array([screw_axes[0]])
    for i,Bi in enumerate(screw_axes[:n-1]): 
        T = MatrixExp6(-screw_axes[n-1]*joints[n-1])
        for B, th in zip(reversed(screw_axes[i+1:n-1]),reversed(joints[i+1:n-1])):
            T = T.dot(MatrixExp6(-B*th))
        V = Adjoint(T).dot(Bi)    
        J = np.concatenate((J,[V]), axis=0)
    J = np.concatenate((J,[screw_axes[n-1]]),axis=0)
    J = np.delete(J,0,0)
    return J.T
    










