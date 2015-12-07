import pylab
import numpy as np
from math import pi
from functions import JointTrajectory, InverseDynamicsTrajectory, ForwardDynamicsTrajectory

M01 = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0.089159],[0,0,0,1]])
M12 = np.array([[0, 0, 1, 0.28],[0, 1, 0, 0.13585],[-1, 0, 0, 0],[0,0,0,1]])
M23 = np.array([[1, 0, 0, 0],[0, 1, 0, -0.1197],[0, 0, 1, 0.395],[0,0,0,1]])
M34 = np.array([[0, 0, 1, 0],[0, 1, 0, 0],[-1, 0, 0, 0.14225],[0,0,0,1]])
M45 = np.array([[1, 0, 0, 0],[0, 1, 0, 0.093],[0, 0, 1, 0],[0,0,0,1]])
M56 = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0.094565],[0,0,0,1]])
M67 = np.array([[1, 0, 0, 0],[0, 1, 0, .082],[0, 0, 1, 0],[0,0,0,1]])
M = [M01, M12, M23, M34, M45, M56, M67]
S1 = np.array([0,0,1,0,0,0])
S2 = np.array([0,1,0,-.089159,0,0])
S3 = np.array([0,1,0,-.089159,0,.425])
S4 = np.array([0,1,0,-.089159,0,.81725])
S5 = np.array([0,0,-1,-.10915,.81725,0])
S6 = np.array([0,1,0,.005491,0,.81725])
S = np.vstack((S1,S2,S3,S4,S5,S6))
m = 3.7; Ixx=0.01026749; Ixy = 0; Ixz = 0; Iyy = 0.01026749; Iyz = 0; Izz = 0.00666
G1 = np.array([[Ixx,Ixy,Ixz,0,0,0],[Ixy,Iyy,Iyz,0,0,0],[Ixz,Iyz,Izz,0,0,0],[0,0,0,m,0,0],[0,0,0,0,m,0],[0,0,0,0,0,m]])
m = 8.393; Ixx=0.22689; Iyy = Ixx; Izz = 0.0151
G2 = np.array([[Ixx,Ixy,Ixz,0,0,0],[Ixy,Iyy,Iyz,0,0,0],[Ixz,Iyz,Izz,0,0,0],[0,0,0,m,0,0],[0,0,0,0,m,0],[0,0,0,0,0,m]])
m = 2.275; Ixx=0.04944; Iyy = Ixx; Izz = 0.004095
G3 = np.array([[Ixx,Ixy,Ixz,0,0,0],[Ixy,Iyy,Iyz,0,0,0],[Ixz,Iyz,Izz,0,0,0],[0,0,0,m,0,0],[0,0,0,0,m,0],[0,0,0,0,0,m]])
m = 1.219; Ixx=0.1111727; Iyy = Ixx; Izz = 0.21942
G4 = np.array([[Ixx,Ixy,Ixz,0,0,0],[Ixy,Iyy,Iyz,0,0,0],[Ixz,Iyz,Izz,0,0,0],[0,0,0,m,0,0],[0,0,0,0,m,0],[0,0,0,0,0,m]])
m = 1.219; Ixx=0.1111727; Iyy = Ixx; Izz = 0.21942
G5 = np.array([[Ixx,Ixy,Ixz,0,0,0],[Ixy,Iyy,Iyz,0,0,0],[Ixz,Iyz,Izz,0,0,0],[0,0,0,m,0,0],[0,0,0,0,m,0],[0,0,0,0,0,m]])
m = .1879; Ixx=0.017136; Iyy = Ixx; Izz = 0.033822
G6 = np.array([[Ixx,Ixy,Ixz,0,0,0],[Ixy,Iyy,Iyz,0,0,0],[Ixz,Iyz,Izz,0,0,0],[0,0,0,m,0,0],[0,0,0,0,m,0],[0,0,0,0,0,m]])
G = [G1, G2, G3, G4, G5, G6]
Ftip = np.array([0,0,0,0,0,0])

pylab.close('all')
thStart = np.array([0,0,0,0,0,0])
thEnd = np.array([pi/2,pi/2,pi/2,pi/2,pi/2,pi/2])
T = 1
del_t = .001
N = T/del_t
tSpace = np.linspace(0,T,N)
thList = JointTrajectory(thStart, thEnd, T, N, 5)
pos = thList[:,0]
vel = np.append(0,np.diff(pos)/del_t)
acc = np.append(0,np.diff(vel)/del_t)
pylab.plot(tSpace,thList[:,0],'--b',label='position (rad)')
pylab.plot(tSpace,vel,'-r',label='velocity (rad/s)')
pylab.plot(tSpace,acc,':g',label='acceleration (rad/s^2)')
pylab.xlabel('Time (s)')
pylab.ylabel('Trajectory')
pylab.title('Joint Trajectory')
pylab.legend(loc='upper right')
pylab.show(block=False)

g = np.array([0,0,-9.8])
Ftips = np.zeros([N, 6])
velList = np.zeros([N,6])
accList = np.zeros([N,6])
for i in range(6):
    pos = thList[:,0]
    vel = np.append(0,np.diff(pos)/del_t)
    acc = np.append(0,np.diff(vel)/del_t)
    velList[:,i] = vel
    accList[:,i] = acc    
torques = InverseDynamicsTrajectory(thList, velList, accList, Ftips, g, M, G, S)

pylab.figure()
pylab.plot(tSpace,torques[:,0],'-r',label='Joint 0')
pylab.plot(tSpace,torques[:,1],'--b',label='Joint 1')
pylab.plot(tSpace,torques[:,2],':g',label='Joint 2')
pylab.plot(tSpace,torques[:,3],'-.m',label='Joint 3')
pylab.plot(tSpace,torques[:,4],'.y',label='Joint 4')
pylab.plot(tSpace,torques[:,5],'oc',label='Joint 5')
pylab.xlabel('Time (s)')
pylab.ylabel('Torque (n*m)')
pylab.title('Desired torque')
pylab.legend(loc='upper left')
pylab.show(block=False)

x,y = ForwardDynamicsTrajectory(thStart, thStart, torques, del_t, g, Ftip, M, G, S)

pylab.figure()
#tSpace = np.linspace(0,T,N)
pylab.plot(tSpace,x[:,2],'--b',label='position (rad)')
#pylab.plot(tSpace,y[:,0],'-r',label='velocity (rad/s)')
#pylab.plot(x[:,0])
pylab.show(block=False)

