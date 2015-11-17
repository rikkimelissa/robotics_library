import pylab
import numpy as np
from math import pi
from functions import IKinBodyMat, JointTrajectory, FKinBody, IKinBody, CartesianTrajectory, TransToRp

L1 = .425
L2 = .392
W1 = .109
W2 = .082
H1 = .089
H2 = .095
M_ur5 = np.array([[1, 0, 0, -L1-L2],[0, 0, -1, -W1-W2],[0, 1, 0, H1-H2],[0,0,0,1]])
S1 = np.array([0,0,1,0,0,0])
S2 = np.array([0,-1,0,H1,0,0])
S3 = np.array([0,-1,0,H1,0,L1])
S4 = np.array([0,-1,0,H1,0,L1+L2])
S5 = np.array([0,0,-1,W1,-L1-L2,0])
S6 = np.array([0,-1,0,H1-H2,0,L1+L2])
screw_axes_ur5_S = np.vstack((S1,S2,S3,S4,S5,S6))
#joints = np.array([pi/4,pi/2,pi/7,-pi/8,-pi,.35])
#np.around(FKinFixed(M,screw_axes,joints),decimals=3)

B1 = np.array([0,1,0,W1+W2,0,L1+L2])
B2 = np.array([0,0,1,H2,-L1-L2,0])
B3 = np.array([0,0,1,H2,-L2,0])
B4 = np.array([0,0,1,H2,0,0])
B5 = np.array([0,-1,0,-W2,0,0])
B6 = np.array([0,0,1,0,0,0])
screw_axes_ur5_B = np.vstack((B1,B2,B3,B4,B5,B6))
#joints = np.array([0,0,0,0,0,0])
#np.around(FKinBody(M_ur5,screw_axes_ur5_B,joints),decimals=3)

# Problem 5
thStart = np.array([0.1,0.1,0.1,0.1,0.1,0.1])
thEnd = np.array([pi/2.,pi/2.,pi/2.,pi/2.,pi/2.,pi/2.])
thList3 = JointTrajectory(thStart, thEnd, 2, 101, 3)
thList5 = JointTrajectory(thStart, thEnd, 2, 101, 5)
tSpace = np.linspace(0,2,101)
pylab.plot(tSpace,thList3[:,0],'--b',label='Cubic')
pylab.plot(tSpace,thList5[:,0],'-r',label='Quintic')
pylab.xlabel('Time (s)')
pylab.ylabel('Joint position (rad)')
pylab.title('Joint Trajectory')
pylab.legend(loc='upper left')
pylab.show(block=False)

# Problem 6
Xstart = FKinBody(M_ur5,screw_axes_ur5_B,thStart)
Xend = FKinBody(M_ur5,screw_axes_ur5_B,thEnd)
'''
In [199]: Xstart
Out[199]: 
array([[ 0.92165131, -0.38799379,  0.00443663, -0.76385111],
       [-0.00735983, -0.02891248, -0.99955485, -0.26818803],
       [ 0.38794935,  0.92120839, -0.02950279, -0.12448378],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

In [200]: Xend
Out[200]: 
array([[ 0.   , -1.   ,  0.   ,  0.109],
       [ 1.   ,  0.   ,  0.   ,  0.297],
       [-0.   ,  0.   ,  1.   , -0.254],
       [ 0.   ,  0.   ,  0.   ,  1.   ]])
'''
Xlist = CartesianTrajectory(Xstart,Xend,2,101,5)
thlist = np.empty((101,6))
thlist[0] = thStart
pDlist = np.empty((100,3))
rDlist = np.empty((100,3,3))
pAlist = np.empty((100,3))
rAlist = np.empty((100,3,3))
for i in range(100):
    r, p = TransToRp(Xlist[i])
    pDlist[i] = p
    rDlist[i] = r
    thlist[i+1] = IKinBody(screw_axes_ur5_B,M_ur5,Xlist[i+1],thlist[i],.0017,.00001,100)
    x = FKinBody(M_ur5,screw_axes_ur5_B,thlist[i+1])
    ra, pa = TransToRp(x)
    pAlist[i] = pa
    rAlist[i] = ra
pylab.figure()
pylab.plot(tSpace,thlist[:,0],'-r',label='Joint 0')
pylab.plot(tSpace,thlist[:,1],'--b',label='Joint 1')
pylab.plot(tSpace,thlist[:,2],':g',label='Joint 2')
pylab.plot(tSpace,thlist[:,3],'-.m',label='Joint 3')
pylab.plot(tSpace,thlist[:,4],'.y',label='Joint 4')
pylab.plot(tSpace,thlist[:,5],'oc',label='Joint 5')
pylab.xlabel('Time (s)')
pylab.ylabel('Joint position (rad)')
pylab.title('Joint Trajectory')
pylab.legend(loc='upper left')
pylab.show(block=False)



