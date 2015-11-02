import numpy as np
from math import sin, cos, pi, tan, acos, sqrt
from functions import ScrewToAxis, MatrixExp6, MatrixLog6, FKinFixed, FKinBody

# Question 2
L1 = .425
L2 = .392
W1 = .109
W2 = .082
H1 = .089
H2 = .095
M = np.array([[1, 0, 0, -L1-L2],[0, 0, -1, -W1-W2],[0, 1, 0, H1-H2],[0,0,0,1]])
S1 = np.array([0,0,1,0,0,0])
S2 = np.array([0,-1,0,H1,0,0])
S3 = np.array([0,-1,0,H1,0,L1])
S4 = np.array([0,-1,0,H1,0,L1+L2])
S5 = np.array([0,0,-1,W1,-L1-L2,0])
S6 = np.array([0,-1,0,H1-H2,0,L1+L2])
screw_axesS = np.vstack((S1,S2,S3,S4,S5,S6))
#joints = np.array([pi/4,pi/2,pi/7,-pi/8,-pi,.35])
#np.around(FKinFixed(M,screw_axes,joints),decimals=3)

B1 = np.array([0,1,0,W1+W2,0,L1+L2])
B2 = np.array([0,0,1,H2,-L1-L2,0])
B3 = np.array([0,0,1,H2,-L2,0])
B4 = np.array([0,0,1,H2,0,0])
B5 = np.array([0,-1,0,-W2,0,0])
B6 = np.array([0,0,1,0,0,0])
screw_axesB = np.vstack((B1,B2,B3,B4,B5,B6))
#joints = np.array([pi/2,0,0,0,0,0])
#np.around(FKinBody(M,screw_axesB,joints),decimals=3)

# Question 4
Tsd = np.array([[0,1,0,-.6],[0,0,-1,.1],[-1,0,0,.1],[0,0,0,1]])
joints = np.array([0,0,0,0,0,0])
err_omega = .01
err_vel = .001
maxiterates = 100
IKinBody(screw_axesB,M,Tsd,joints,err_omega,err_vel,maxiterates)
Out[505]: 
array([[  0.   ,   0.   ,   0.   ,   0.   ,   0.   ,   0.   ],
       [  0.   ,  -0.254,  -1.775,  -0.162,   0.616,   0.   ],
       [ -0.343,   1.216,  -2.486,  -0.985,   0.219,   0.814],
       [ -0.65 ,   1.594,  -1.444,  -4.683,   0.429,   2.989],
       [ -0.224,   0.93 ,  -1.872,  -3.011,   0.046,   2.474],
       [ -0.642,   3.154,  -3.344,   6.43 ,   0.434,  -7.817],
       [  6.114,  19.399,  -1.728, -10.853,   6.108,  -8.455],
       [  5.712,  19.357,  -1.769, -12.417,   5.795,  -6.723],
       [  5.766,  19.387,  -1.544, -11.71 ,   6.048,  -7.692],
       [  5.81 ,  19.321,  -1.339, -11.541,   5.816,  -8.01 ],
       [  5.814,  19.341,  -1.395, -11.663,   5.82 ,  -7.854],
       [  5.814,  19.344,  -1.397, -11.664,   5.819,  -7.854]])

