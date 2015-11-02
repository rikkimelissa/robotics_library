import numpy as np
from math import sin, cos, pi, tan, acos, sqrt
from functions import ScrewToAxis, MatrixExp6, MatrixLog6, FKinFixed, FKinBody

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
screw_axes = np.vstack((S1,S2,S3,S4,S5,S6))
joints = np.array([pi/4,pi/2,pi/7,-pi/8,-pi,.35])
np.around(FKinFixed(M,screw_axes,joints),decimals=3)

B1 = np.array([0,1,0,W1+W2,0,L1+L2])
B2 = np.array([0,0,1,H2,-L1-L2,0])
B3 = np.array([0,0,1,H2,-L2,0])
B4 = np.array([0,0,1,H2,0,0])
B5 = np.array([0,-1,0,-W2,0,0])
B6 = np.array([0,0,1,0,0,0])
screw_axesB = np.vstack((B1,B2,B3,B4,B5,B6))
joints = np.array([pi/2,0,0,0,0,0])
np.around(FKinBody(M,screw_axesB,joints),decimals=3)

