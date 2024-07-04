import numpy as np
from math import sin,cos,pi
import math
def cal(alpha,a,d,theta):
    m=np.array([[cos(theta),-sin(theta),0,a],
    [sin(theta)*cos(alpha),cos(theta)*cos(alpha),-sin(alpha),-sin(alpha)*d],
    [sin(theta)*sin(alpha),cos(theta)*sin(alpha),cos(alpha),cos(alpha)*d],
    [0,0,0,1]])
    return m
def euler_angles(R):
    alpha=math.atan2(-R[ 1][ 2],R[2][2])
    beta=math.asin(R[0][ 2])
    gamma=math.atan2(-R[ 0][ 1],R[0][0])
    return np.array([alpha,beta,gamma])
def euler_trans(alpha,beta,gamma):
    R=np.eye(3)
    R1=np.array([[1,0,0],
                 [0,cos(alpha),-sin(alpha)],
                 [0,sin(alpha),cos(alpha)]])
    R2=np.array([[cos(beta),0,sin(beta)],
                 [0,1,0],
                 [-sin(beta),0,cos(beta)]])
    R3=np.array([[cos(gamma),-sin(gamma),0],
                 [sin(gamma),cos(gamma),0],
                 [0,0,1]])
    R=np.matmul(R1,R2)
    R=np.matmul(R,R3)
    return R
def FK(alpha,a,d,theta,offset):
    m = np.eye(4)
    for i in range(len(a)):
        m = np.matmul(m, cal(alpha[i], a[i], d[i], theta[i]+offset[i]))
    position=m[0:3,3].T
    # print(m)
    euler=euler_angles(m)/pi*180
    return position,euler,m
# alpha=[0,-pi/2,pi/2,0,0,pi/2,pi/2]
# a=[0,0,0,-400,400,0,0]
# d=[120,100,100,0,200,100,120]
# theta=[0,pi/2,0,0,0,pi/2,0]
# alpha=[0,-pi/2,pi/2,0,0,-pi/2,pi/2]
# a=[0,0,0,400,-400,0,0]
# d=[120,100,100,0,200,100,120]
# offset=[0,pi/2,pi,0,0,-pi/2,0]
if __name__=='__main__':
    alpha=[0,-pi/2,pi/2,0,pi,pi/2,pi/2]
    a=[0,0,0,400,-400,0,0]
    d=[120,100,100,0,-200,-100,85]
    offset=[0,pi/2,pi,0,pi,pi/2,pi]
    #依次输出旋转角度即可
    theta=[pi,pi/2,pi/2,0,pi/2,pi/4,pi/4]
    theta=[pi/3,pi/5,pi/6,0,0,0,0]
    position,euler,T=FK(alpha,a,d,theta,offset)
    np.set_printoptions(precision=3, suppress=True)
    print("位置",position+np.array([650,0,200]))
    print("欧拉角",euler)
    R=euler_trans(0,0,pi)
    print(R)