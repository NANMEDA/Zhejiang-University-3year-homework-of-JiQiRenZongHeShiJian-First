import numpy as np
from FK import FK,euler_trans
from math import cos,sin,atan2,sqrt,pi
d7=120
d1=120
d3=100
d5=-200
d6=-100
d2=100
def IKsolve(T,t2):
    r11,r12,r13=T[0][0:3]
    r21,r22,r23=T[1][0:3]
    r31,r32,r33=T[2][0:3]
    px,py,pz=T[0][3],T[1][3],T[2][3]
    # t2=pi/4
    x=px*cos(t2)-d7*r13*cos(t2)
    y=py*cos(t2)-d7*r23*cos(t2)
    k=d3-d5+sin(t2)*(pz-d1)-d7*sin(t2)*r33
    r=sqrt(x**2+y**2)
    t1=atan2(sqrt(1-(k/r)**2),k/r)+atan2(y,x)
    t1_1=atan2(-sqrt(1-(k/r)**2),k/r)+atan2(y,x)
    m=IKsolve_1(T, t1, t2)+IKsolve_1(T,t1_1,t2)
    alpha=[0,-pi/2,pi/2,0,pi,pi/2,pi/2]
    a=[0,0,0,400,-400,0,0]
    d=[120,100,100,0,-200,-100,120]
    offset=[0,pi/2,pi,0,pi,pi/2,pi]
    k=[]
    for i in m:
        p,e,T1=FK(alpha,a,d,i,offset)
        if (np.abs(T1-T)<0.1).all():
            k.append(i)
    if len(k)==0:
        print("未找到逆运动学解")
    else:
        print("共找到%d个逆运动学解"%(len(k)))
    return k
## 计算t6,t7
def IKsolve_1(T,t1,t2):
    r11,r12,r13=T[0][0:3]
    r21,r22,r23=T[1][0:3]
    r31,r32,r33=T[2][0:3]
    px,py,pz=T[0][3],T[1][3],T[2][3]
    m=sin(t2)*r33-r13*cos(t2)*cos(t1)-r23*cos(t2)*sin(t1)
    t6=atan2(m,sqrt(1- m**2))
    t6_1=atan2(m,-sqrt(1-m**2))
    x=r11*cos(t1)*cos(t2)+r21*sin(t1)*cos(t2)-r31*sin(t2)
    y=r12*cos(t2)*cos(t1)+r22*cos(t2)*sin(t1)-r32*sin(t2)
    t7=atan2(-y,x)
    # t7=-atan2(y,x)
    # t7_1=pi-atan2(y,x)
    return Iksolve_2(T,t1,t2,t6,t7)+Iksolve_2(T,t1,t2,t6_1,t7)
## 计算t3+t4-t5,t4
def Iksolve_2(T,t1,t2,t6,t7):
    r11,r12,r13=T[0][0:3]
    r21,r22,r23=T[1][0:3]
    r31,r32,r33=T[2][0:3]
    px,py,pz=T[0][3],T[1][3],T[2][3]
    c345=-(sin(t7)*(r21*cos(t1)-r11*sin(t1))+cos(t7)*(r22*cos(t1)-r12*sin(t1)))
    if cos(t6)!=0:
        s345=(r23*cos(t1)-r13*sin(t1))/cos(t6)
        t345=atan2(s345,c345)
    else:
        s345=(r11*cos(t1)+r21*sin(t1))*sin(t2)*sin(t7)+r31*cos(t2)*sin(t7)+(r12*cos(t1)+r22*sin(t1))*cos(t7)*sin(t2)+r32*cos(t2)*cos(t7)
        s345=-s345
        t345 = atan2(s345, c345)
    # A=120*cos(t2)+100*cos(t7)*(r32*cos(t2)+r12*cos(t1)*sin(t2)+r22*sin(t1)*sin(t2))+100*sin(t7)*(r31*cos(t2)+r11*cos(t1)*sin(t2)+r21*sin(t1)*sin(t2))-pz*cos(t2)+120*r33*cos(t2)-px*cos(t1)*sin(t2)+120*r13*cos(t1)*sin(t2)-py*sin(t1)*sin(t2)+120*r23*sin(t1)*sin(t2)
    A= d1 * cos(t2) - pz * cos(t2) + d7 * r33 * cos(t2) - px * cos(t1) * sin(t2) - py * sin(t1) * sin(t2) + d7 * r23 * sin(
        t1) * sin(t2) - d6 * r32 * cos(t2) * cos(t7) + d7 * r13 * cos(t1) * sin(t2) - d6 * r31 * cos(t2) * sin(
        t7) - d6 * r12 * cos(t1) * cos(t7) * sin(t2) - d6 * r11 * cos(t1) * sin(t2) * sin(t7) - d6 * r22 * cos(
        t7) * sin(t1) * sin(t2) - d6 * r21 * sin(t1) * sin(t2) * sin(t7)
    A=A/400
    # B=py*cos(t1)-100*sin(t7)*(r21*cos(t1)-r11*sin(t1))-120*r23*cos(t1)-px*sin(t1)+120*r13*sin(t1)-100*cos(t7)*(r22*cos(t1)-r12*sin(t1))-100
    # B=B/400
    B=py*cos(t1) - d2 - px*sin(t1) - d7*r23*cos(t1) + d7*r13*sin(t1) - d6*r11*sin(t1)*sin(t7) + d6*r22*cos(t1)*cos(t7) - d6*r12*cos(t7)*sin(t1) + d6*r21*cos(t1)*sin(t7)
    B=B/400
    t4=atan2(sqrt(1-((2-A**2-B**2)/2)**2),(2-A**2-B**2)/2)
    t4_1=atan2(-sqrt(1-((2-A**2-B**2)/2)**2),(2-A**2-B**2)/2)
    return Iksolve_3(T,t1,t2,t4,t345,t6,t7,A,B)+Iksolve_3(T,t1,t2,t4_1,t345,t6,t7,A,B)
#计算t3,t5
def Iksolve_3(T,t1,t2,t4,t345,t6,t7,A,B):
    r11,r12,r13=T[0][0:3]
    r21,r22,r23=T[1][0:3]
    r31,r32,r33=T[2][0:3]
    px,py,pz=T[0][3],T[1][3],T[2][3]
    x=cos(t4)-1
    y=sin(t4)
    r=sqrt(x**2+y**2)
    fai=atan2(y,x)

    if t4 != 0.0 or t4 !=-0.0:
        if abs(B) > abs(r):
            r = np.sign(r) * abs(B)
        t3=atan2(B/r,sqrt(1-(B/r)**2))-fai
        t3_1=atan2(B/r,-sqrt(1-(B/r)**2))-fai
        t5=-(t345-t3-t4)
        t5_1=-(t345-t3_1-t4)
        u=np.array([t1,t2,t3,t4,t5,t6,t7])
        u1 = np.array([t1, t2, t3_1, t4, t5_1, t6, t7])
        # u=np.mod(u+2*pi,2*pi)
        # u1 = np.mod(u1 + 360, 360)
        return [u,u1]
    else:
        t3=t345/2
        t5=-t345/2
        u = np.array([t1, t2, t3, t4, t5, t6, t7])
        # u = np.mod(u+360,360)
        return [u]
    # u=np.mod(u+360,360)
    # u1 = np.mod(u1 + 360, 360)
    # else:

    return [u,u1]
if __name__=='__main__':
    alpha=[0,-pi/2,pi/2,0,pi,pi/2,pi/2]
    a=[0,0,0,400,-400,0,0]
    d=[120,100,100,0,-200,-100,120]
    offset=[0,pi/2,pi,0,pi,pi/2,pi]
    #依次输出旋转角度即可
    theta=[0,0,pi,0,0,0,0]
    position,euler,T=FK(alpha,a,d,theta,offset)
    np.set_printoptions(precision=3, suppress=True)
    # T=np.array([[1,0,0,600],
    #             [0,-1,0,0],
    #             [0,0,-1,0],
    #             [0,0,0,1]])
    # R=euler_trans(pi,0,0)
    # T[0][0:3]=R[0]
    # T[1][0:3]=R[1]
    # T[2][0:3]=R[2]
    T = np.array([[1, 0, 0, 600],
                  [0, -1, 0, 0],
                  [0, 0, -1, 0],
                  [0, 0, 0, 1]])
    T = np.array([[1, 0, 0, 400],
                  [0, 0, 1, -100],
                  [0, -1, 0, -100],
                  [0, 0, 0,1]])
    T.astype(np.float64)
    m=IKsolve(T)
    # for theta in m:
    #     position,euler,T=FK(alpha,a,d,theta,offset)
    #     print(T)
    #     position,euler,T=FK(alpha,a,d,i,offset)
    #     np.set_printoptions(precision=3, suppress=True)
    #     print(T)
