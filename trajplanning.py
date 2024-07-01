import math
import numpy as np
# 两点间五次多项式轨迹规划，返回值kArray为多项式系数
def Planning(startPosition, endPosition, time):
    timeMatrix = np.matrix([
        [         0,           0,             0,          0,        0,   1],
        [   time**5,     time**4,       time**3,    time**2,     time,   1],
        [         0,           0,             0,          0,        1,   0],
        [ 5*time**4,   4*time**3,     3*time**2,     2*time,        1,   0],
        [         0,           0,             0,          2,        0,   0],
        [20*time**3,  12*time**2,        6*time,          2,        0,   0]

    ])
    invTimeMatrix = np.linalg.inv(timeMatrix)
    kArray = []
    for i in range(len(startPosition)):
        X = np.matrix([startPosition[i], endPosition[i],0,0,0,0]).T
        k = np.dot(invTimeMatrix,X)
        kArray.append(k)
    return kArray

# 两点间规划，带入当前时间time，求得当前时刻下的关节位置值
def Excute(kArray, time):
    timeVector = np.matrix([time**5,     time**4,       time**3,    time**2,     time,   1]).T
    jointPositions = []
    for i in range(7):
        jointPosition = np.dot(kArray[i].T,timeVector)
        jointPositions.append(jointPosition[0,0])
    return np.array(jointPositions)