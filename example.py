# Before running this example, make sure you have installed the following package.
# pip install coppeliasim-zmqremoteapi-client numpy
# You can find more information about ZeroMQ remote API 
# in the file path <Coppeliasim_install_path>\programming\zmqRemoteApi
# or on https://github.com/CoppeliaRobotics/zmqRemoteApi
#
# You can get more API about coppleliasim on https://coppeliarobotics.com/helpFiles/en/apiFunctions.htm

import time
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from math import pi
from IK import IKsolve
from trajplanning import Excute, Planning


def reverse_joint(reserve, objects_list_origin):
    objects_list = objects_list_origin.copy()
    for object in objects_list:
        sim.setObjectParent(object, -1)
    if reserve == False:
        objects_list.reverse()
    for i in range(len(objects_list) - 1):
        sim.setObjectParent(objects_list[i], objects_list[i + 1],True)


def get_position(reverse,objects_list):
    pos = np.zeros(7)
    if(reverse):
        pos[6] = sim.getJointPosition(objects_list[1])
        pos[5] = sim.getJointPosition(objects_list[3])
        pos[4] = sim.getJointPosition(objects_list[5])
        pos[3] = sim.getJointPosition(objects_list[7])
        pos[2] = sim.getJointPosition(objects_list[9])
        pos[1] = sim.getJointPosition(objects_list[11])
        pos[0] = sim.getJointPosition(objects_list[13])
    else:
        pos[0] = sim.getJointPosition(objects_list[1])
        pos[1] = sim.getJointPosition(objects_list[3])
        pos[2] = sim.getJointPosition(objects_list[5])
        pos[3] = sim.getJointPosition(objects_list[7])
        pos[4] = sim.getJointPosition(objects_list[9])
        pos[5] = sim.getJointPosition(objects_list[11])
        pos[6] = sim.getJointPosition(objects_list[13])
    return pos


def get_velocity(objects_list):
    vel = np.zeros(7)
    vel[0] = sim.getJointVelocity(objects_list[1])
    vel[1] = sim.getJointVelocity(objects_list[3])
    vel[2] = sim.getJointVelocity(objects_list[5])
    vel[3] = sim.getJointVelocity(objects_list[7])
    vel[4] = sim.getJointVelocity(objects_list[9])
    vel[5] = sim.getJointVelocity(objects_list[11])
    vel[6] = sim.getJointVelocity(objects_list[13])
    return vel

def set_joint(reverse,minus,objects_list,angle):
    if minus:
        angle = -angle
    if reverse:
        sim.setJointPosition(objects_list[1], angle[6])
        sim.setJointPosition(objects_list[3], angle[5])
        sim.setJointPosition(objects_list[5], angle[4])
        sim.setJointPosition(objects_list[7], angle[3])
        sim.setJointPosition(objects_list[9], angle[2])
        sim.setJointPosition(objects_list[11], angle[1])
        sim.setJointPosition(objects_list[13], angle[0])
    else:
        sim.setJointPosition(objects_list[1], angle[0])
        sim.setJointPosition(objects_list[3], angle[1])
        sim.setJointPosition(objects_list[5], angle[2])
        sim.setJointPosition(objects_list[7], angle[3])
        sim.setJointPosition(objects_list[9], angle[4])
        sim.setJointPosition(objects_list[11], angle[5])
        sim.setJointPosition(objects_list[13], angle[6])

def get_object(sim):
    # Get object handle
    l_base = sim.getObject('./L_Base')
    l_joint1 = sim.getObject('./L_Joint1')
    l_link1 = sim.getObject('./L_Link1')
    l_joint2 = sim.getObject('./L_Joint2')
    l_link2 = sim.getObject('./L_Link2')
    l_joint3 = sim.getObject('./L_Joint3')
    l_link3 = sim.getObject('./L_Link3')

    joint4 = sim.getObject('./Joint4')

    r_joint1 = sim.getObject('./R_Joint1')
    r_link1 = sim.getObject('./R_Link1')
    r_joint2 = sim.getObject('./R_Joint2')
    r_link2 = sim.getObject('./R_Link2')
    r_joint3 = sim.getObject('./R_Joint3')
    r_link3 = sim.getObject('./R_Link3')
    r_base = sim.getObject('./R_Base')

    objects_list = [
        l_base, l_joint1, l_link1, l_joint2, l_link2, l_joint3, l_link3,
        joint4,
        r_link3, r_joint3, r_link2, r_joint2, r_link1, r_joint1, r_base
    ]
    return objects_list


if __name__ == '__main__':

    print('Program started')

    client = RemoteAPIClient()
    sim = client.getObject('sim')

    objects_list = get_object(sim)
    # When simulation is not running, ZMQ message handling could be a bit
    # slow, since the idle loop runs at 8 Hz by default. So let's make
    # sure that the idle loop runs at full speed for this program:
    defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
    sim.setInt32Param(sim.intparam_idle_fps, 0)
    # Run a simulation in stepping mode:
    client.setStepping(True)
    sim.startSimulation()

    k = []
    for i in range(16, 30):
        k.append(sim.getObjectMatrix(i + 1, i))
    k1 = []
    for i in range(30, 16, -1):
        k1.append(sim.getObjectMatrix(i - 1, i))

    t0 = sim.getSimulationTime()
    reverse_joint(True, objects_list)

    T = np.array([[1, 0, 0, 600],
                  [0, -1, 0, 0],
                  [0, 0, -1, 0],
                  [0, 0, 0, 1]])
    m = IKsolve(T, 0)
    theta0 = m[2]

    offset = [0, pi, pi, 0, pi, pi, 0]
    theta0 = theta0 + offset
    theta0[6] = -theta0[6]
    theta0[3] = -theta0[3]
    last0 = 5
    kArray = Planning(np.zeros(7), theta0, last0)
    while (t := sim.getSimulationTime() - t0) < last0:
        angle = Excute(kArray, t)
        set_joint(True,False,objects_list,angle)
        client.step()  # triggers next simulation step
        time.sleep(0.01)


    t1 = sim.getSimulationTime()

    reverse_joint(False, objects_list)
    for i in range(17, 31, 1):
        sim.setObjectMatrix(i, i - 1, k[i - 17])
    set_joint(True,True,objects_list,angle)
    T = np.array([[1, 0, 0, 400],
                  [0, 0, 1, -100],
                  [0, -1, 0, -100],
                  [0, 0, 0, 1]])

    m = IKsolve(T, 0)
    theta1 = m[2]
    last1 = 5

    position = get_position(False,objects_list)
    kArray = Planning(position, theta1, last1)
    while (t := sim.getSimulationTime() - t1) < last1:
        angle = Excute(kArray, t)
        set_joint(False, False, objects_list, angle)
        client.step()  # triggers next simulation step
        time.sleep(0.01)



    reverse_joint(True, objects_list)
    for i in range(29,15,-1):
        sim.setObjectMatrix(i, i+1,k1[29-i])
    set_joint(False,True,objects_list,angle)
    last2 = 5
    position = get_position(True,objects_list)
    kArray = Planning(position, offset, last2)
    t2 = sim.getSimulationTime()
    while (t := sim.getSimulationTime() - t2) <= 5:
        angle = Excute(kArray, t)
        set_joint(True, False, objects_list, angle)
        client.step()  # triggers next simulation step
        time.sleep(0.01)

    # Stop simulation
    sim.stopSimulation()

    # Restore the original idle loop frequency:
    sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)

    print('Program ended')
