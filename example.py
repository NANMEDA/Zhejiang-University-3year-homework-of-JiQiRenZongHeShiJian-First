
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
        sim.setObjectParent(objects_list[i], objects_list[i + 1], True)


def get_position(reverse, objects_list):
    pos = np.zeros(7)
    if (reverse):
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


def set_joint(reverse, minus, objects_list, angle):
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

    '''t0 = sim.getSimulationTime()
    reverse_joint(True, objects_list)

    T = np.array([[1, 0, 0, -300],
                  [0, -1, 0, 0],
                  [0, 0, -1, 100],
                  [0, 0, 0, 1]])
    m = IKsolve(T, 0)
    theta0 = m[2]

    offset = [0, pi, -pi, 0, pi, -pi, 0]
    theta0 = theta0 + offset
    theta0[6] = -theta0[6]
    theta0[3] = -theta0[3]
    last01 = 5
    kArray = Planning(np.zeros(7), theta0, last01)'''

    '''
    while (t := sim.getSimulationTime() - t0) < last01:
        angle = Excute(kArray, t)
        set_joint(True,False,objects_list,angle)
        client.step()  # triggers next simulation step
        time.sleep(0.01)'''

    reverse_joint(True, objects_list)
    t_ = sim.getSimulationTime()

    '''T = np.array([[1, 0, 0, -300],
                  [0, -1, 0, 0],
                  [0, 0, -1, 0],
                  [0, 0, 0, 1]])
    m = IKsolve(T, 0)
    theta0 = m[2]

    offset = [0, pi, pi, 0, pi, pi, 0]
    theta0 = theta0 + offset
    theta0[6] = -theta0[6]
    theta0[3] = -theta0[3]
    last02 = 30
    kArray = Planning(get_position(True, objects_list), np.zeros(7), last02)
    while (t := sim.getSimulationTime() - t_) < last02:
        angle = Excute(kArray, t)
        set_joint(True, False, objects_list, angle)
        client.step()  # triggers next simulation step
        time.sleep(0.01)
    '''

    i = 0
    while (i < 100):
        set_joint(False, False, objects_list, [0, 0, pi * i / 200, 0, pi * i / 200, 0, 0])
        client.step()  # triggers next simulation step
        time.sleep(0.01)
        i = i + 1

    t0 = sim.getSimulationTime()

    last01 = 5
    d_theata = [2*pi, 0, pi/2, 0, pi/2, 0, -2*pi]
    while (t := sim.getSimulationTime() - t0) < last01:
        T = np.array([[1, 0, 0, -300],
                      [0, -1, 0, 0],
                      [0, 0, -1, t / last01 * 150],
                      [0, 0, 0, 1]])
        m = IKsolve(T, 0)
        theta0 = m[2]
        offset = [pi, 0, 0, 0, 0, 0, pi]
        theta0 = theta0 + offset
        theta0[6] = -theta0[6]
        theta0[3] = -theta0[3]
        sum = 0
        jump = False
        for i in range(7):
            sum += abs(theta0[i]-d_theata[i])
            if(sum>1):
                jump = True
                break
        if(jump):
            client.step()  # triggers next simulation step
            time.sleep(0.01)
            continue
        d_theata = theta0
        set_joint(True, False, objects_list, theta0)
        client.step()  # triggers next simulation step
        time.sleep(0.01)

    T = np.array([[1, 0, 0, 600],
                  [0, -1, 0, 0],
                  [0, 0, -1, 100],
                  [0, 0, 0, 1]])
    m = IKsolve(T, 0)
    theta0 = m[2]

    offset = [pi, 0, 0, 0, 0, 0, pi]
    theta0 = theta0 + offset
    theta0[6] = -theta0[6]
    theta0[3] = -theta0[3]
    last02 = 5
    kArray = Planning(get_position(True, objects_list), theta0, last02)
    while (t := sim.getSimulationTime() - t0 - last01) < last02:
        angle = Excute(kArray, t)
        set_joint(True, False, objects_list, angle)
        client.step()  # triggers next simulation step
        time.sleep(0.01)

    '''i = 0
    while (i < 100):
        set_joint(True, False, objects_list, get_position(True,objects_list) + [0, 0, 0, 0, 0, 0, -6*pi/100])
        client.step()  # triggers next simulation step
        time.sleep(0.01)
        i = i + 1'''

    t_oo = sim.getSimulationTime()
    while (t := sim.getSimulationTime() - t_oo) < 5:
        T = np.array([[1, 0, 0, 600],
                      [0, -1, 0, 0],
                      [0, 0, -1, 100 - t / 5 * 100],
                      [0, 0, 0, 1]])
        print(T)
        m = IKsolve(T, 0)
        theta0 = m[2]
        offset = [pi, 0, 0, 0, 0, 0, pi]
        theta0 = theta0 + offset
        theta0[6] = -theta0[6]
        theta0[3] = -theta0[3]
        set_joint(True, False, objects_list, theta0)
        client.step()  # triggers next simulation step
        time.sleep(0.01)

    T = np.array([[1, 0, 0, 600],
                  [0, -1, 0, 0],
                  [0, 0, -1, 0],
                  [0, 0, 0, 1]])
    m = IKsolve(T, 0)
    theta0 = m[2]
    offset = [pi, 0, 0, 0, 0, 0, pi]
    theta0 = theta0 + offset
    theta0[6] = -theta0[6]
    theta0[3] = -theta0[3]
    last02 = 5
    kArray = Planning(get_position(True, objects_list), theta0, last02)
    angle = Excute(kArray, 5)
    '''

    '''

    t1 = sim.getSimulationTime()

    reverse_joint(False, objects_list)
    for i in range(17, 31, 1):
        sim.setObjectMatrix(i, i - 1, k[i - 17])
    set_joint(True, False, objects_list, angle)
    client.step()  # triggers next simulation step
    time.sleep(0.01)


    while (t := sim.getSimulationTime() - t1) < 5:
        T = np.array([[1, 0, 0, -600],
                      [0, -1, 0, 0],
                      [0, 0, -1, 100 * t / 5],
                      [0, 0, 0, 1]])
        m = IKsolve(T, 0)
        theta1 = m[2]
        #if (theta1[0] == -2 * pi) or (theta1[0] == 2 * pi): theta1[0] = 0;
        #if (theta1[6] == -2 * pi) or (theta1[6] == 2 * pi): theta1[6] = 0;
        # offset = position#[pi, 0, 0, 0, 0, 0, pi]
        position = get_position(False, objects_list)
        kArray = Planning(position, theta1, 0.07)
        t_in = sim.getSimulationTime()
        d_angle = [ 2.07894903 , 0. ,-0.69182244 , 1.40543955 , 0.71361711 , 0.,  2.08354019]
        while (t := sim.getSimulationTime() - t_in) < 0.07:
            angle = Excute(kArray, t)

            sum = 0
            jump = False
            for i in range(7):
                sum += abs(angle[i] - d_angle[i])
                if sum > 1:
                    jump = True
                    break
            if jump:
                client.step()  # triggers next simulation step
                time.sleep(0.01)
                continue

            set_joint(False, False, objects_list, angle)
            client.step()  # triggers next simulation step
            time.sleep(0.01)
            d_angle = angle

    i = 0
    while (i < 100):
        set_joint(False, False, objects_list, angle + [pi * i / 200, 0, 0, 0, 0, 0, 0])
        client.step()  # triggers next simulation step
        time.sleep(0.01)
        i = i + 1

    t_3 = sim.getSimulationTime()

    T = np.array([[1, 0, 0, 400],
                  [0, 0, 1, -180],
                  [0, -1, 0, -80],
                  [0, 0, 0, 1]])

    m = IKsolve(T, 0)
    theta1 = m[2]
    last1 = 5

    position = get_position(False, objects_list)
    kArray = Planning(position, theta1, last1)
    while (t := sim.getSimulationTime() - t_3) < last1:
        angle = Excute(kArray, t)
        set_joint(False, False, objects_list, angle)
        client.step()  # triggers next simulation step
        time.sleep(0.01)

    while (t := sim.getSimulationTime() - t_3 - 5) < 5:
        T = np.array([[1, 0, 0, 400],
                      [0, 0, 1, -180 + 100 * t / 5],
                      [0, -1, 0, -80],
                      [0, 0, 0, 1]])
        m = IKsolve(T, 0)
        theta1 = m[2]
        # offset = position#[pi, 0, 0, 0, 0, 0, pi]
        position = get_position(False, objects_list)
        kArray = Planning(position, theta1, 0.1)
        t_in = sim.getSimulationTime()
        while (t := sim.getSimulationTime() - t_in) < 0.1:
            angle = Excute(kArray, t)
            set_joint(False, False, objects_list, angle)
            client.step()  # triggers next simulation step
            time.sleep(0.01)


    reverse_joint(True, objects_list)
    for i in range(29, 15, -1):
        sim.setObjectMatrix(i, i + 1, k1[29 - i])
    set_joint(False, True, objects_list, angle)
    '''
    last2 = 2
    position = get_position(True, objects_list)
    kArray = Planning(position, offset, last2)

    t2 = sim.getSimulationTime()
    while (t := sim.getSimulationTime() - t2) <= last2:
        angle = Excute(kArray, t)
        set_joint(True, False, objects_list, angle)
        client.step()  # triggers next simulation step
        time.sleep(0.01)

    
    while (t := sim.getSimulationTime() - t2-last2) <= 2:
        client.step()  # triggers next simulation step
        time.sleep(0.01)
    '''
    t_stop = sim.getSimulationTime()
    while (t := sim.getSimulationTime() - t_stop) <= 10:
        client.step()  # triggers next simulation step
        time.sleep(0.01)

    # Stop simulation
    sim.stopSimulation()

    # Restore the original idle loop frequency:
    sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)

    print('Program ended')
