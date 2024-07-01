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

def reverse_joint(reserve,objects_list):
    for object in objects_list:
        sim.setObjectParent(object, -1)
    if reserve == False:
        objects_list.reverse()
    for i in range(len(objects_list) - 1):
        sim.setObjectParent(objects_list[i], objects_list[i + 1])

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

    while (t := sim.getSimulationTime()) < 3:
        sim.setJointPosition(objects_list[1], 2 * np.pi * t / 3)

        message = f'Simulation time: {t:.2f} s'
        print(message)
        sim.addLog(sim.verbosity_scriptinfos, message)
        client.step()  # triggers next simulation step
        time.sleep(0.01)

    # Change the parent
    reverse_joint(True, objects_list)

    while (t := sim.getSimulationTime()) < 60:
        sim.setJointPosition(objects_list[-2], 2 * np.pi * t / 3)

        message = f'Simulation time: {t:.2f} s'
        print(message)
        sim.addLog(sim.verbosity_scriptinfos, message)
        client.step()  # triggers next simulation step
        time.sleep(0.01)
    # Stop simulation
    sim.stopSimulation()

    # Restore the original idle loop frequency:
    sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)

    print('Program ended')