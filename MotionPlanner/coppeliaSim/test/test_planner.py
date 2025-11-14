import json 
import sys 
import os 
lib_path = "/home/hellscoffe/Development/Multi-UAV_Tasking_n_Coordination/MotionPlanner/coppeliaSim"
sys.path.append(lib_path)
from QuadMILP import Path, TrajManager


class QuadManager(TrajManager):
    def __init__(self):
        quadNames = ['/Quadcopter[%d]'%i for i in range(3)]
        targetNames = ['/target[%d]'%i for i in range(3)]
        self.quadHandles = [sim.getObject(quad) for quad in quadNames]
        self.targetHandles = [sim.getObject(target) for target in targetNames]
        self.altitude = self.getPositions()[0][-1]
    def getPositions(self):
        return [sim.getObjectPosition(handle, -1) for handle in self.quadHandles]
    def setInitPositions(self, point, index):
        pose = [point[0], point[1], self.altitude]
        sim.setObjectPosition(self.quadHandles[index], -1, pose)
        sim.setObjectPosition(self.targetHandles[index], -1, pose)
        
    def setPositions(self, point, index):
        pose = [point[0], point[1], self.altitude]
        sim.setObjectPosition(self.targetHandles[index], -1, pose)

def sysCall_thread():
    # e.g. non-synchronized loop:
    
    filename = 'test/area_decomposition_02.json'
    filename = os.path.join(lib_path, filename)
    
    with open(filename) as file:
        jdata = json.load(file)
    
    paths = []
    manager = QuadManager()
    for i in range(3):
        path = Path(index=i, data=jdata)
        path.interpolate(velocity=0.50)
        path.fit(-5, 5, -5, 5)
        # print(path)
        manager.setInitPositions(path[0], i)
        paths.append(path)
    delay = 0.01
    sim.setThreadAutomaticSwitch(True)
    for traj in manager.trajectory_planning(paths, delay):
        result = traj[0]
        print(result)
        manager.setInitPositions(result.positions, result.id)
        #sim.switchThread()
    sim.stopSimulation()
    # while True:
    #     p=sim.getObjectPosition(objHandle,-1)
    #     p[0]=p[0]+0.001
    #     sim.setObjectPosition(objHandle,-1,p)
        
    # e.g. synchronized loop:
    # sim.setThreadAutomaticSwitch(False)
    # while True:
    #     p=sim.getObjectPosition(objHandle,-1)
    #     p[0]=p[0]+0.001
    #     sim.setObjectPosition(objHandle,-1,p)
    #     sim.switchThread() # resume in next simulation step
    pass

# See the user manual or the available code snippets for additional callback functions and details

