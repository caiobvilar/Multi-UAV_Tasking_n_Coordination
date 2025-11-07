#python
import json 
import sys 
import os 
lib_path = "/workspaces/Multi-UAV_Tasking_n_Coordination/MotionPlanner/coppeliaSim"
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
manager, motion = None, None  
def sysCall_init():
    # do some initialization here
    global manager, motion 
    filename = 'test/area_decomposition.json'
    with open(os.path.join(lib_path, filename)) as file:
        jdata = json.load(file)
    
    paths = []
    manager = QuadManager()
    for i in range(3):
        path = Path(index=i, data=jdata)
        path.fit(0, 100, -5, 5)
        path.interpolate(velocity=0.50)
        # print(path)
        manager.setInitPositions(path[0], i)
        paths.append(path)
    delay = 0.2
    motion = manager.trajectory_planning(paths, delay)


def sysCall_actuation():
    # put your actuation code here
    result = next(motion)[0]
    print(result)
    manager.setInitPositions(result.positions, result.id)

def sysCall_sensing():
    # put your sensing code here
    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details
