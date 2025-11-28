# planner_external.py

import os
import json
import time
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(HERE)     # /.../MotionPlanner/coppeliaSim
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from QuadMILP import Path, TrajManager
from coppeliasim_zmqremoteapi_client import RemoteAPIClient  # pip install coppeliasim-zmqremoteapi-client

# adjust this path for your project, same as you use today:
LIB_PATH = "/home/hellscoffe/Development/Multi-UAV_Tasking_n_Coordination/MotionPlanner/coppeliaSim"
JSON_REL_PATH = "test/area_decomposition_02.json"


class QuadManager(TrajManager):
    def __init__(self, sim):
        self.sim = sim
        quadNames   = [f"/Quadcopter[{i}]" for i in range(3)]
        # child target under each quad:
        targetNames = [f"/Quadcopter[{i}]/target" for i in range(3)]

        self.quadHandles   = [self.sim.getObject(q) for q in quadNames]
        self.targetHandles = [self.sim.getObject(t) for t in targetNames]
        self.altitude = self.getPositions()[0][-1]

    def getPositions(self):
        return [self.sim.getObjectPosition(h, -1) for h in self.quadHandles]

    def setInitPositions(self, point, index):
        pose = [point[0], point[1], self.altitude]
        self.sim.setObjectPosition(self.quadHandles[index], -1, pose)
        self.sim.setObjectPosition(self.targetHandles[index], -1, pose)

    def setPositions(self, point, index):
        pose = [point[0], point[1], self.altitude]
        self.sim.setObjectPosition(self.targetHandles[index], -1, pose)


def quad_error(q_pos, t_pos):
    # q_pos and t_pos are [x, y, z] lists from sim.getObjectPosition
    dx = q_pos[0] - t_pos[0]
    dy = q_pos[1] - t_pos[1]
    dz = q_pos[2] - t_pos[2]
    return dx*dx + dy*dy + dz*dz

def main():
    print("A: creating client")
    client = RemoteAPIClient()
    print("B: requiring sim")
    sim = client.require('sim')
    print("C: connected to sim")

    sim.setStepping(True)
    sim.startSimulation()
    print("D: simulation started")

    filename = os.path.join(LIB_PATH, JSON_REL_PATH)
    print("E: loading JSON from", filename)
    with open(filename) as f:
        jdata = json.load(f)
    print("F: JSON loaded")

    manager = QuadManager(sim)
    print("G: QuadManager created")
    paths = []

    for i in range(3):
        path = Path(index=i, data=jdata)
        path.interpolate(velocity=0.50)
        path.fit(-5, 5, -5, 5)
        manager.setInitPositions(path[0], i)
        paths.append(path)
    print("H: built", len(paths), "paths")

    delay = 0.01
    step_count = 0
    print("I: entering trajectory_planning loop")
    for traj in manager.trajectory_planning(paths, delay):
        step_count += 1
        result = traj[0]
        print("STEP", step_count, "id", result.id, "pos", result.positions)
        manager.setPositions(result.positions, result.id)
        q0_pos = sim.getObjectPosition(manager.quadHandles[0], -1)
        t0_pos = sim.getObjectPosition(manager.targetHandles[0], -1)
        print("STEP", step_count,
              "Q0", q0_pos,
              "T0", t0_pos)
        sim.step()
        time.sleep(delay)

    print("J: loop finished, steps:", step_count)
    sim.stopSimulation()

if __name__ == "__main__":
    try:
        print(">>> starting planner_external.py")
        main()
        print(">>> main() finished OK")
    except Exception as e:
        import traceback
        print(">>> EXCEPTION in main():", e)
        traceback.print_exc()

