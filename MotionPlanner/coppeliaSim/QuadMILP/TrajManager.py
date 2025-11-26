import math 
from time import sleep
from .model import Solver
from .RobotModel import RobotModel


MASS = 1.8 # kg 
V_MAX = 0.3375  # m/s
F_MAX = 0.441  # N
dt = 0.50   # sampling time
altitude = 1.0   # meter
safe_dist = 0.55   # meter


class State():
    id = 0
    positions = []
    velocities = []
    accerlerations = []
    effors = []
    time_from_start = 0
    def __repr__(self) -> str:
        return f"[robot {self.id}]: {self.positions}"

class TrajManager:
    def __init__(self, sim):
        super().__init__()
        self.sim = sim
        quadNames = ['/Quadcopter[%d]'%i for i in range(3)]
        targetNames = ['/target[%d]'%i for i in range(3)]
        self.quadHandles = [sim.getObject(quad) for quad in quadNames]
        self.targetHandles = [sim.getObject(target) for target in targetNames]
        self.altitude = self.getPositions()[0][-1]

    def get_yaw_angle(self, predicted_traj):
        first_point = predicted_traj[0]
        last_point = predicted_traj[-1]
        dx = last_point[0] - first_point[0]
        dy = last_point[1] - first_point[1]
        yaw = math.atan2(dy, dx)
        return yaw

    def gen_traj_msg(self, robot, seq):
        duration = 0
        yaw = self.get_yaw_angle(robot.predicted_traj)
        traj = []
        for state in robot.predicted_traj:
            s = State()
            s.id = seq 
            s.positions = state[0:2]
            s.velocities = state[2:4]
            s.accelerations = [f/robot.m for f in state[4:6]]
            s.effort = [yaw]
            duration += robot.dt
            s.time_from_start = duration
            traj.append(s)
        return traj

    def isRunning(self, robots):
        status = (robots[i].count < len(robots[i].waypoints) for i in range(len(robots)))
        return any(status)

    def trajectory_planning(self, all_paths, delay):
        print("Trajecotry planning ....")
        robots = []
        for i, wp in enumerate(all_paths):
            robot = RobotModel(id=i, dt= dt, mass=MASS, V_MAX=V_MAX, F_MAX=F_MAX, safe_dist=safe_dist)
            robot.add_waypoints(wp)
            robots.append(robot)
        print("Trajecotry following .... (%d)"%len(robots[0].waypoints))
        
        while self.isRunning(robots):
            try:
                sim_time = self.sim.getSimulationTime() if hasattr(self, "sim") and self.sim is not None else None
                solver = Solver(robots, sim_time=sim_time)
                if solver.solve():
                    for count, r in enumerate(robots):
                        traj = self.gen_traj_msg(r, count)
                        yield traj
                sleep(delay)
            except KeyboardInterrupt:
                break

