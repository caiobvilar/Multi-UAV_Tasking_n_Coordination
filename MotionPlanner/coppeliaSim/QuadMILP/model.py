from gurobipy import *
import numpy as np
from math import *
from .RobotModel import RobotModel
from collections import defaultdict
from itertools import combinations
import logging
import re
logging.basicConfig(level=logging.ERROR)

setParam('OutputFlag', 0)
class State(object):
    def __init__(self,  V_MIN, V_MAX, F_MIN, F_MAX, T_MAX, N):
        self.model = Model("MultiQuads")
        x = [(i, t) for t in range(T_MAX) for i in range(N)]
        get_name = lambda x: ["{}_{}_{}".format(x, i, t) for t in range(T_MAX) for i in range(N)]

        self.x_i_t = self.model.addVars(x, vtype=GRB.CONTINUOUS, lb=-GRB.INFINITY, ub=GRB.INFINITY, name=get_name("x"))
        self.y_i_t = self.model.addVars(x, vtype=GRB.CONTINUOUS, lb=-GRB.INFINITY, ub=GRB.INFINITY, name=get_name("y"))
        self.Vx_i_t = self.model.addVars(x, vtype=GRB.CONTINUOUS, lb=V_MIN, ub=V_MAX, name=get_name("Vx"))
        self.Vy_i_t = self.model.addVars(x, vtype=GRB.CONTINUOUS, lb=V_MIN, ub=V_MAX, name=get_name("Vy"))
        self.Fx_i_t = self.model.addVars(x, vtype=GRB.CONTINUOUS, lb=F_MIN, ub=F_MAX, name=get_name("Fx"))
        self.Fy_i_t = self.model.addVars(x, vtype=GRB.CONTINUOUS, lb=F_MIN, ub=F_MAX, name=get_name("Fy"))
        self.Abs_Fx_i_t = self.model.addVars(x, vtype=GRB.CONTINUOUS, ub=F_MAX)
        self.Abs_Fy_i_t = self.model.addVars(x, vtype=GRB.CONTINUOUS, ub=F_MAX)

        self.b_i_t = self.model.addVars(x, vtype=GRB.BINARY, name=get_name("b"))
        c = [t for t in range(T_MAX)]
        self.T_t = self.model.addVars(c, vtype=GRB.CONTINUOUS, ub=GRB.INFINITY, name=["t_%d" % t for t in range(T_MAX)])



class ProblemDefinition(State):
    def __init__(self, T_MAX, N, V_MIN, V_MAX, F_MIN, F_MAX, dt ):
        super().__init__(V_MIN, V_MAX, F_MIN, F_MAX, T_MAX, N)
        epsilon = 0.01
        partA = quicksum(self.b_i_t[i, t] * self.T_t[t] for t in range(T_MAX) for i in range(N))
        partB = quicksum(self.Abs_Fx_i_t[i, t] + self.Abs_Fy_i_t[i, t] for t in range(T_MAX) for i in range(N))
        self.model.modelSense = GRB.MINIMIZE
        self.model.setObjective(partA + epsilon * partB)
        self.dt = dt
        self.V_MIN, self.V_MAX, self.F_MIN, self.F_MAX, self.T_MAX, self.N = V_MIN, V_MAX, F_MIN, F_MAX, T_MAX, N

    def add_force_constraints(self):
        M = 10
        for m in range(1, M + 1):
            for i in range(self.N):
                for t in range(self.T_MAX - 1):
                    self.model.addConstr(
                        self.Fx_i_t[i, t] * sin(2 * pi * m / M) + self.Fy_i_t[i, t] * cos(2 * pi * m / M) <= self.F_MAX)
                    self.model.addConstr(
                        self.Vx_i_t[i, t] * sin(2 * pi * m / M) + self.Vy_i_t[i, t] * cos(2 * pi * m / M) <= self.V_MAX)

    def intialization(self, robots):

        for i in range(self.N):
            self.model.addConstr(self.x_i_t[i, 0] == robots[i].x)
            self.model.addConstr(self.y_i_t[i, 0] == robots[i].y)
            self.model.addConstr(self.Vx_i_t[i, 0] == robots[i].vx)
            self.model.addConstr(self.Vy_i_t[i, 0] == robots[i].vy)
            self.model.addConstr(self.Fx_i_t[i, 0] == robots[i].fx)
            self.model.addConstr(self.Fy_i_t[i, 0] == robots[i].fy)

    def add_transitions(self, robots):
        N = len(robots)
        self.model.addConstr(self.T_t[self.T_MAX - 1] <= self.T_MAX)
        for i in range(N):
            A, B = robots[i].get_model()
            for t in range(self.T_MAX - 1):
                X = np.array([self.x_i_t[i, t], self.y_i_t[i, t], self.Vx_i_t[i, t], self.Vy_i_t[i, t]])
                U = np.array([self.Fx_i_t[i, t], self.Fy_i_t[i, t]])
                X = np.matmul(A, X) + np.matmul(B, U)
                self.model.addConstr(self.x_i_t[i, t + 1] == X[0])
                self.model.addConstr(self.y_i_t[i, t + 1] == X[1])
                self.model.addConstr(self.Vx_i_t[i, t + 1] == X[2])
                self.model.addConstr(self.Vy_i_t[i, t + 1] == X[3])
                self.model.addConstr(self.T_t[t + 1] == self.T_t[t] + robots[i].dt)
                self.model.addGenConstrAbs(self.Abs_Fx_i_t[i, t], self.Fx_i_t[i, t], "abs_fx_%d_%d" % (i, t))
                self.model.addGenConstrAbs(self.Abs_Fy_i_t[i, t], self.Fy_i_t[i, t], "abs_fy_%d_%d" % (i, t))


class Solver(ProblemDefinition):
    def __init__(self, robots):
        self.robots = robots
        N = len(robots)
        super().__init__(robots[0].T_MAX, N, robots[0].V_MIN, robots[0].V_MAX, robots[0].F_MIN, robots[0].F_MAX, robots[0].dt)


    def add_minimum_time_trajctory(self, goal):
        R = 1000000
        xi = 1e-6
        for i in range(self.N):
            self.model.addConstr(quicksum(self.b_i_t[i, t] for t in range(1, self.T_MAX)) == 1)
            self.model.addConstr(self.x_i_t[i, self.T_MAX - 1] == goal[i][0])
            self.model.addConstr(self.y_i_t[i, self.T_MAX - 1] == goal[i][1])

            for t in range(0, self.T_MAX):
                self.model.addConstr(self.x_i_t[i, t] - goal[i][0] <=  xi + R * (1 - self.b_i_t[i, t]))
                self.model.addConstr(self.x_i_t[i, t] - goal[i][0] >= -xi - R * (1 - self.b_i_t[i, t]))
                self.model.addConstr(self.y_i_t[i, t] - goal[i][1] <=  xi + R * (1 - self.b_i_t[i, t]))
                self.model.addConstr(self.y_i_t[i, t] - goal[i][1] >= -xi - R * (1 - self.b_i_t[i, t]))


    # def add_collision_avoidance(self):
    #     d = self.robots[0].radius
    #     R = 1000000
    #     q = [(t, i) for i in range(4) for t in range(self.T_MAX)]
    #     q_t_k = self.model.addVars(q, vtype=GRB.BINARY)
    #     for t in range(0, self.T_MAX):
    #         self.model.addConstr(self.x_i_t[0, t] - self.x_i_t[1, t] >= d - R * q_t_k[t, 0])
    #         self.model.addConstr(self.x_i_t[1, t] - self.x_i_t[0, t] >= d - R * q_t_k[t, 1])
    #         self.model.addConstr(self.y_i_t[0, t] - self.y_i_t[1, t] >= d - R * q_t_k[t, 2])
    #         self.model.addConstr(self.y_i_t[1, t] - self.y_i_t[0, t] >= d - R * q_t_k[t, 3])
    #         self.model.addConstr(quicksum(q_t_k[t, i] for i in range(4)) <= 3)

    def add_collision_avoidance(self):
        d = self.robots[0].radius
        R = 1000000

        combo = combinations(range(len(self.robots)), 2)
        # combo = filter(lambda x: self.robots[x[1]] - self.robots[x[0]] <= self.robots[x[0]].radius , combo)
        for item in combo:
            ii, jj = item
            # print(f"[+] collision constraints {ii}, {jj}")
            q = [(t, i) for i in range(4) for t in range(self.T_MAX)]
            q_t_k = self.model.addVars(q, vtype=GRB.BINARY)
            for t in range(0, self.T_MAX):
                self.model.addConstr(self.x_i_t[ii, t] - self.x_i_t[jj, t] >= d - R * q_t_k[t, 0])
                self.model.addConstr(self.x_i_t[jj, t] - self.x_i_t[ii, t] >= d - R * q_t_k[t, 1])
                self.model.addConstr(self.y_i_t[ii, t] - self.y_i_t[jj, t] >= d - R * q_t_k[t, 2])
                self.model.addConstr(self.y_i_t[jj, t] - self.y_i_t[ii, t] >= d - R * q_t_k[t, 3])
                self.model.addConstr(quicksum(q_t_k[t, i] for i in range(4)) <= 3)

    def evaluate(self):
        ''':parameter read gurobi model and update robot parameter '''
        data = defaultdict(list)
        param = ["x_", "y_", "Vx_", "Vy_", "Fx_", "Fy_", "b_"]
        if self.model.status != GRB.OPTIMAL:
            logging.error(f'[TrajPlanner] SOLUTION NOT FOUND !!! T_MAX = {self.T_MAX}')

            for i in range(self.N):
                self.robots[i].T_MAX = 50
                self.robots[i].count += 1
            return True
        for v in self.model.getVars():
            for p in param:
                for i in range(self.N):
                    key = "{}{}".format(p, i)
                    # print(key)
                    if re.search(key, v.varName):
                        data[key].append(v.x)
            if re.search("t_", v.varName):
                data["time"].append(v.x)

        update = []
        for i in range(self.N):
            traj = list(zip(data['x_%d' % i], data['y_%d' % i], data['Vx_%d' % i], data['Vy_%d' % i], data['Fx_%d' % i],
                            data['Fy_%d' % i]))
            self.robots[i].T_MAX = 10
            predicted_traj = np.array(traj)
            res = self.robots[i].update(predicted_traj[1], predicted_traj[1:,:])
            update.append(res)
        return all(update)

    def solve(self):
        goals = [next(self.robots[i]) for i in range(self.N)]
        self.intialization(self.robots)
        self.add_transitions(self.robots)
        self.add_minimum_time_trajctory(goals)
        self.add_collision_avoidance()
        self.model.optimize()
        return self.evaluate()

if __name__ == '__main__':
    MASS = 2.8
    T_MAX = 50
    V_MAX = 2.5 * 0.225  # m/s
    F_MAX = 2.5 * 0.294  # N
    V_MIN = -V_MAX
    F_MIN = - F_MAX
    dt = 1  # sampling time
    N = 2  # number of uavs
    epsilon = 0.000001
    R = 1000000
    SCALE = 10
    d = 1.5
    robot = [RobotModel(id=i, mass=MASS, V_MAX=V_MAX, F_MAX=F_MAX) for i in range(2)]
    solver = Solver(T_MAX, N, V_MIN, V_MAX, F_MIN, F_MAX, dt)

    problem = ([[0, 0], [10, 10]], [[10, 0], [0, 10]])
    solver.solve(robot, problem, problem)
