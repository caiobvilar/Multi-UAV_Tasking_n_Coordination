# Multi-UAV Coordination and Control for Smart Agriculture 
This repository contains codes for reproducing the results obtained in the paper https://dl.acm.org/doi/pdf/10.1145/3749194

## Overview 
For monitoring of a farm land using multi-UAV platform, first, an area of interest is captured as a convex polygon and decomposed into several partitions to assign them to different UAVs based on their flying and sensing capabilities. 
Then, the assigned areas are covered generating the back and fourth sweeping paths. Finally, a novel Mixed Integer Linear Programming (MILP) based optimum motion planning technique is developed to automatically generate collision free trajectories along these paths while minimizing the overall mission time and energy consumption subject to the physical constraints of UAVs as well as data collection requirements. 

## Depndencies

* Ubuntu (18.04 or higher)
* CMake 
* Python
* Gurobi
* CoppeliaSim


