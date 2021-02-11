# Coordinated Motion Planning

This repository contains code investigating the [coordinated motion planning challenge](https://cgshop.ibr.cs.tu-bs.de/competition/cg-shop-2021/#problem-description), part of CG Week 2021.


Code Structure:
The Julia folder contains the most up-to-date code.
- `priority_planning`: contains an implementation of a "priority planning" approach, in which robot paths are planned one at a time
- `astar`: contains an implementation of the A* algorithm for finding a path for a single robot
- `robot_visualization`: contains utility visualization functions
