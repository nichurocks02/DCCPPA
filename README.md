# DCCPPA
Optimized Path planning algorithm 
dccppa.py represents the dynamic curvature constrained path planning algorithm which is built on top of rrt and prm providing less number of nodes taken to construct a path during path planning from start to end goal.

## Requirements
Stable operating system with python version >=3.8 installed in it.

### Note 
If you want to change the obstacles between the start and end goal or in the config space please do add or remove the coordinates of the obstacles. The current version doesn't provide the option for users to input while running the script but rather you need to edit it on before hand. Improvements planned for the next update. This goes for RRT , PRM and DCCPPA scripts.

How to run 
1. python dccppa.py
2. python prm_rrt.py
The output of these files generate the no of steps created by the path from start point to end goal.
