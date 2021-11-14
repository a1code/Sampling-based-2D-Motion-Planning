# Geometric Motion Planning for a Car
The parser is implemented in file_parse.py. It reads space separated coordinates from a world definition text file and a problem definition text file. Figures below show a world definition file named robot_env_01.txt, and a problem definition file named probs_01.txt. The solution to this problem works as long as the inputs are given in the shown format.  

<img width="434" alt="q2_world_file" src="https://user-images.githubusercontent.com/10013303/141658478-ed3e7b94-30d2-47d0-97ea-19442bfe64af.png">
<img width="270" alt="q2_problem_file" src="https://user-images.githubusercontent.com/10013303/141658483-b298232b-26b4-400f-b29c-c87a8cdd30db.png">

The first line of the world file defines the robot and the rest of the lines are the obstacles (may or may not be convex), one obstacle per line. The robot geometry is defined by the width w and the length l separated by one space. The robot translates as well as rotates about its center, within a 10 × 10 environment with the lower left corner being (0, 0) and the upper right corner being (10, 10). The problem file contains one problem per line. Each problem is defined by start and goal poses from left to right. For each pose, the coordinates are x, y and &theta; respectively from left to right.  

An instance of _Robot_ class implemented in robot.py is initialized with the robot width and height. For a given pose, the _transform_ method translates and rotates the robot vertices, obtained from width w and height l using _(−w/2, −l/2), (−w/2, l/2), (w/2, l/2), (w/2, −l/2)_, to get its spatial configuration.  

<img width="875" alt="robot" src="https://user-images.githubusercontent.com/10013303/141658590-ed3e1509-78d1-4d2a-96f5-a853cf261a18.png">

Figure below shows the visualization for the obstacles, the world boundaries, the start and goal states of the robot for the first line in the problem file seen previously, by transforming the robot local geometry to the global coordinate system. Robot at start is shown in blue, robot at goal in green, whereas all obstacles are shown in red in the problem workspace.  

<img width="1440" alt="q2_problem1" src="https://user-images.githubusercontent.com/10013303/141658635-8d0a363d-68ea-4409-981d-940a79daac37.png">

The _sample_ method in sampler.py generates random samples within the 10 X 10 workspace (boundaries included) for x and y coordinates with uniform probability over the entire space, as well as within the `[-pi, pi`] radians for the &theta; with uniform probability over the entire angular region. A correct way to uniform sampling takes more consideration of the topology, but a naive implementation to directly sample randomly in the interval is used.  

In the _Tree_ class the _distance_ method is updated, where instead of Euclidean distance, the new distance formulation between poses (x<sub>1</sub>, y<sub>1</sub>, &theta;<sub>1</sub>) and (x<sub>2</sub>, y<sub>2</sub>, &theta;<sub>2</sub>) is used as shown below.  

<img width="878" alt="distance" src="https://user-images.githubusercontent.com/10013303/141662792-1203c270-cf0d-4857-8c75-f0b13d1a4b77.png">

- _radiusOfSuccess_ is set to 0.8 for both RRT and RRT*.
- _stepSize_ of 0.1 is used for both RRT and RRT*, to incrementally move the x,y coordinates as well as smaller of the two angles in the extend function (subject to not hitting an obstacle in between and not overshooting the sample point).
- radiusOfNeighborhood is set to 1.5 for RRT*.
- _iter_n_ is set to 10000 for both RRT and RRT*.  

For the problem seen above (first line in probs_01.txt), the figure below shows the path returned by a run of the **RRT** implementation (green line). The path from start pose (2.0, 2.0, 0.0) to goal pose (7.0, 8.0, 5.57) discovered for this run is (2.0, 2.0, 0.0), (2.0, 2.0, -0.1), (2.0, 2.2, -0.13), (2.0, 2.2, -0.14), (2.0, 2.2, -0.22), (2.0, 2.2, -0.26), (2.0, 2.2, -0.43), (2.0, 0.4, -0.52), (4.7, 0.4, -0.59), (8.7, 0.4, -0.61), (8.7, 4.0, -0.64), (9.7, 5.0, 0.56), (9.7, 7.9, 0.53), (8.0, 9.6, 0.43), (8.0, 8.4, 0.41), (6.8, 8.4, 0.36), (6.8, 8.4, 0.26), (7.2, 8.0, 0.23), (7.2, 8.0, 0.21), (7.2, 8.0, 0.18), (7.2, 8.0, 0.11), (7.0, 8.0, 5.57).  

<img width="1440" alt="q2_rrt_path_1" src="https://user-images.githubusercontent.com/10013303/141663338-ba33b7f7-7c12-4efa-9b05-0b27fd56c354.png">

The analysis for RRT over 20 runs is shown below as well as in the figure. **Remember to set the variable analysis_mode to True to generate these results. For a usual run of the algorithm, the variable should be set to False.**  

- Total number of runs = 20
- Success rate in percentage = 100.0
- Average number of iterations to find path = 2394.4
- Average path length = 21.5

<img width="1440" alt="q2_rrt_analysis" src="https://user-images.githubusercontent.com/10013303/141663476-ea775db6-5983-42ce-a223-14b4bb67b6db.png">

For the same problem above (first line in probs_01.txt), the figure below shows the path returned by a run of the **RRT*** implementation (green line). The path from start pose (2.0, 2.0, 0.0) to goal pose (7.0, 8.0, 5.57) discovered for this run is (2.0, 2.0, 0.0), (2.1, 2.0, 0.1), (2.1, 2.0, 0.06), (2.1, 2.0, 0.02), (2.1, 2.0, -0.01), (2.1, 1.6, 0.19), (1.8, 2.6, 0.58), (2.2, 3.6, -0.02), (2.2, 3.6, -0.04), (3.3, 4.3, -0.07), (4.1, 5.1, -0.14), (4.6, 5.6, 0.53), (5.3, 6.3, 0.5), (5.3, 7.7, 0.41), (6.5, 7.7, 0.38), (6.5, 7.7, 0.37), (6.7, 7.7, 0.87), (7.5, 9.0, 1.77), (6.9, 9.6, 1.74), (6.8, 9.3, -0.03), (6.8, 8.0, -0.05), (7.0, 8.0, 5.57).  

<img width="1440" alt="q2_rrt_star_path_1" src="https://user-images.githubusercontent.com/10013303/141663524-f63e838c-94ee-461b-827c-76c587036328.png">

The analysis for RRT* over 20 runs is shown below as well as in the figure. **Remember to set the variable analysis_mode to True to generate these results. For a usual run of the algorithm, the variable should be set to False.**  

- Total number of runs = 20
- Success rate in percentage = 100.0
- Average number of iterations to find path = 2290.2
- Average path length = 19.85

<img width="1440" alt="q2_rrt_star_analysis" src="https://user-images.githubusercontent.com/10013303/141663535-62a1f34e-2241-4841-9072-923f6ab563f4.png">

