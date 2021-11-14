# 2D Geometric Motion Planning
The parser is implemented in file_parse.py. It reads space separated coordinates from a world definition text file and a problem definition text file. Figures below show a world definition file named robot_env_01.txt, and a problem definition file named probs_01.txt. The solution to this problem works as long as the inputs are given in the shown format.  

<img width="450" alt="world_file" src="https://user-images.githubusercontent.com/10013303/141657506-65ad45c6-7f9a-4611-9668-8fd528ccf858.png">
<img width="279" alt="problem_file" src="https://user-images.githubusercontent.com/10013303/141657507-d110564c-688a-4ca4-8015-d3bb5f68cffe.png">

The first line of the world file defines the robot and the rest of the lines are the obstacles (may or may not be convex), one obstacle per line. The robot, in its local coordinate system, is defined as a list of clockwise arranged points, with the first coordinate being the origin of the robot coordinate system. The robot only translates within a 10 × 10 environment with the lower left corner being (0, 0) and the upper right corner being (10, 10). The problem file contains one problem per line. Each problem is defined by x and y coordinates of start point followed by the x and y coordinates of goal point respectively from left to right.  

<img width="883" alt="robot_example" src="https://user-images.githubusercontent.com/10013303/141657551-3b6050db-a40e-4084-8177-cca7961b8b26.png">

Figures below show the visualization for the obstacles, the world boundaries, the start and goal states of the robot for the two lines in the problem file seen previously. Robot at the start point is shown in blue, robot at the end point in green, whereas all obstacles are shown in red in the problem workspace.  

<img width="1440" alt="problem1" src="https://user-images.githubusercontent.com/10013303/141657614-9970b74f-1a6b-475d-ba3b-ffe9884ffe63.png">
<img width="1440" alt="problem2" src="https://user-images.githubusercontent.com/10013303/141657617-3ac7ab81-066a-4dc0-9ce0-f41ac10d0174.png">

The _sample_ method in sampler.py generates random samples within the 10 X 10 workspace (boundaries included), with uniform probability over the entire space. In collision.py, we convert the robot vertices to world coordinates and translate it to a given point to check whether it is collision free at that point. It is deemed collision free if all three of the checks below are passed.  

- The new vertex coordinates stay inside the $10 \times 10$ workspace.
- The new robot coordinates are not completely inside any of the obstacles.
- There is no intersection between any of the robot edges and any of the obstacle edges, for all the obstacles.  

An instance of the _Tree_ class, implemented in tree.py, will hold our random tree that rapidly explores the workspace during sampling based motion planning. The parameters and key implementation specifics for the same are listed below.  

- A hash set named _vertices_ is used to store all the tree nodes.
- Tree _edges_ are specified by entries in a Python defaultdict. For each key-value pair in this dictionary, there is an edge between key and value, with key as the parent node. Defaultdict is used because it allows duplicate keys, since one node can have multiple children in a tree.
- _radiusOfSuccess_ is a parameter which specifies the distance from the exact goal coordinates, which if reached is considered as the criteria of success for the path finding problem. A value of 0.3 is used for both RRT and RRT*.
- _stepSize_ is a parameter used for the discretized version of the _extend_ method implementation. It defines the size of each step taken, in order to extend an edge from a tree node towards a valid sample (subject to not hitting an obstacle in between and not overshooting the sample point). A value of 0.1 is used for both RRT and RRT*.
- _radiusOfNeighborhood_ is a parameter which specifies the distance from a point to look for in order to find the neighborhood, while executing the RRT* algorithm. The value used is 0.5. Another possibility is to replace this value with log of the number of vertices in the tree, but that is not used in this implementation.
- For each node, the dictionary _costs_ holds the cost from start to that node, measured as Euclidean path length. The costs are only considered for RRT* implementation, since the algorithm tries to achieve asymptotic optimality.
- The method _distance_ is used to compute Euclidean distance between any given pair of coordinates in the workspace. All distance computations for this implementation are Euclidean distances. 
- We continue iterating until either the region of success around the goal is reached, or _n_iter_ = 1000 number of loops.  
- We finally backtrack from the goal node until the root using _parent_ method to trace the start to goal path.  

For the first problem above (first line in probs_01.txt), the figures below show the path returned by a run of the **RRT** implementation (green line), the corresponding samples collected during the iterations (black dots), and the final state of the search tree in the same workspace. The path from start (0.1, 0.1) to goal (6.0, 8.0) discovered for this run is (0.1, 0.1), (1.8, 0.1), (1.8, 1.6), (0.1, 3.3), (0.1, 9.6), (2.9, 9.6), (9.2, 9.6), (7.3, 9.6), (6.4, 8.7), (5.8, 8.1), (6.0, 8.0).  

<img width="1440" alt="q1_rrt_path_1" src="https://user-images.githubusercontent.com/10013303/141657891-a24c0d6e-ac9f-4613-846b-6636966584f4.png">
<img width="1440" alt="q1_rrt_sample_1" src="https://user-images.githubusercontent.com/10013303/141657895-849d1e38-d061-4d6e-96d0-15e5c9d3b8e4.png">
<img width="1440" alt="q1_rrt_tree_1" src="https://user-images.githubusercontent.com/10013303/141657899-0899eeb0-05ec-4736-a419-b45c43e045dd.png">

The analysis for RRT over 60 runs is shown below as well as in figure. **Remember to set the variable _analysis_mode_ to True to generate these results. For a usual run of the algorithm, the variable should be set to False.**  

- Total number of runs = 60
- Success rate in percentage = 78.33333333333333
- Average number of iterations to find path = 474.9574468085106
- Average path length = 14.340425531914894

<img width="1440" alt="q1_rrt_analysis" src="https://user-images.githubusercontent.com/10013303/141658060-ed4254f6-aa8c-49af-9f6d-b4c988ab1a38.png">

For the first problem above (first line in probs_01.txt), the figures below show the path returned by a run of the **RRT*** implementation (green line), the corresponding samples collected during the iterations (black dots), and the final state of the search tree in the same workspace. The path from start (0.1, 5.0) to goal (9.0, 8.0) discovered for this run is (0.1, 5.0), (1.8, 3.3), (2.9, 4.4), (4.7, 4.4), (4.1, 4.4), (4.1, 6.4), (6.7, 9.0), (7.5, 8.2), (8.3, 8.2), (8.5, 8.2), (9.0, 8.2), (9.0, 8.0).  

<img width="1440" alt="q1_rrt_star_path_1" src="https://user-images.githubusercontent.com/10013303/141658171-6a4caa53-ef43-4653-83cf-73871cfb7d08.png">
<img width="1440" alt="q1_rrt_star_sample_1" src="https://user-images.githubusercontent.com/10013303/141658179-90ce71e2-aa86-4c36-8733-ed5474c76bb1.png">
<img width="1440" alt="q1_rrt_star_tree_1" src="https://user-images.githubusercontent.com/10013303/141658187-0c96843d-e7c0-458a-b19b-c0d1eba1e3e2.png">

The analysis for RRT* over 60 runs is shown below as well as in figure. **Remember to set the variable _analysis_mode_ to True to generate these results. For a usual run of the algorithm, the variable should be set to False.**  

- Total number of runs = 60
- Success rate in percentage = 100.0
- Average number of iterations to find path = 663.4833333333333
- Average path length = 15.316666666666666

<img width="1440" alt="q1_rrt_star_analysis" src="https://user-images.githubusercontent.com/10013303/141658232-dac1c38e-74ef-4699-8e41-b15a1a08b157.png">

_Comparing the two algorithms, we do notice that RRT* creates relatively straighter paths. It vines around obstacles, while moving fairly straight through free spaces. However, it ends up taking more iterations to converge to the success region, since the rewiring process makes it computationally more expensive._  
