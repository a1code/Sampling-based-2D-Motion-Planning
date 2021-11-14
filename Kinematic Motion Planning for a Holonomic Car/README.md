# Kinematic Motion Planning for a Holonomic Car
Previously in the Geometric Motion Planning for a Car, we only reasoned about geometry, and didn’t consider velocity or even accelerations. However, in real life, robots have to respect physical constraints. Using the same representation for the car geometry as before, we extend here to the kinematics motion planning setting. We assume we have direct control over the velocities of the car using two variables: linear velocity v and the angular velocity &omega; for steering. (Note that here we assume you can suddenly change the velocities of the robot, which is not achievable in reality). With these two variables, for a given pose of the robot at time t (x, y, &theta;), the velocity of the pose is given by  

<img width="935" alt="kinematics" src="https://user-images.githubusercontent.com/10013303/141663790-dfca4d44-4389-4722-b16a-e201a640cc5d.png">

In order to account for the kinematics of the system, we add methods _kinematics_ and _propagate_ in the _Robot_ class, and only update the _extend_ method in the _Tree_ class to use the random shooting algorithm. The random shooting algorithm just randomly samples control input u, and a duration number n ∈ \[n<sub>1</sub>, n<sub>2</sub>\), and then propagate the state q(0) by them to obtain the end state q(n · dt). The control input u is defined within some limits u ∈ \[u<sub>low</sub>, u<sub>high</sub>\], which are the tunable hyperparameters in this implementation besides n<sub>1</sub> and n<sub>2</sub>. If collision happens in the middle, the extend method stops at the last collision-free point as the end point. It then adds the endpoint to the tree.

- _radiusOfSuccess_ = 0.3
- _u<sub>low</sub>_ = 0.1
- _u<sub>high</sub>_ = 0.9
- _n<sub>1</sub>_ = 1
- _n<sub>2</sub>_ = 9
- _dt_ = 0.01
- _iter_n_ = 10000

For the same problem as seen in Geometric Motion Planning for a Car, the figures below show the path  (green line) as well as the trajectory (black dots) returned by a run of the **RRT** implementation. The path from start pose (2.0, 2.0, 0.0) to goal pose (7.0, 8.0, 5.57) is (2.0, 2.0, 0.0), (2.0, 2.3, 0.56), (2.0, 2.3, 0.98), (1.1, 2.3, 2.54), (1.1, 2.3, 3.05), (1.1, 2.3, 4.15), (1.1, 2.3, 5.12), (1.1, 2.3, 6.0), (1.1, 2.6, 7.24), (0.8, 2.6, 8.43), (0.8, 2.6, 9.43), (0.8, 2.6, 10.4), (1.8, 2.6, 11.45), (1.8, 2.9, 12.48), (1.8, 2.9, 13.13), (1.0, 2.9, 14.73), (1.0, 2.9, 15.92), (1.1, 2.9, 17.2), (1.1, 2.9, 17.87), (1.2, 4.1, 19.69), (1.2, 4.1, 20.58), (1.2, 4.1, 21.51), (1.2, 3.9, 22.72), (2.3, 3.9, 24.42), (2.2, 5.3, 26.27), (2.2, 5.3, 27.37), (2.2, 5.2, 28.57), (2.5, 5.1, 29.97), (2.5, 5.1, 31.08), (2.5, 5.1, 31.28), (2.5, 5.1, 31.3), (2.5, 5.1, 31.55), (2.5, 5.1, 32.44), (2.5, 5.1, 33.58), (2.5, 5.1, 34.18), (3.0, 4.2, 35.88), (3.4, 4.2, 36.51), (3.4, 4.2, 37.3), (3.4, 4.5, 37.55), (3.4, 4.5, 38.57), (3.4, 4.5, 39.19), (3.4, 4.5, 39.87), (3.4, 4.5, 40.51), (3.4, 4.5, 41.54), (3.4, 4.5, 41.85), (3.6, 4.5, 43.12), (3.6, 4.5, 43.54), (3.6, 4.7, 44.66), (3.6, 4.7, 45.25), (3.6, 4.7, 45.55), (3.6, 4.7, 46.29), (3.6, 4.7, 47.0), (3.6, 4.6, 48.15), (4.5, 5.3, 50.06), (4.2, 5.9, 51.65), (4.2, 5.9, 52.04), (4.2, 5.9, 52.83), (4.2, 5.9, 53.73), (4.2, 5.9, 54.33), (5.1, 5.9, 55.95), (5.1, 6.9, 57.53), (5.1, 6.9, 58.68), (5.1, 6.9, 59.86), (5.1, 6.9, 60.61), (5.1, 6.9, 61.59), (5.1, 6.9, 62.16), (4.8, 8.3, 64.16), (4.8, 8.3, 65.01), (4.8, 8.3, 65.73), (4.8, 8.2, 67.08), (5.9, 8.4, 68.82), (5.7, 9.1, 70.3), (5.7, 9.1, 70.62), (5.7, 9.1, 71.45), (5.8, 7.9, 73.27), (6.8, 7.9, 74.77), (7.0, 8.0, 5.57).  

<img width="1440" alt="q3_rrt_path_1" src="https://user-images.githubusercontent.com/10013303/141664161-5f14a521-35b0-4ffd-a71d-016f41b2e0d9.png">
<img width="1440" alt="q3_rrt_traj_1" src="https://user-images.githubusercontent.com/10013303/141664164-af43c1b0-0d1d-4d75-8d30-9f6753822435.png">

The analysis for RRT over 10 runs is shown below as well as in the figure. **Remember to set the variable analysis to True to generate these results. For a usual run of the algorithm, the variable should be set to False.**  

- Total number of runs = 10
- Success rate in percentage = 30.0
- Average number of iterations to find path = 6574.333333333333
- Average path length = 116.66666666666667

<img width="1440" alt="q3_rrt_analysis" src="https://user-images.githubusercontent.com/10013303/141664186-4045bc90-5fb9-44c1-9cd8-20df67d2bfe8.png">
