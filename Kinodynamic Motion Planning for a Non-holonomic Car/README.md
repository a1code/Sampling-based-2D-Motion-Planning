Previously in the Kinematic Motion Planning for a Holonomic Car, we assumed to have direct control over the velocities. However, due to the Newton’s Laws, sudden change to velocities is unrealistic. Here we implement RRT in the more realistic setting where we have direct control over the system kinodynamics with accelerations.  

We control the accelerations of the car using two variables: linear acceleration a<sub>v</sub> and the steering angle acceleration a<sub>&theta;</sub>. Given the pose of the robot at time t as (x, y, &theta;), the acceleration of the pose is then given by  

<img width="829" alt="dynamics" src="https://user-images.githubusercontent.com/10013303/141664335-69303aea-d09f-4c63-ab21-6ba02f37f715.png">

In order to account for the dynamics of the system besides kinematics, we add the method _dynamics_ in the Robot class. It implements the dynamics equation of the robot by following (2) and return the computed configuration velocity.  

- _radiusOfSuccess_ = 0.8
- _u<sub>low</sub>_ = 0.1
- _u<sub>high</sub>_ = 0.9
- _n<sub>1</sub>_ = 1
- _n<sub>2</sub>_ = 9
- _dt_ = 0.01
- _iter_n_ = 10000

For the same problem as seen in Geometric Motion Planning for a Car, the figures below show the path (green line) as well as the trajectory (black dots) returned by a run of the **RRT** implementation. The path from start pose (2.0, 2.0, 0.0) to goal pose (7.0, 8.0, 5.57) is (2.0, 2.0, 0.0), (2.0, 2.0, 1.27), (1.3, 2.0, 2.67), (1.3, 2.0, 3.69), (1.3, 2.0, 4.49), (1.3, 2.0, 5.02), (1.3, 2.0, 5.85), (0.9, 3.2, 7.63), (0.9, 3.2, 8.42), (0.9, 3.2, 8.91), (0.9, 3.2, 9.69), (0.9, 3.2, 10.13), (1.7, 3.2, 11.37), (1.7, 3.2, 11.72), (1.7, 4.1, 13.41), (1.7, 4.1, 13.98), (1.7, 4.1, 14.74), (1.7, 4.1, 15.94), (1.7, 4.1, 16.65), (1.7, 4.1, 17.14), (2.5, 4.4, 18.82), (2.4, 5.0, 20.27), (2.4, 5.0, 21.15), (2.4, 5.0, 21.91), (3.1, 4.0, 23.43), (3.1, 4.0, 24.08), (3.1, 4.4, 24.89), (3.1, 4.4, 25.02), (3.1, 4.4, 25.77), (3.1, 4.4, 26.31), (3.1, 4.4, 26.35), (3.1, 4.4, 27.01), (3.1, 4.4, 27.51), (3.1, 4.4, 27.83), (3.1, 4.4, 27.99), (3.1, 4.4, 28.69), (3.1, 4.4, 29.08), (3.8, 4.4, 29.89), (3.8, 4.4, 30.66), (3.8, 5.1, 32.15), (3.8, 5.1, 32.42), (3.8, 5.1, 33.28), (3.8, 5.1, 34.23), (3.8, 5.1, 34.98), (3.8, 5.1, 35.92), (4.7, 5.9, 37.64), (4.4, 6.6, 39.09), (4.4, 6.6, 39.94), (4.4, 6.6, 40.99), (4.4, 6.6, 41.51), (5.3, 6.5, 42.7), (5.3, 6.5, 43.36), (5.0, 7.6, 45.05), (5.0, 7.6, 46.08), (5.0, 7.6, 46.38), (5.0, 7.6, 47.32), (5.0, 7.6, 48.05), (5.8, 7.6, 49.65), (5.8, 7.6, 50.65), (5.8, 7.6, 51.78), (5.8, 7.6, 52.99), (5.8, 7.6, 53.49), (5.8, 7.6, 53.82), (5.8, 7.6, 54.43), (6.3, 7.6, 55.82), (7.0, 8.0, 5.57).  

<img width="1440" alt="q4_rrt_path" src="https://user-images.githubusercontent.com/10013303/141664554-bb10097c-ccef-4167-ba37-888f6eff88ac.png">
<img width="1440" alt="q4_rrt_trajectory" src="https://user-images.githubusercontent.com/10013303/141664556-16ef0a76-c504-4b79-8ae9-689f652f001f.png">

The analysis for RRT over 10 runs is shown below as well as in the figure. **Remember to set the variable analysis to True to generate these results. For a usual run of the algorithm, the variable should be set to False.**  

- Total number of runs = 10
- Success rate in percentage = 100.0
- Average number of iterations to find path = 5466.0
- Average path length = 100.6

<img width="1440" alt="q4_rrt_analysis" src="https://user-images.githubusercontent.com/10013303/141664576-8b190704-308a-4968-84de-390524d3cb93.png">
