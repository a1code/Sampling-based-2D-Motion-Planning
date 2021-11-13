# Geometric Motion Planning for a Car
The parser is implemented in file_parse.py. It reads space separated coordinates from a world definition text file and a problem definition text file. Figures below show a world definition file named robot_env_01.txt, and a problem definition file named probs_01.txt. The solution to this problem works as long as the inputs are given in the shown format.  

<img width="434" alt="q2_world_file" src="https://user-images.githubusercontent.com/10013303/141658478-ed3e7b94-30d2-47d0-97ea-19442bfe64af.png">
<img width="270" alt="q2_problem_file" src="https://user-images.githubusercontent.com/10013303/141658483-b298232b-26b4-400f-b29c-c87a8cdd30db.png">

The first line of the world file defines the robot and the rest of the lines are the obstacles (may or may not be convex), one obstacle per line. The robot geometry is defined by the width w and the length l separated by one space. The robot translates as well as rotates about its center, within a 10 × 10 environment with the lower left corner being (0, 0) and the upper right corner being (10, 10). The problem file contains one problem per line. Each problem is defined by start and goal poses from left to right. For each pose, the coordinates are x, y and &theta; respectively from left to right.  

An instance of _Robot_ class implemented in robot.py is initialized with the robot width and height. For a given pose, the _transform_ method translates and rotates the robot vertices, obtained from width w and height l using _(−w/2, −l/2), (−w/2, l/2), (w/2, l/2), (w/2, −l/2)_, to get its spatial configuration.  

<img width="875" alt="robot" src="https://user-images.githubusercontent.com/10013303/141658590-ed3e1509-78d1-4d2a-96f5-a853cf261a18.png">

Figure below shows the visualization for the obstacles, the world boundaries, the start and goal states of the robot for the first line in the problem file seen previously, by transforming the robot local geometry to the global coordinate system. Robot at start is shown in blue, robot at goal in green, whereas all obstacles are shown in red in the problem workspace.  

<img width="1440" alt="q2_problem1" src="https://user-images.githubusercontent.com/10013303/141658635-8d0a363d-68ea-4409-981d-940a79daac37.png">
