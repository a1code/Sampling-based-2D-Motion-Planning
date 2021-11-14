# Sampling-based-2D-Motion-Planning
Probabilistic Roadmap Method (PRM) is mainly geared towards "multi-query" motion planning problems. The idea is to sample points in the robot workspace and connect them using a fast "local planner", in order to build an offline graph or "roadmap", representing the conectivity of the environment. At run-time, we connect the start and goal to the closest nodes in the roadmap, for quickly figuring out paths.  

After an initial roadmap is built, multiple queries can be executed on the same roadmap, hence the "multi-query" approach. However, this also means that the roadmap is likely to contain a lot of useless information for a single query. So we explore the "single-query" alternatives, the RRT (Rapidly-exploring Random Tree) and RRT* algorithms in this implementation, and their application to systems with dynamics. The focus of the approaches is to reach quickly the unexplored parts of the space and then fill in the details.  

**Implementation Summary** :  
- Wrote modules in Python to define the geometry, kinematics and dynamics of polygonal translating and rotating robots, in 2D environments containing polygonal obstacles
- Implemented and visualized collision-free path planning between source and goal states using sampling-based RRT and RRT* algorithms
- Performed trials and reported observations to compare the success rates, path lengths and convergence time for applying these algorithms in multiple robot settings
