from collision import *
import numpy as np
import math
from collections import defaultdict

class Tree:
	def __init__(self, robot, obstacles, start, goal):
		self.robot = robot
		self.obstacles = obstacles
		self.start = start
		self.goal = goal
		self.vertices = set()
		self.vertices.add(start)
		self.edges = defaultdict(list)
		self.edges[None] = start
		self.radiusOfSuccess = 0.8	# defines region of success around the goal
		self.success = False
		self.path = []
		self.ulow = 0.1
		self.uhigh = 0.9
		self.n1 = 1
		self.n2 = 9
		self.dt = 0.01
		self.trajectory = []
		self.iterations = 0

	def add(self, point1, point2):
		self.vertices.add(point2)
		self.edges[point1].append(point2)

	def exists(self, point):
		if point in self.vertices:
			return True
		else:
			return False

	def parent(self, point):
		for k,v in self.edges.items():
			for val in v:
				if val == point:
					return k
		return None

	def distance(self, point1, point2):
		x1,y1,theta1 = point1
		x2,y2,theta2 = point2
		
		d_x = (x1 - x2)**2.
		d_y = (y1 - y2)**2.
		delta = theta1 - theta2
		d_angular = (math.radians((math.degrees(delta) + 180) % 360 - 180))**2
		return round(math.sqrt(d_x + d_y + d_angular), 1)

	def nearest(self, point):
		nearest = None
		min_dist = None
		for vertex in self.vertices:
			dist = self.distance(point, vertex)
			if min_dist is None or dist < min_dist:
				min_dist = dist
				nearest = vertex
		return nearest

	def extend(self, point, n1, n2, dt):
		x = np.random.uniform(self.ulow, self.uhigh)
		u = [(x, x) for i in range(0, 40)]
		n = [np.random.uniform(low=n1, high=n2) for i in range(0, 40)]
		states = self.robot.propagate(point, u, n, dt)

		cur = point
		for state_new in states[1:]:
			if not isCollisionFree(self.robot, state_new, self.obstacles) or self.exists(state_new):
				break
			self.trajectory.append(state_new)
			cur = state_new
		if cur != point:
			self.add(point, cur)
			return cur
		else:
			return None


# only for testing standalone "tree"
if __name__ == "__main__":
	if len(sys.argv) < 3:
		print("Two arguments required: python tree.py [world_file] [problem_file]")
		sys.exit()
	
	world_file = sys.argv[1]
	problem_file = sys.argv[2]
	robot, obstacles, problems = parse_problem(world_file, problem_file)

	start = problems[0][0]
	goal = problems[0][1]

	tree = Tree(robot, obstacles, start, goal)
	print(tree.edges)