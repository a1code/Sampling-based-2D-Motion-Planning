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
		self.stepSize = 0.1	# for discretized version of extend
		self.radiusOfSuccess = 0.8	# defines region of success around the goal
		self.success = False
		self.radiusOfNeighborhood = 1.5	# defines neighborhood around a point in rrt-star
		self.costs = {}
		self.costs[start] = 0
		self.path = []
		self.iterations = 0

	def add(self, point1, point2):
		self.vertices.add(point2)
		self.edges[point1].append(point2)
		self.costs[point2] = self.costs[point1] + self.distance(point1, point2)

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

	def extend(self, point1, point2):
		x1,y1,theta1 = point1
		x2,y2,theta2 = point2
		cur = point1

		# translate
		if x1 != x2 and y1 != y2:
			# discrete version with stepSize
			dirn = np.array((x2,y2)) - np.array((x1,y1))
			length = np.linalg.norm(dirn)
			dirn = (dirn / length) * min(self.stepSize, length)

			while True:
				newvex = (round(cur[0]+dirn[0], 1), round(cur[1]+dirn[1], 1), theta1)
				if not isCollisionFree(self.robot, newvex, self.obstacles) or self.exists(newvex):
					break

				if self.distance(point1, newvex) <= self.distance(point1, point2):
					cur = newvex
				else:
					break

		# rotate
		if theta1 != theta2:
			# discrete version with stepSize
			theta_cur = min(theta1, theta2)
			theta_final = max(theta1, theta2)
			while True:
				newvex = (cur[0], cur[1], round(theta_cur, 2))
				if not isCollisionFree(self.robot, newvex, self.obstacles) or self.exists(newvex):
					break
				if theta_cur <= theta_final:
					cur = newvex
				else:
					break
				theta_cur += self.stepSize

		if cur != point1:
			self.add(point1, cur)
			self.costs[cur] = self.costs[point1] + self.distance(cur, point1)
			return cur
		else:
			return None

	def get_cost(self, point):
		return self.costs[point]

	def get_neighborhood(self, point):
		neighbors = []
		for vertex in self.vertices:
			if vertex == point:
				continue
			if self.distance(point, vertex) <= self.radiusOfNeighborhood:
				neighbors.append(vertex)
		return neighbors

	def rewire(self, point, r):
		# for RRT*
		nearest = self.nearest(point)
		last = self.extend(nearest, point)
		if last is None:
			return
		if self.distance(last, self.goal) <= self.radiusOfSuccess:
			self.add(last, self.goal)
			self.success = True

		neighbors = self.get_neighborhood(point)
		for neighbor in neighbors:
			if self.get_cost(last) + self.distance(last, neighbor) < self.get_cost(neighbor):
				# check if no intersection with any obstacle before adding new edge
				intersection_flag = False
				for obstacle in self.obstacles:
					obs = [(Point(a[0],a[1]), Point(b[0],b[1])) for idx, a in enumerate(obstacle) for b in obstacle[idx + 1:]]
					for j in range(0, len(obs)):
						if doIntersect(Point(point[0],point[1]), Point(neighbor[0],neighbor[1]), obs[j][0], obs[j][1]):
							intersection_flag = True
				if not intersection_flag:
					self.costs[neighbor] = self.get_cost(last) + self.distance(last, neighbor)
					for k,v in self.edges.items():
						if neighbor in v:
							v.remove(neighbor)
					self.edges[last].append(neighbor)



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