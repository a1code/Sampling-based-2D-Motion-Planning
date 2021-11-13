from collision import *
import numpy as np
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
		self.radiusOfSuccess = 0.3	# defines region of success around the goal
		self.success = False
		self.radiusOfNeighborhood = 0.5	# defines neighborhood around a point in rrt-star
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
		# Euclidean distance between two points
		return np.linalg.norm(np.array(point1) - np.array(point2))

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
		# discrete version with stepSize
		dirn = np.array(point2) - np.array(point1)
		length = np.linalg.norm(dirn)
		dirn = (dirn / length) * min (self.stepSize, length)

		cur = point1
		while True:
			newvex = (round(cur[0]+dirn[0], 1), round(cur[1]+dirn[1], 1))
			if not isCollisionFree(self.robot, newvex, self.obstacles) or self.exists(newvex):
				break

			if self.distance(point1, newvex) <= self.distance(point1, point2):
				cur = newvex
			else:
				break
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