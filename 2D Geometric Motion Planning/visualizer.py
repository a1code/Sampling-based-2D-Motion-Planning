from file_parse import parse_problem
from sampler import sample
import matplotlib.pyplot as plt
import random
import sys

def visualize_problem(robot, obstacles, start, goal):
	plt.figure()
	# environment limits
	plt.xlim([0, 10]) 
	plt.ylim([0, 10])

	# convert robot coordinates to world coordinates and translate to start and goal
	origin = robot[0]
	rob_start = []
	for idx, pt in enumerate(robot):
		if idx == 0:
			rob_start.append((start[0], start[1]))
		else:
			rob_start.append((round(pt[0]+start[0]-origin[0], 1), round(pt[1]+start[1]-origin[1], 1)))
	rob_end = []
	for idx, pt in enumerate(robot):
		if idx == 0:
			rob_end.append((goal[0], goal[1]))
		else:
			rob_end.append((round(pt[0]+goal[0]-origin[0], 1), round(pt[1]+goal[1]-origin[1], 1)))

	# robot start (shown in blue color)
	x1, y1 = zip(*rob_start)
	plt.fill(x1, y1, facecolor='blue', edgecolor='blue')
	
	# robot end (shown in green color)
	x2, y2 = zip(*rob_end)
	plt.fill(x2, y2, facecolor='green', edgecolor='green')

	# obstacles (shown in red color)
	for obstacle in obstacles:
		xs, ys = zip(*obstacle)
		plt.fill(xs, ys, facecolor='red', edgecolor='red')

	return plt


def visualize_points(points, robot, obstacles, start, goal):
	obj = visualize_problem(robot, obstacles, start, goal)

	for x,y in points:
		obj.plot(x,y,'ko', ms = 3) 
	obj.title("All samples during path planning")
	return obj


def visualize_path(robot, obstacles, path):
	start = path[0]
	end = path[-1]
	obj = visualize_problem(robot, obstacles, start, end)
	x_values = [pt[0] for pt in path]
	y_values = [pt[1] for pt in path]
	obj.plot(x_values, y_values, color='green')
	obj.title("Path found from start to goal")
	return obj

def visualize_rrt(robot, obstacles, start, goal, iter_n, edges):
	obj = visualize_problem(robot, obstacles, start, goal)
	for k,v in edges:
		if k is not None:
			for val in v:
				x_values = [k[0], val[0]]
				y_values = [k[1], val[1]]
				r = random.random()
				b = random.random()
				g = random.random()
				c = (r, g, b)
				obj.plot(x_values, y_values, color=c)
	obj.title("Final state of tree in RRT")
	return obj

def visualize_rrt_star(robot, obstacles, start, goal, iter_n, edges):
	obj = visualize_problem(robot, obstacles, start, goal)
	for k,v in edges:
		if k is not None:
			for val in v:
				x_values = [k[0], val[0]]
				y_values = [k[1], val[1]]
				r = random.random()
				b = random.random()
				g = random.random()
				c = (r, g, b)
				obj.plot(x_values, y_values, color=c)
	obj.title("Final state of tree in RRT*")
	return obj


def plot_statistics(iterations, path_lengths, algorithm):
	plt.figure()
	z = [x for _, x in sorted(zip(iterations, path_lengths))]
	plt.plot(sorted(iterations), z)
	plt.xlabel('number of iterations to find path')
	plt.ylabel('path length')
	plt.title("path length vs iterations for "+algorithm)
	return plt


# only for testing standalone "visualizer"
if __name__ == "__main__":
	if len(sys.argv) < 3:
		print("Two arguments required: python visualizer.py [world_file] [problem_file]")
		sys.exit()
	
	world_file = sys.argv[1]
	problem_file = sys.argv[2]
	robot, obstacles, problems = parse_problem(world_file, problem_file)

	start = problems[0][0]
	goal = problems[0][1]
	viz1 = visualize_problem(robot, obstacles, start, goal)
	viz1.show()

	pts = [sample() for x in range(0,20)]
	viz2 = visualize_points(pts, robot, obstacles, start, goal)
	viz2.show()