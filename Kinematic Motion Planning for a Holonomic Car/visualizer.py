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

	# transform the robot local geometry to the global coordinate system
	plt.plot(start[0],start[1],'ko', ms = 3)
	robot.set_pose(start)
	rob_start = robot.transform()
	plt.plot(goal[0],goal[1],'ko', ms = 3)
	robot.set_pose(goal)
	rob_end = robot.transform()

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

	for pose in points:
		plt.plot(pose[0],pose[1],'ko', ms = 3)
		robot.set_pose(pose)
		robot_new = robot.transform()
		xs, ys = zip(*robot_new)
		obj.fill(xs, ys, facecolor='orange', edgecolor='orange')
	return obj

def visualize_path(robot, obstacles, path):
	start = path[0]
	goal = path[-1]
	obj = visualize_problem(robot, obstacles, start, goal)
	for pose in path[1:-1]:
		obj.plot(pose[0],pose[1],'ko', ms = 3)
		robot.set_pose(pose)
		robot_new = robot.transform()
		xs, ys = zip(*robot_new) 
		obj.fill(xs, ys, facecolor='orange', edgecolor='orange')
	x_values = [pt[0] for pt in path]
	y_values = [pt[1] for pt in path]
	obj.plot(x_values, y_values, color='green')
	obj.title("Path found from start to goal")
	return obj

def visualize_trajectory(robot, obstacles, start, goal, trajectory):
	obj = visualize_problem(robot, obstacles, start, goal)
	for pose in trajectory:
		obj.plot(pose[0],pose[1],'ko', ms = 3)
	x_values = [start[0]]
	y_values = [start[1]]
	for pt in trajectory:
		x_values.append(pt[0])
		y_values.append(pt[1])
	x_values.append(goal[0])
	y_values.append(goal[1])
	obj.title("Robot trajectory (valid states visited)")
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
	