from file_parse import parse_problem
from visualizer import *
from sampler import sample
from collision import *
from tree import Tree
import sys

viz_prob = None
viz_pts = None
viz_tree = None
viz_path = None

def rrt_star(robot, obstacles, start, goal, iter_n, visualize=False):
	tree = Tree(robot, obstacles, start, goal)
	samples = []
	i = 0
	# stop if region of success found or iter_n number of iterations complete
	while not tree.success and i < iter_n:
		i += 1
		point2 = sample()
		samples.append(point2)
		if not isCollisionFree(tree.robot, point2, tree.obstacles) or tree.exists(point2):
			continue
		tree.rewire(point2, tree.radiusOfNeighborhood)
	tree.iterations = i
	# backtrack from goal node to root node to trace the path
	if tree.success:
		node = goal
		while node is not None:
			tree.path.append(node)
			node = tree.parent(node)
		tree.path = list(reversed(tree.path))

		if visualize:
			# visualize workspace, all sample points, the final tree state, and the path found
			global viz_path, viz_prob, viz_pts, viz_tree
			viz_prob = visualize_problem(robot, obstacles, start, goal)
			viz_pts = visualize_points(samples, robot, obstacles, start, goal)
			viz_tree = visualize_rrt_star(robot, obstacles, start, goal, iter_n, tree.edges.items())
			viz_path = visualize_path(robot, obstacles, tree.path)

	return tree

def analysis(robot, obstacles, problems, iter_n):
	iterations = []
	path_lengths = []
	outcomes = []

	trials = range(0, 30)
	for trial in trials:
		print('trial', trial)
		for problem in problems:
			start = problem[0]
			goal = problem[1]
			tree = rrt_star(robot, obstacles, start, goal, iter_n)
			outcomes.append(tree.success)
			if tree.success:
				iterations.append(tree.iterations)
				path_lengths.append(len(tree.path))
	print('Total no. of runs', len(outcomes))
	print('Success Rate in %', (sum(outcomes)*1./len(outcomes))*100.)
	print('Avg number of iterations to find path', sum(iterations)*1./len(iterations))
	print('Avg path length', sum(path_lengths)*1./len(path_lengths))
	viz_stats = plot_statistics(iterations, path_lengths, 'RRT*')
	viz_stats.show()


# execution starts here
if __name__ == "__main__":
	if len(sys.argv) < 3:
		print("Two arguments required: python rrt_star.py [world_file] [problem_file]")
		sys.exit()
	
	world_file = sys.argv[1]
	problem_file = sys.argv[2]
	robot, obstacles, problems = parse_problem(world_file, problem_file)
	iter_n = 2000

	# only set True for collecting algorithm statistics
	analysis_mode = False
	if analysis_mode:
		analysis(robot, obstacles, problems, iter_n)
	else:
		# find paths
		all_plots = []
		for problem in problems:
			start = problem[0]
			goal = problem[1]
			print('start', start)
			print('goal', goal)
			tree = rrt_star(robot, obstacles, start, goal, iter_n, visualize=True)
			print('Found region of success around the goal', tree.success)
			print('path', tree.path)
			if viz_prob:
				all_plots.append(viz_prob)
			if viz_pts:
				all_plots.append(viz_pts)
			if viz_tree:
				all_plots.append(viz_tree)
			if viz_path:
				all_plots.append(viz_path)

		# display visualizations
		for plot in all_plots:
			plot.show()