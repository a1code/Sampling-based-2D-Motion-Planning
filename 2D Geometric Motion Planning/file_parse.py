import sys

def parse_problem(world_file, problem_file):
	robot = []
	obstacles = []
	problems = []

	with open(world_file) as world:
		lines = world.readlines()

		# set robot
		r = lines[0].split(' ')
		coords = []
		for pt in r:
			if len(coords) == 2:
				robot.append((coords[0], coords[1]))
				coords = []
			coords.append(float(pt))
		robot.append((coords[0], coords[1]))
			
		
		# set obstacles
		for line in lines[1:]:
			obs = []
			o = line.split(' ')
			coords = []
			for pt in o:
				if len(coords) == 2:
					obs.append((coords[0], coords[1]))
					coords = []
				coords.append(float(pt))
			obs.append((coords[0], coords[1]))
			obstacles.append(obs)

	# set problems
	with open(problem_file) as prob:
		lines = prob.readlines()
		for line in lines:
			pair = []
			coords = line.split(' ')
			pair.append((float(coords[0]), float(coords[1])))
			pair.append((float(coords[2]), float(coords[3])))
			problems.append(pair)
	return (robot, obstacles, problems)


# only for testing standalone "file_parse"
if __name__ == "__main__":
	if(len(sys.argv) < 3):
		print("Two arguments required: python file_parse.py [world-file] [problem-file]")
		exit()
	
	world = sys.argv[1]
	problem = sys.argv[2]
	robot, obstacles, problems = parse_problem(world, problem)
	print('-------- ROBOT --------')
	print(robot)
	print('-------- OBSTACLES --------')
	print(obstacles)
	print('-------- PROBLEMS --------')
	print(problems)