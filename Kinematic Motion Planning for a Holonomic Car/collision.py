from file_parse import parse_problem
from sampler import sample
import sys

class Point:
	def __init__(self, x, y):
		self.x = x
		self.y = y

def onSegment(p, q, r):
	if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and
		   (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))):
		return True
	return False

def orientation(p, q, r):
	val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y))
	if (val > 0):
		 
		# Clockwise orientation
		return 1
	elif (val < 0):
		 
		# Counterclockwise orientation
		return 2
	else:
		 
		# Collinear orientation
		return 0

def doIntersect(p1,q1,p2,q2):
	o1 = orientation(p1, q1, p2)
	o2 = orientation(p1, q1, q2)
	o3 = orientation(p2, q2, p1)
	o4 = orientation(p2, q2, q1)
 
	# General case
	if ((o1 != o2) and (o3 != o4)):
		return True
 
	# Special Cases
 
	# p1 , q1 and p2 are collinear and p2 lies on segment p1q1
	if ((o1 == 0) and onSegment(p1, p2, q1)):
		return True
 
	# p1 , q1 and q2 are collinear and q2 lies on segment p1q1
	if ((o2 == 0) and onSegment(p1, q2, q1)):
		return True
 
	# p2 , q2 and p1 are collinear and p1 lies on segment p2q2
	if ((o3 == 0) and onSegment(p2, p1, q2)):
		return True
 
	# p2 , q2 and q1 are collinear and q1 lies on segment p2q2
	if ((o4 == 0) and onSegment(p2, q1, q2)):
		return True
 
	# If none of the cases
	return False


def check_point_in_poly(p, poly):
	poly_cross_count = 0
	for p_1_idx in xrange(0,len(poly)):
		p_1 = poly[p_1_idx]
		p_2 = poly[(p_1_idx + 1) % len(poly)]
		if(p[1] >  min(p_1[1] , p_2[1]) and
		   p[1] <= max(p_1[1] , p_2[1]) and
		   p[0] <= max(p_1[0] , p_2[0]) and
		   p_1[1] != p_2[1]):
			interval = (p[1] - p_1[1]) * (p_2[0] - p_1[0]) / (p_2[1] - p_1[1]) + p_1[0];
			if( p_1[0] == p_2[0] or p[0] <= interval):
				poly_cross_count += 1

	poly_cross_count = poly_cross_count % 2
	if(poly_cross_count):
		return True
	else:
		return False


def check_point_in_polys(p , polygons):
	for poly in polygons:
		if(check_point_in_poly(p ,poly)):
			return True
	return False


def isCollisionFree(robot, point, obstacles):
	robot.set_pose(point)
	rob_at_pt = robot.transform()

	# check if outside env
	for pt in rob_at_pt:
		if pt[0]>=10 or pt[1]>=10 or pt[0]<=0 or pt[1]<=0:
			return False

		# check if inside obstacles
		if(check_point_in_polys(pt , obstacles)):
			return False

	# check if edge intersections
	rob = [(Point(a[0],a[1]), Point(b[0],b[1])) for idx, a in enumerate(rob_at_pt) for b in rob_at_pt[idx + 1:]]
	for obstacle in obstacles:
		obs = [(Point(a[0],a[1]), Point(b[0],b[1])) for idx, a in enumerate(obstacle) for b in obstacle[idx + 1:]]
		for i in range(0, len(rob)):
			for j in range(0, len(obs)):
				if doIntersect(rob[i][0], rob[i][1], obs[j][0], obs[j][1]):
					return False
	return True


# only for testing standalone "collision"
if __name__ == "__main__":
	if(len(sys.argv) < 3):
		print("Two arguments required: python collision.py [world-file] [problem-file]")
		exit()
	
	world = sys.argv[1]
	problem = sys.argv[2]
	robot, obstacles, _ = parse_problem(world, problem)

	pts = [sample() for x in range(0,20)]
	for pt in pts:
		print('collision free', isCollisionFree(robot, pt, obstacles))