import math

class Robot():
	def __init__(self, width, height):
		self.width = width
		self.height = height
		self.translation = (0.,0.)
		self.rotation = math.radians(0.)

	def set_pose(self, pose):
		self.translation = (pose[0], pose[1])
		self.rotation = math.radians(pose[2])
		
	def transform(self):
		w, l = self.width, self.height
		x_t, y_t = self.translation
		points = []
		for x, y in [(-w / 2, -l / 2), (-w / 2, l / 2), (w / 2, l / 2), (w / 2, -l / 2)]:
			new_x = x * math.cos(self.rotation) - y * math.sin(self.rotation) + x_t
			new_y = x * math.sin(self.rotation) + y * math.cos(self.rotation) + y_t
			points.append((new_x, new_y))
		self.translated_robot = points
		return points