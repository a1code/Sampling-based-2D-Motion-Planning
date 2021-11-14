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

	def kinematics(self, state, control):
		q = state 	#pose
		u = control #(v, omega)
		return (u[0]*math.cos(q[2] + math.pi/2.), u[0]*math.sin(q[2] + math.pi/2.), u[1])

	def dynamics(self, state, control):
		v_x,v_y,v_theta = self.kinematics(state, control)
		x,y,theta = state

		q = (x,y,theta,v_x,v_y,v_theta)
		u = control 	# a_v,a_theta
		return (q[3],q[4],q[5],u[0]*math.cos(q[2]+math.pi/2),u[0]*math.sin(q[2]+math.pi/2),u[1])

	def propagate(self, state, controls, durations, dt):
		out = []
		q_cur = state
		out.append(q_cur)
		for idx, control in enumerate(controls):
			q_dot = self.dynamics(q_cur, control)
			q_new = (round(q_cur[0] + q_dot[0]*durations[idx]*dt, 1), \
				round(q_cur[1] + q_dot[1]*durations[idx]*dt, 1), \
				round(q_cur[2] + q_dot[2]*durations[idx]*dt, 2))
			out.append(q_new)
			q_cur = q_new
		return out