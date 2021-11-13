import random
import math

def sample():
	x = round(random.uniform(0, 10), 1)
	y = round(random.uniform(0, 10), 1)
	theta = round(random.uniform(-1.*math.pi, math.pi), 2)

	return (x,y,theta)

# only for testing standalone "sampler"
if __name__ == "__main__":
	x,y,theta = sample()
	print((x,y,theta))