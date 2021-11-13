import random

def sample():
	x = round(random.uniform(0, 10), 1)
	y = round(random.uniform(0, 10), 1)

	return (x,y)

# only for testing standalone "sampler"
if __name__ == "__main__":
	x,y = sample()
	print((x,y))