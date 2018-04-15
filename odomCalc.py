import math

def odomCalc(prevTicks, currTicks):
	"""Calculates distance traveled from number of encoder ticks"""
	maxTicks = 32767 #Assuming tick count is 0 to 32767
	radTicks = (2 * math.pi) / maxTicks #radians per tick
	wheelDiam = 2 #2 inches, approximately
	wheelCircum  = wheelDiam * math.pi

	if(currTicks < prevTicks): #If the tick count rolls over
		diffTicks = (maxTicks - prevTicks) + currTicks
	else:
		diffTicks =  abs(currTicks - prevTicks)

	#Convert to radians
	diffTicksRad = diffTicks * radTicks
	#Convert to distance traveled
	distTraveled = (wheelCircum * diffTicksRad) / (2 * math.pi)
	return distTraveled