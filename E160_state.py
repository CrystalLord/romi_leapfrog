
class E160_state:
    def __init__(self, x=0, y=0, theta=0):
        self.set_state(x, y, theta)
        
    def set_state(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __repr__(self):
        return "E160_state({},{},{})".format(self.x, self.y, self.theta)
