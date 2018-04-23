from E160_robot import *
from E160_state import *
from E160_wall import *
import E160_PF

import serial
from xbee import XBee

class E160_environment:
    
    def __init__(self, mode="SIMULATION MODE"):
        #self.width = 2.0
        #self.height = 1.2
        self.width = 6.0
        self.height = 3.0

        # set up walls, putting top left point first
        self.walls = []
        self.walls.append(E160_wall([-0.5, 0.3, -0.5, -0.3], "vertical"))
        self.walls.append(E160_wall([1, 0.8, 1, -0.3], "vertical"))
        self.walls.append(E160_wall([-0.4, 0.5, 0.4, 0.5], "horizontal"))
        self.walls.append(E160_wall([-0.4, 1, 1, 1], "horizontal"))
        self.walls.append(E160_wall([-1, 0.8, -1, -0.3], "vertical"))
        # self.walls.append(E160_wall([0.5, -0.5, 1, -0.5],"horizontal"))
        # self.walls.append(E160_wall([-0.5, -0.5, 0.5, -1],"horizontal"))
        # self.walls.append(E160_wall([-0.5, -0.5, 0.0, -1.0],"vertical"))

        # create vars for hardware vs simulation
        # "SIMULATION MODE" or "HARDWARE MODE"
        self.robot_mode = mode
        self.control_mode = "MANUAL CONTROL MODE"

        # setup xbee communication
        if (self.robot_mode == "HARDWARE MODE"):
            self.serial_port = serial.Serial('COM9', 9600)
            print(" Setting up serial port")
            try:
                self.xbee = XBee(self.serial_port)
            except:
                print("Couldn't find the serial port")
        
        # Setup the robots

        self.num_robots = 2
        self.robots = []
        for i in range(self.num_robots):
            # TODO: assign different address to each bot
            r = E160_robot(self, '\x00\x0C', i)
            self.robots.append(r)

        self.pf = E160_PF.E160_PF(self, self.robots)

    def update_robots(self, deltaT):
        
        # loop over all robots and update their state
        for r in self.robots:
            
            # set the control actuation
            r.update(deltaT)


    def log_data(self):
        
        # loop over all robots and update their state
        for r in self.robots:
            r.log_data()
            
    def quit(self):
        self.xbee.halt()
        self.serial.close()

    def get_walls(self, robot_id):
        all_walls = [i for i in self.walls] # Copy the walls.
        for i, r in enumerate(self.robots):
            if i != robot_id:
                e = r.state_est
                radius = r.radius
                new_wall_r = E160_wall([e.x+radius, e.y+radius, e.x+radius,
                                        e.y-radius], "vertical")
                new_wall_l = E160_wall([e.x-radius, e.y+radius, e.x-radius,
                                        e.y-radius], "vertical")
                new_wall_t = E160_wall([e.x-radius, e.y+radius, e.x+radius,
                                        e.y+radius], "horizontal")
                new_wall_b = E160_wall([e.x-radius, e.y-radius, e.x+radius,
                                        e.y-radius], "horizontal")
                all_walls += [new_wall_r, new_wall_l, new_wall_t, new_wall_b]
        return all_walls
