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

        # create vars for hardware vs simulation
        # "SIMULATION MODE" or "HARDWARE MODE"
        self.robot_mode = mode
        self.using_leapfrog = True
        self.control_mode = "MANUAL CONTROL MODE"

        # setup xbee communication
        if self.robot_mode == "HARDWARE MODE":
            serial_port1 = serial.Serial('COM9', 9600)
            print(" Setting up serial port")
            try:
                self.xbee = XBee(serial_port1)
            except:
                print("Couldn't find the serial port")

            #serial_port2 = serial.Serial('COM5', 9600)
            #print(" Setting up serial port")
            #try:
            #    self.xbee2 = XBee(serial_port2)
            #except:
            #    print("Couldn't find the serial port")

        # Setup the robots


        self.num_robots = 2
        self.robot_pos = [(0.35, 1, 0), (1, 0, 0)]
        self.robots = []
        self.state_odo = [E160_state() for _ in range(self.num_robots)]
        addresses = ['\x00\x0C', '\x00\x01']
        for i in range(self.num_robots):
            # TODO: assign different address to each bot
            r = E160_robot(self, addresses[i], i)
            self.robots.append(r)
            self.state_odo[i].set_from_tuple(self.robot_pos[i])

        # Pair the two robots up
        self.robots[0].other_pair_id = 1
        self.robots[1].other_pair_id = 0

        # Store the measurements of all robots here.
        self.range_meas = [[0] for _ in self.robots]
        self.encoder_meas = [[0, 0] for _ in self.robots]
        self.last_encoder_meas = [[0, 0] for _ in self.robots]

        self.pf = E160_PF.E160_PF(self, self.robots)

    def update_robots(self, deltaT):

        # loop over all robots and update their state
        for i, r in enumerate(self.robots):
            
            # set the control actuation
            encoder_meas, range_meas = r.update_encoders_and_ranges(deltaT)
            self.encoder_meas[i] = encoder_meas
            self.range_meas[i] = range_meas
        for r in self.robots:
            # This modifies self.last_encoder_meas
            r.update(deltaT)


    def log_data(self):
        
        # loop over all robots and update their state
        for r in self.robots:
            r.log_data()
            
    def quit(self):
        self.xbee.halt()
        self.serial.close()

    def get_walls_leap(self, robot_id, particle_states):
        all_walls = [i for i in self.walls]  # Copy the walls.
        for i, r in enumerate(self.robots):
            if i != robot_id:
                e = E160_state()
                e.set_from_tuple(particle_states[i])
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

    def get_odo(self, robot_id):
        return self.state_odo[robot_id]

    def set_odo(self, robot_id, val):
        self.state_odo[robot_id] = val

    def simulate_range_finder(self, robot_id, states, sensorT):
        """Simulate range readings, given a simulated ground truth state"""
        temp_list = []
        for s in states:
            temp_list.append((s.x, s.y, s.theta))
        p = self.pf.Particle(tuple(temp_list), weight=0)
        walls = self.get_walls_leap(robot_id, p.states)
        return self.pf.FindMinWallDistance(
            p,
            walls,
            sensorT,
            robot_id
        )

    def reset(self):
        for r in self.robots:
            r.cancel_path()

        self.range_meas = [[0] for _ in self.robots]
        self.encoder_meas = [[0, 0] for _ in self.robots]
        self.last_encoder_meas = [[0, 0] for _ in self.robots]
        self.pf.InitializeParticles()
