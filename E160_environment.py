from E160_robot import *
from E160_state import *
from E160_wall import *
import E160_PF

import serial
import datetime
import time
from xbee import XBee

class E160_environment:
    
    def __init__(self, mode="SIMULATION MODE"):
        #self.width = 2.0
        #self.height = 1.2
        self.width = 6.0
        self.height = 3.0

        # set up walls, putting top left point first
        self.walls = []
        #self.walls.append(E160_wall([-0.5, 0.3, -0.5, -0.3], "vertical"))
        #self.walls.append(E160_wall([1, 0.8, 1, -0.3], "vertical"))
        self.walls.append(E160_wall([0, 1, 15, 1], "horizontal"))
        self.walls.append(E160_wall([0, -1, 15, -1], "horizontal"))
        #self.walls.append(E160_wall([-1, 0.8, -1, -0.3], "vertical"))

        # create vars for hardware vs simulation
        # "SIMULATION MODE" or "HARDWARE MODE"
        self.robot_mode = mode
        self.using_leapfrog = True
        self.control_mode = "MANUAL CONTROL MODE"

        # setup xbee communication
        if self.robot_mode == "HARDWARE MODE":
            self.serial = serial.Serial('COM9', 9600)
            print("Setting up serial port")
            try:
                self.xbee = XBee(self.serial)
                print("Found Computer XBee")
            except:
                print("Couldn't find the serial port")

            #serial_port2 = serial.Serial('COM5', 9600)
            #print(" Setting up serial port")
            #try:
            #    self.xbee2 = XBee(serial_port2)
            #except:
            #    print("Couldn't find the serial port")

        # Setup the robots

        self.file_name = 'Log/{}_All_Bots' '_' \
                         + datetime.datetime.now() \
                             .replace(microsecond=0) \
                             .strftime('%y-%m-%d %H.%M.%S') + '.txt'
        self.file_name = self.file_name.format(self.robot_mode)

        self.num_robots = 2
        if self.num_robots == 1:
            self.robot_pos = [(0, 0, 0)]
        elif self.num_robots == 2:
            self.robot_pos = [(0.5, 0, 0), (0, 0, 0)]
        self.robots = []
        self.state_odo = [E160_state() for _ in range(self.num_robots)]
        addresses = ['\x00\x0C', '\x00\x01']
        for i in range(self.num_robots):
            # TODO: assign different address to each bot
            r = E160_robot(self, addresses[i], i)
            self.robots.append(r)
            self.state_odo[i].set_from_tuple(self.robot_pos[i])

        # Pair the two robots up
        if self.num_robots == 2:
            self.robots[0].other_pair_id = 1
            self.robots[1].other_pair_id = 0

        # Store the measurements of all robots here.
        self.range_meas = [[0] for _ in self.robots]
        self.encoder_meas = [[0, 0] for _ in self.robots]
        self.last_encoder_meas = [[0, 0] for _ in self.robots]
        self.bearing_from_other = [0 for _ in self.robots]

        self.pf = E160_PF.E160_PF(self, self.robots)

        self.make_header(self.num_robots)

    def update_robots(self, deltaT):

        # loop over all robots and update their state
        for i, r in enumerate(self.robots):
            # set the control actuation
            encoder_meas, range_meas, camera_angle =\
                r.update_encoders_and_ranges(deltaT)
            self.encoder_meas[i] = encoder_meas
            self.range_meas[i] = range_meas
            if r.other_pair_id is not None:
                self.bearing_from_other[r.other_pair_id] = camera_angle
                #print("Robot {} found... {}".format(i, camera_angle))
        for r in self.robots:
            # This modifies self.last_encoder_meas
            r.update(deltaT)

        #print("BEARING: {}".format(self.bearing_from_other))


    def log_data(self):
        self._log_line(self.num_robots)
            
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
                #print("new_wall_r {}".format(new_wall_r))
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
        #print("WALLS: {}".format(walls))
        return self.pf.FindMinWallDistance(
            p,
            walls,
            sensorT,
            robot_id
        )

    def simulate_camera_angle(self, robot_id, states):
        """Simulate camera angle readings, given simulated ground truth states.
            robot_id is the observer_state
        """
        observer_state = states[robot_id]
        observed_state = states[robot_id^1]

        x_diff = observed_state.x - observer_state.x
        y_diff = observed_state.y - observer_state.y 

        observed_angle = math.atan2(y_diff, x_diff)
        lower_angle = observed_angle - 0.1745
        upper_angle = observed_angle + 0.1745

        if observer_state.theta < upper_angle and observer_state.theta > lower_angle:
           camera_angle = 0
        else:
           camera_angle = None     

        return camera_angle
 
    def reset(self):
        for r in self.robots:
            r.cancel_path()

        self.range_meas = [[0] for _ in self.robots]
        self.encoder_meas = [[0, 0] for _ in self.robots]
        self.last_encoder_meas = [[0, 0] for _ in self.robots]
        self.pf.InitializeParticles()

    def make_header(self, num_robots):
        mesg = ""
        for i in range(num_robots):
            mesg += "Range_r{},".format(i)
            mesg += "R_enc_r{},".format(i)
            mesg += "L_enc_r{},".format(i)
            mesg += "x_odo_r{},".format(i)
            mesg += "y_odo_r{},".format(i)
            mesg += "theta_odo_r{},".format(i)
            mesg += "x_r{},".format(i)
            mesg += "y_r{},".format(i)
            mesg += "theta_r{},".format(i)
            mesg += "bearing_r{},".format(i)

        mesg += "time\n"
        f = open(self.file_name, 'a+')
        f.write(mesg)
        f.close()

    def _log_line(self, num_robots):
        mesg = ""
        for i in range(num_robots):
            mesg += str(self.range_meas[i][0]) + ","
            mesg += str(self.encoder_meas[i][0]) + ","
            mesg += str(self.encoder_meas[i][1]) + ","
            mesg += str(self.state_odo[i].x) + ","
            mesg += str(self.state_odo[i].y) + ","
            mesg += str(self.state_odo[i].theta) + ","
            mesg += str(self.robots[i].state_est.x) + ","
            mesg += str(self.robots[i].state_est.y) + ","
            mesg += str(self.robots[i].state_est.theta) + ","
            mesg += str(self.bearing_from_other[i]) + ","
        mesg += str(time.time()) + "\n"
        f = open(self.file_name, 'a+')
        f.write(mesg)
        f.close()
