import math
import random
import numpy as np
import time
import E160_subcluster
import copy
from E160_state import *
from scipy.stats import norm
from E160_rangeconv import range2m, m2range


class E160_PF(object):
    def __init__(self, environment, robots):
        self.particles = []
        self.environment = environment
        self.numParticles = 50

        # maybe should just pass in a robot class?
        self.FAR_READING = range2m(0)

        self.robots = robots

        # PF parameters
        if environment.robot_mode == "HARDWARE MODE":
            self.IR_sigma = m2range(0.0039, scale=True)*10
        else:
            self.IR_sigma = m2range(0.0039, scale=True)
        # Added by Jordan, odometry for distance
        # for each wheel
        self.odom_wheel_std = 0.6 #0.2 #0.3

        self.particle_weight_sum = 0

        # Introducing new particles.
        self.new_random_prop = 0.02 # CHANGE THIS
        self.part_scale = 0#200#200#1/100 #200 #40
        self.flat_rand = 0#40 #10
        # DON'T CHANGE THIS
        #self.new_random_count = int(math.floor(self.numParticles *
         #                                      self.new_random_prop))

        # define the sensor orientations
        #self.sensor_orientation = [-math.pi/2, 0, math.pi/2] #[0] #[
        # -math.pi/2,
        #  0, math.pi/2] #
        # orientations
        #  of the sensors on robot

        # initialize the current state
        self.state = E160_state()

        # TODO: change this later
        self.map_maxX = 1.0
        self.map_minX = -1.0
        self.map_maxY = 1.0
        self.map_minY = -1.0
        self.InitializeParticles()
        self.last_encoder_measurements = [0, 0]

    def InitializeParticles(self):
        ''' Populate self.particles with random Particle
            Args:
                None
            Return:
                None'''
        self.particles = []
        for i in range(0, self.numParticles):
            p = self.Particle(num_bots=len(self.robots),
                              weight=1/self.numParticles)
            #p = self.Particle(0.0, 0.0, 0.0, 1.0/self.numParticles)
            #self.SetRandomStartPos(p)
            self.SetKnownStartPos(p, ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
            self.particles.append(p)

    def GetPDF(self, mu, sd, x):
        """
        Args:
            mu: Expected value (the mean)
            sd: The standard deviation of sensor values
            x: Value to pick from.
        Returns:
            Returns the probability density at a given sensor value
        """
        front = 1/(2.506 * sd)
        power = - 0.5*((x - mu)/sd)**2
        f = front * math.exp(power)
        return f

    def SetRandomStartPos(self, p):
        """
        Set the state of a given particle i to a random value.
        Args:
            p: Particle to set position of.
        """
        p.randomise((self.map_minX, self.map_maxX),
                    (self.map_minY, self.map_maxY))
        #randx = random.random()*(self.map_maxX - self.map_minX) +
        # self.map_minX
        #randy = random.random()*(self.map_maxY - self.map_minY) +
        # self.map_minY
        #randh = self.angleDiff(random.random()*2*math.pi)

        #p.x = randx
        #p.y = randy
        #p.heading = randh

    def SetKnownStartPos(self, p, states):
        """
        Set the state of a given particle i to a known value.
        Args:
            p: Particle to set position of.
        """
        for i, v in enumerate(states):
            p.set_x(i, v[0])
            p.set_y(i, v[1])
            p.set_theta(i, v[2])

        #p.x = x
        #p.y = y
        #p.heading = heading

    def LocalizeEstWithParticleFilter(self,
                                      encoder_measurements,
                                      last_encoder_measurements,
                                      sensor_readings,
                                      robot_id,
                                      angle_reading=None):
        """" Localize the robot with particle filters. Call everything
            Args:
                delta_s (float): change in distance as calculated by odometry
                delta_heading (float): change in heading as calcualted by odometry
                sensor_readings([float, float, float]): sensor readings from range finders
            Return:
                state_est (E160 state): state derived from the average of all particles
        """
        start_time = time.time()

        # Propagate every particle individually.
        tempsum = 1.0e-100
        for i in range(len(self.particles)):
            self.Propagate(robot_id,
                           encoder_measurements,
                           last_encoder_measurements,
                           i)
            # TODO: CHANGE THIS TO A MULTIROBOT SOLUTION
            if self.environment.using_leapfrog:
                self.particles[i].weight = self.CalculateWeight(
                    sensor_readings[0],
                    self.environment.get_walls_leap(0,
                                                    self.particles[i].states),
                    i,
                    0
                )
                self.particles[i].weight += self.CalculateWeight(
                    sensor_readings[1],
                    self.environment.get_walls_leap(1,
                                                    self.particles[i].states),
                    i,
                    1
                )
            else:
                self.particles[i].weight = self.CalculateWeight(
                    sensor_readings,
                    self.environment.get_walls(0),
                    i,
                    self.environment.robots[0]
                )
            tempsum += self.particles[i].weight
        self.particle_weight_sum = tempsum

        # Resample particles to get a new set of particles
        diff_encoder_l = (encoder_measurements[0] -
                          last_encoder_measurements[0])
        diff_encoder_r = (encoder_measurements[1] -
                          last_encoder_measurements[1])
        encoder_mag = diff_encoder_r**2 + diff_encoder_l**2
        #print(diff_e, diff_encoder_r)
        # Make sure we actually are moving
        if encoder_mag > 0.05:
            # Use approximate particle sampling method
            #particle_samp = []
            #for i in range(len(self.particles)):
            #    relative_weight = (self.particles[i].weight
            #                       / self.particle_weight_sum)
            #    if relative_weight < 0.5/self.numParticles:
            #        particle_samp.append(self.particles[i])
            #    elif relative_weight < 1.0/self.numParticles:
            #        particle_samp.append(self.particles[i])
            #        particle_samp.append(self.particles[i])
            #    elif relative_weight < 1.5/self.numParticles:
            #        particle_samp.append(self.particles[i])
            #        particle_samp.append(self.particles[i])
            #        particle_samp.append(self.particles[i])
            #    else:
            #        # Add four copies
            #        particle_samp.append(self.particles[i])
            #        particle_samp.append(self.particles[i])
            #        particle_samp.append(self.particles[i])
            #        particle_samp.append(self.particles[i])

            new_particles = []
            #new_random_count = int(math.floor(self.numParticles
            #                                  * self.new_random_prop))
            new_random_count = int(math.floor(self.new_random_prop *
                                              self.part_scale
                                              / self.particle_weight_sum +
                                              self.flat_rand))
            new_random_count = min(self.numParticles, new_random_count)
            self.AddRandomPoints(new_random_count, new_particles)
            for i in range(self.numParticles-new_random_count):
                new_particles.append(self.Resample())
                #chosen_p = random.choice(particle_samp)
                #new_p = self.Particle(chosen_p.x, chosen_p.y,
                #                      chosen_p.heading, chosen_p.weight)
                #new_particles.append(new_p)

            self.particles = new_particles

        # At the end, update our encoder measurements
        return self.GetEstimatedPos(robot_id)

    def Propagate(self, robot_id, encoder_measurements,
                  last_encoder_measurements, i):
        """
        Propagate all the particles from the last state with odometry readings
            Args:
                robot_id (int): Robot id of robot to propagate forwards
                encoder_measurements (wheel1, wheel2):
                i: Particle index
            return:
                None
        """
        # add student code here
        robot = self.environment.robots[robot_id]
        lastEncoderL = last_encoder_measurements[0]
        lastEncoderR = last_encoder_measurements[1]

        # Get the change in encoder readings.
        diff_encoder_l = encoder_measurements[0] - lastEncoderL
        diff_encoder_r = encoder_measurements[1] - lastEncoderR

        #print("lastEncoderL {}".format(lastEncoderL))

        # Introduce noise into the wheel odometry.
        noise_l = np.random.normal(1, self.odom_wheel_std)
        noise_r = np.random.normal(1, self.odom_wheel_std)

        # Calculate the distance each wheel travels, and apply noise.
        wheel_distance_l = (
            ((2*math.pi*diff_encoder_l * robot.wheel_radius) /
             robot.encoder_resolution) * noise_l
        )
        wheel_distance_r = (
                ((2*math.pi*diff_encoder_r * robot.wheel_radius) /
                 robot.encoder_resolution) * noise_r
        )

        # Calculate delta_s
        delta_s = 0.5 * (wheel_distance_r + wheel_distance_l)
        # Calculate delta_theta 
        delta_theta = 0.5 * (wheel_distance_r - wheel_distance_l)/robot.radius

        # Update the state estimate of the particle.
        self.update_particle_state(i, delta_s, delta_theta, robot_id=robot_id)

    def update_particle_state(self, i, del_s, del_theta, robot_id):
        """
        Args:
            i (int): Particle index
            del_s (float): Change in forward distance
            del_theta (float): Change in angle
            robot_id (int): ID of the robot in self.robots
        Returns:
            None
        """
        particle = self.particles[i]
        px = particle.get_x(robot_id)
        py = particle.get_y(robot_id)
        theta = particle.get_theta(robot_id)

        #theta = self.particles[i].heading

        particle.set_x(robot_id, px + del_s * math.cos(theta + 0.5*del_theta))
        particle.set_y(robot_id, py + del_s * math.sin(theta + 0.5*del_theta))
        particle.set_theta(robot_id, self.angleDiff(theta + del_theta))

        #particle.x += del_s * math.cos(theta + 0.5*del_theta)
        #particle.y += del_s * math.sin(theta + 0.5*del_theta)
        #particle.heading = self.angleDiff(theta + del_theta)

    def CalculateWeight(self, sensor_readings, walls, particle_index,
                        robot_id):
        '''Calculate the weight of a particular particle
            Args:
                particle (E160_Particle): a given particle
                sensor_readings ( [float, ...] ): readings from the IR sensors
                                                  for only this robot.
                walls ([ [four doubles], ...] ): positions of the walls from
                                                 environment,
                                                 represented as 4 doubles
            return:
                new weight of the particle (float) '''
        particle = self.particles[particle_index]
        expected = []
        robot = self.environment.robots[robot_id]
        for o in robot.sensor_orientation:
            expected.append(self.FindMinWallDistance(particle, walls, o,
                                                     robot_id))

        # We need the probability of getting these sensor readings at this
        # given state.

        # Find probability of getting IR sensor reading given the particle's
        # distance from the wall. We assume a gaussian distribution with the
        # mean and std derived in Lab 1.
        distance_sensor_prob = 1
        for i in range(len(expected)):
            # Mean of distribution
            mu = m2range(expected[i])
            sd = self.IR_sigma
            distance_sensor = sensor_readings[i]
            # Probability density of particle given distance sensor
            addition = math.pow(self.GetPDF(mu, sd, distance_sensor),
                                1/len(expected))
            if mu < m2range(self.FAR_READING) + 5:
                addition *= 0.25 #0.1
            distance_sensor_prob *= addition

            px = particle.get_x(0)
            py = particle.get_y(0)

            if (px < self.map_minX or px > self.map_maxX
                    or py < self.map_minY or py > self.map_maxY):
                distance_sensor_prob = 0

        newWeight = distance_sensor_prob
        return newWeight

    def Resample(self):
        '''Resample the particles systematically
            Args:
                None
            Return:
                particle (E160_particle): a given particle 
        '''
        # add student code here 
        current_sum = 0
        random_num = random.random()*self.particle_weight_sum 
        chosen_particle = 0
        for i in range(len(self.particles)):
            current_sum += self.particles[i].weight 
            if random_num < current_sum:
                chosen_particle = i
                break
        # end student code here
        chosen = self.particles[chosen_particle]
        newpart = chosen.deepcopy()
        #newpart = self.Particle(chosen.x, chosen.y, chosen.heading,
                                #1)
        #                        chosen.weight)
        return newpart

    def GetEstimatedPos(self, robot_id):
        ''' Calculate the mean of the particles and return it
            Args:
                None
            Return:
                state (E160 robot state): Estimated state from the average of the particles.'''
        # add student code here
        # Calculate state estimate by taking average of particles
        mode = E160_subcluster.subcluster(self.particles, 0.2, robot_id)
        robot_state = E160_state(mode.get_x(robot_id), mode.get_y(robot_id),
                                 mode.get_theta(robot_id))
        return robot_state

    def FindMinWallDistance(self, particle, walls, sensorT, robot_id):
        ''' Given a particle position, walls, and a sensor, find
            shortest distance to the wall
            Args:
                particle (E160_Particle): a particle
                walls ([E160_wall, ...]): represents endpoint of the wall
                sensorT: orientation of the sensor on the robot
            Return:
                distance to the closest wall' (float)'''
        current_min = self.FAR_READING
        for w in walls:
            p = []
            for s in range(len(w.points)//2):
                p.append((w.points[s*2], w.points[s*2+1]))

            segment1 = (p[0][0], p[0][1], p[1][0], p[1][1]) # Top segment
            segment2 = (p[1][0], p[1][1], p[2][0], p[2][1]) # Right segment
            segment3 = (p[2][0], p[2][1], p[3][0], p[3][1]) # Bottom segment
            segment4 = (p[3][0], p[3][1], p[0][0], p[0][1]) # Left segment

            seg1dist = self.FindWallDistance(particle, segment1, sensorT,
                                             robot_id)
            seg2dist = self.FindWallDistance(particle, segment2, sensorT,
                                             robot_id)
            seg3dist = self.FindWallDistance(particle, segment3, sensorT,
                                             robot_id)
            seg4dist = self.FindWallDistance(particle, segment4, sensorT,
                                             robot_id)

            newdist = min(seg1dist, seg2dist, seg3dist, seg4dist)
            if newdist < current_min:
                current_min = newdist
        #print("IN WALLS FINDER -----------")
        #print("id: {}, current_min: {}".format())
        return current_min

    def FindWallDistance(self, particle, wall, sensorT, robot_id):
        ''' Given a particle position, a wall, and a sensor, find distance to the wall
            Args:
                particle (E160_Particle): a particle
                wall ([float x4]): represents endpoint of the wall
                sensorT: orientation of the sensor on the robot
            Return:
                distance to the closest wall (float)'''
        raydir = particle.get_theta(robot_id) + sensorT
        px = particle.get_x(robot_id)
        py = particle.get_y(robot_id)
        raygrad = math.tan(raydir)
        ray_y_int = -raygrad * px + py

        if raygrad == 0:
            raygrad = 1e-100

        try:
            wallgrad = (wall[1]-wall[3])/(wall[0]-wall[2])
        except ZeroDivisionError:
            wallgrad = 100000000
        if wallgrad == 0:
            wallgrad = 1.1e-100
        wall_y_int = -wallgrad*wall[0]+wall[1]

        # Find the intercept of these two lines
        x_cross = -(wall_y_int - ray_y_int)/(wallgrad - raygrad)
        y_cross = ((ray_y_int/raygrad - wall_y_int/wallgrad)
                   / (1/raygrad - 1/wallgrad))
        if not self.pointWithinViewport(
                x_cross,
                y_cross,
                particle.get_x(robot_id),
                particle.get_y(robot_id),
                particle.get_theta(robot_id),
                wall,
                sensorT):
            return self.FAR_READING
        # Return the distance if we got the far reading.
        d = self.dist(px, py, x_cross, y_cross)
        return d

    def pointWithinViewport(self, x, y, partx, party, parttheta,
                            viewport, sensor_t):
        dx = x - partx #particle.get_x()
        dy = y - party #particle.y
        theta = parttheta #particle.heading
        point_angle = self.angleDiff(math.atan2(dy, dx))
        view1_angle = math.atan2(viewport[1] - party, viewport[0] -
                                 partx)
        view2_angle = math.atan2(viewport[3] - party, viewport[2] -
                                 partx)

        # If the point is behind the sensor, we should consider that outside
        # the viewport.
        if abs(self.angleDiff(point_angle - sensor_t - theta)) > math.pi*0.5:
            return False

        # Check if we straddle the backwards line.
        straddling = (abs(view1_angle) > math.pi/2
                      and abs(view2_angle) > math.pi/2
                      and ((view1_angle > 0) != (view2_angle > 0)))

        if straddling:
            return (abs(point_angle) > abs(view2_angle)
                    and abs(point_angle) > abs(view1_angle))

        if view1_angle >= view2_angle:
            return view1_angle >= point_angle >= view2_angle
        else:
            return view1_angle <= point_angle <= view2_angle

    def AddRandomPoints(self, count, l):
        for i in range(count):
            p = self.Particle(num_bots=2)
            p.weight = self.particle_weight_sum/self.numParticles
            #p = self.Particle(0, 0, 0,
            #                  self.particle_weight_sum/self.numParticles)
            self.SetRandomStartPos(p)
            l.append(p)

    def dist(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)

    def angleDiff(self, ang):
        ''' Wrap angles between -pi and pi'''
        while ang < -math.pi:
            ang = ang + 2 * math.pi
        while ang > math.pi:
            ang = ang - 2 * math.pi
        return ang

    class Particle:
        def __init__(self, states=None, weight=0, **kwargs):
            try:
                self.states = list(map(list, states))
            except TypeError:
                if "num_bots" in kwargs:
                    num_bots = kwargs["num_bots"]
                    self.states = [[0, 0, 0] for _ in range(num_bots)]
                else:
                    raise ValueError()
            #self.x = x
            #self.y = y
            #self.heading = heading
            self.weight = weight

        def set_states(self, states):
            self.states = list(map(list, states))

        def get_x(self, robot_id):
            return self.states[robot_id][0]

        def get_y(self, robot_id):
            return self.states[robot_id][1]

        def get_theta(self, robot_id):
            return self.states[robot_id][2]

        def set_x(self, robot_id, value):
            self.states[robot_id][0] = value

        def set_y(self, robot_id, value):
            self.states[robot_id][1] = value

        def set_theta(self, robot_id, value):
            self.states[robot_id][2] = value

        def randomise(self, x_range, y_range):
            map_minX = x_range[0]
            map_maxX = x_range[1]
            map_maxY = y_range[0]
            map_minY = y_range[1]
            for i in range(len(self.states)):
                randx = random.random()*(map_maxX - map_minX) + map_minX
                randy = random.random()*(map_maxY - map_minY) + map_minY
                randh = self.angleDiff(random.random()*2*math.pi)
                self.set_x(i, randx)
                self.set_y(i, randy)
                self.set_theta(i, randh)

        def deepcopy(self):
            newstates = [(self.get_x(i),
                          self.get_y(i),
                          self.get_theta(i))
                         for i in range(len(self.states))]
            newpart = E160_PF.Particle(newstates, weight=self.weight)
            return newpart

        def __str__(self):
            message = ""
            for s in range(len(self.states)):
                message += (
                    "(" + str(self.get_x(s)) + " " + str(self.get_y(s)) + " "
                    + str(self.get_theta(s)) + ")"
                )
                message += " "
            return message

        def angleDiff(self, ang):
            ''' Wrap angles between -pi and pi'''
            while ang < -math.pi:
                ang = ang + 2 * math.pi
            while ang > math.pi:
                ang = ang - 2 * math.pi
            return ang