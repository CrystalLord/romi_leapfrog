
from E160_state import *
from E160_PF import E160_PF
from color_tracking import cameraTracking

import E160_medianfilter
import E160_rangeconv
import math
import datetime
import time

class E160_robot:

    def __init__(self, environment, address, robot_id, other_pair_id=None,
                 position=None):
        self.debug = False

        self.environment = environment
        self.state_est = E160_state()
        self.state_des = E160_state()
        if position is None:
            self.state_est.set_state(0, 0, 0)
            self.state_des.set_state(0, 0, 0)
        else:
            self.state_est.set_state(position[0], position[1], position[2])
            self.state_des.set_state(position[0], position[1], position[2])

        # Where are we drawing the robot on the GUI?
        self.state_draw = E160_state()
        #self.state_odo = E160_state()
        #self.state_odo.set_state(0, 0, 0) # real position for simulation
        # --------------------------------------------------------------------

        self.R = 0
        self.L = 0
        self.radius = 0.147 / 2
        self.width = 2*self.radius
        self.wheel_radius = 0.035
        self.address = address
        self.ID = self.address.encode().__str__()[-1]
        self.last_measurements = []
        self.robot_id = robot_id
        self.other_pair_id = other_pair_id
        self._manual_control_left_motor = 0
        self._manual_control_right_motor = 0

        self.encoder_resolution = 1440

        #self.last_encoder_measurements = [0, 0]
        #self.encoder_measurements = [0, 0]
        #self.range_measurements = [0, 0, 0]
        # Orientations of the sensors on the robot
        self.sensor_orientation = [0] #[-math.pi/4, 0, math.pi/4]
        self.last_simulated_encoder_R = 0
        self.last_simulated_encoder_L = 0
        
        self.Kpho = 1#1.0
        self.Kalpha = 2#2.0
        self.Kbeta = -0.5#-0.5

        self.rotate_gain = 12

        if self.environment.robot_mode == "HARDWARE MODE":
            self.max_velocity = 0.08
            self.camera = cameraTracking(robot_id+1)
        else:
            self.max_velocity = 1
        self.max_angular_vel = 3
        self.point_tracked = True
        self.encoder_per_sec_to_rad_per_sec = 10

        # New for Lab 4 ------------------------------------------------------
        #self.PF = E160_PF(environment, self.width, self.wheel_radius,
        # self.encoder_resolution)
        # --------------------------------------------------------------------

        # For lab 3
        # ADDED BY JORDAN R ABRAHAMS
        self.momentum = 0
        # Conversion between desired angular velocities and actual
        # wheel speed in m/s. This value is measured empirically.
        self.angular_to_wheel_conv = 440

        # ADDED BY M SANGHEETHA NAIDU
        self.path = None
        self.is_following_path = False
        self.is_rotation_tracking = False
        self.use_full_path_distance = False
        self.use_median_filter = True
        self.median_filters = [E160_medianfilter.MedianFilter(4) for _ in
                               range(3)]
        #self.points = [[0.2,-0.1,-1.57]]
                      # [0.0, 0.0,0.0],
                      # [0.20,0.0,1.57], 
                      # [0.20, 0.20, 1.57],
                      # [0.0, 0.0, 0.0]]

    def update_encoders_and_ranges(self, delta_t):
        """
        Slightly different from normal update.
        Because we have to collect all the information from all robots
        before doing localisation, we need to call this separately, before
        update for all robots.
        Args:
            delta_t (float): timestep size

        Returns:
            Tuple of (encoder measurements, range measurements)
        """
        encoder_meas, range_meas, camera_angle = \
            self.update_sensor_measurements(delta_t)

        return encoder_meas, range_meas, camera_angle

    def update(self, deltaT):
        # get sensor measurements
            #self.encoder_measurements, self.range_measurements =\
            #    self.update_sensor_measurements(deltaT)
        #self.encoder_measurements, range_mes =\
        #    self.update_sensor_measurements(deltaT)
        #self.environment.range_measurements[self.robot_id] = range_mes

        encoder_measurements = self.environment.encoder_meas[self.robot_id]
        range_measurements = self.environment.range_meas
        bearing_from_other = self.environment.bearing_from_other
        # Retrieve the
        last_encoder_measurements =\
            self.environment.last_encoder_meas[self.robot_id]

        # update odometry
        delta_s, delta_theta = self.update_odometry(encoder_measurements,
                                                    last_encoder_measurements)


        # update simulated real position, find ground truth for simulation
        state_odo = self.environment.get_odo(self.robot_id)
        state_odo = self.localize(state_odo, delta_s, delta_theta)
        self.environment.set_odo(self.robot_id, state_odo)

        # localize with particle filter
        if self.environment.robot_mode == "HARDWARE MODE":
            self.state_est =\
                self.environment.pf.LocalizeEstWithParticleFilter(
                    encoder_measurements,
                    last_encoder_measurements,
                    [[i[1]] for i in range_measurements],
                    bearing_from_other,
                    self.robot_id
                )
        else:
            #print("range_measurements: {}".format(range_measurements))
            self.state_est = self.environment.pf.LocalizeEstWithParticleFilter(
                encoder_measurements,
                last_encoder_measurements,
                range_measurements,
                bearing_from_other,
                self.robot_id
            )

        # Update the environment's encoder measurements
        self.environment.last_encoder_meas[self.robot_id] = \
            encoder_measurements
        # Old system of updating encoder measurements which are local.
        #self.last_encoder_measurements = self.encoder_measurements
        # to out put the true location for display purposes only.
        self.state_draw = self.environment.get_odo(self.robot_id)

        # call motion planner
        #self.motion_planner.update_plan()
        
        # determine new control signals
        self.R, self.L = self.update_control()
        
        # send the control measurements to the robot
        self.send_control(self.R, self.L, deltaT)

    def update_sensor_measurements(self, deltaT):
        
        if self.environment.robot_mode == "HARDWARE MODE":
            command = '$S @'
            self.environment.xbee.tx(dest_addr=self.address, data=command)

            update = self.environment.xbee.wait_read_frame()

            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            # We flip these because the robot is driving backwards technically.
            encoder_measurements = list(reversed(data[-2:]))
            range_measurements = data[:-2]

            camera_angle = self.camera.getAngle()

        elif self.environment.robot_mode == "SIMULATION MODE":
            encoder_measurements = self.simulate_encoders(self.R, self.L,
                                                          deltaT)
            # New in lab 4
            range_measurements = []
            for o in self.sensor_orientation:
                new_reading = self.environment.simulate_range_finder(
                    self.robot_id,
                    self.environment.state_odo,
                    o
                )
                range_measurements.append(new_reading)
            range_measurements = list(map(E160_rangeconv.m2range,
                                          range_measurements))
            # TODO: Change this to the simulated camera angle.
            camera_angle = self.environment.simulate_camera_angle(
                    self.robot_id,
                    self.environment.state_odo)

        if self.use_median_filter:
            range_measurements = [self.median_filters[i].filter(x)
                                  for i, x in enumerate(range_measurements)]
        return encoder_measurements, range_measurements, camera_angle

    def localize(self, state_est, delta_s, delta_theta):
        # New lab 4 state estimate function. We must be given delta_s and
        # delta_theta now.
        state_est = self.update_state(state_est, delta_s, delta_theta)
        return state_est
    
    def angle_wrap(self, a):
        while a > math.pi:
            a = a - 2*math.pi
        while a < -math.pi:
            a = a + 2*math.pi
            
        return a
        
    def update_control(self):

        # Initialise to zero
        desiredWheelSpeedR = 0
        desiredWheelSpeedL = 0

        if self.environment.control_mode == "MANUAL CONTROL MODE":
            desiredWheelSpeedR = self._manual_control_right_motor
            desiredWheelSpeedL = self._manual_control_left_motor
        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":
            if self.is_following_path:
                # Follow the entire path
                desiredWheelSpeedR, desiredWheelSpeedL = self.path_tracker_control()
            else:
                # Follow only next point.
                desiredWheelSpeedR, desiredWheelSpeedL = self.point_tracker_control()
        elif self.environment.control_mode == "LEAP PATH CONTROL MODE":
            if self.is_following_path:
                desiredWheelSpeedR, desiredWheelSpeedL =\
                    self.path_tracker_control()
            elif self.is_rotation_tracking:
                desiredWheelSpeedR, desiredWheelSpeedL =\
                    self.rotation_track(self.other_pair_id)
        else:
            raise ValueError()

        return desiredWheelSpeedR, desiredWheelSpeedL
  
    def path_tracker_control(self):
        """ Takes in a list of points, where each point consists of x, y 
            and theta. Iteratively calls point_tracker_control() to take
            robot on path."""
        if len(self.path) == 0:
            return 0, 0
        else:
            self.state_des.x = self.path[0][0]
            self.state_des.y = self.path[0][1]
            self.state_des.theta = self.path[0][2]
            self.point_tracked = False
            print("go to first point: ", self.state_des.x)

            if len(self.path) == 1:
                desiredWheelSpeedR, desiredWheelSpeedL =\
                    self.point_tracker_control()
            else:
                desiredWheelSpeedR, desiredWheelSpeedL = \
                    self.point_tracker_control(track_theta=False)
            if self.debug:
                print("Desired Wheel Speeds:", desiredWheelSpeedR,
                      desiredWheelSpeedL)
            if self.point_tracked:
                self.path = self.path[1:]

        return desiredWheelSpeedR, desiredWheelSpeedL
    
    def point_tracker_control(self, track_theta=True, use_full_path=True):
        """Set desired wheel positions from a given desired point"""
        # Local gains, set to environment variables later
        distGain = 1.8
        alphaGain = 10
        betaGain = -0.5

        velNom = 0.4

        # What are the thresholds for reaching the target? When are we close
        # enough?
        distThresh = 0.05
        thetaThresh = 0.06

        # Shorthands for our current state
        curX = self.state_est.x
        curY = self.state_est.y
        curTheta = self.state_est.theta
        delx, dely, deltheta = (self.state_des.x - curX,
                                self.state_des.y - curY,
                                self.norm_theta(self.state_des.theta
                                                - curTheta))
        if not track_theta:
            deltheta = 0

        # Get distance to target.
        distTarget = math.sqrt(delx**2 + dely**2)
        if not use_full_path:
            rho = distTarget
        else:
            rho = self.length_along_path()
        # Is the target point in front or behind the robot?
        dirX = math.cos(curTheta)
        dirY = math.sin(curTheta)
        dot = self.norm_dotprod(dirX, dirY, delx, dely)
        print("IN FRONT?: ", dot > 0)

        if abs(distTarget) < distThresh:
            onlyRotate = True
            if abs(deltheta) < thetaThresh:
                print("TRACKED! --------")
                self.point_tracked = True
        else:
            onlyRotate = False

        print("distTarget      :", distTarget)
        print("rho      :", rho)
        print("deltheta :", math.degrees(deltheta))
        print("thetaThresh :", math.degrees(thetaThresh))

        # If the desired point is not tracked yet, then track it
        if not self.point_tracked:
            # We are going to the desired point.
            # Check if we should go forwards or backwards
            goForwards = dot > 0 and self.momentum > -0.05

            if goForwards:
                # Get angle towards the target
                alpha = self.norm_theta(-curTheta
                        + self.norm_theta(math.atan2(dely * dot, delx * dot)))
                # Set drive amount
                desV = (distGain*rho + velNom)
            else:
                # Reverse the angle if the target is behind us.
                alpha = self.norm_theta(
                    -curTheta + self.norm_theta(math.atan2(dely * dot,
                                                           delx * dot)))
                # Reverse the drive direction.
                desV = (distGain*rho + velNom) * -1

            # Get reverse change of angle.
            beta = self.norm_theta(self.norm_theta(-curTheta - alpha)+self.state_des.theta)
            # Compute the desired angular velocity.
            desW = alphaGain*alpha + betaGain*beta

            print("alpha    :", math.degrees(alpha))
            print("beta     :", math.degrees(beta))

            if onlyRotate:
                print("ONLY ROTATING------")
                desW = deltheta * self.rotate_gain
                desV = 0

            # Add nominal angular velocity
            #desW = desW + math.copysign(desW, 1) * rotNom

            print("desW     :", desW)
            print("desV     :", desV)

            desiredWheelSpeedR, desiredWheelSpeedL = self.get_des_wheel(
                desV, desW
            )
            #desiredWheelSpeedR = 2*(desV + desW*self.width) \
            #        /self.wheel_radius * -1
            #desiredWheelSpeedL = 2*(-desV + desW*self.width) \
            #        /self.wheel_radius

        # the desired point has been tracked, so don't move
        else:
            desiredWheelSpeedR = 0
            desiredWheelSpeedL = 0

        # Make sure to cap the maximum wheel velocities.
        desiredWheelSpeedR, desiredWheelSpeedL = self.cap_wheels(
            (desiredWheelSpeedR, desiredWheelSpeedL), self.max_velocity)

        print("WR:{}, WL:{}".format(desiredWheelSpeedR, desiredWheelSpeedL))
        print("-----------")

        return desiredWheelSpeedR, desiredWheelSpeedL

    def get_des_wheel(self, desV, desW):
        desiredWheelSpeedR = 2*(desV + desW*self.width) \
                             /self.wheel_radius * -1
        desiredWheelSpeedL = 2*(-desV + desW*self.width) \
                             /self.wheel_radius
        desiredWheelSpeedR, desiredWheelSpeedL = self.cap_wheels(
            (desiredWheelSpeedR, desiredWheelSpeedL), self.max_velocity
        )
        return desiredWheelSpeedR, desiredWheelSpeedL

    def send_control(self, R, L, deltaT):
        # send to actual robot !!!!!!!!
        if self.environment.robot_mode == "HARDWARE MODE":
            if (L < 0):
                LDIR = 0
            else:
                LDIR = 1

            if (R < 0):
                RDIR = 0
            else:
                RDIR = 1
            RPWM = int(abs(R))
            LPWM = int(abs(L))
            command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            self.environment.xbee.tx(dest_addr = self.address, data = command)

    def simulate_encoders(self, R, L, deltaT):
        last_r = self.last_simulated_encoder_R
        last_l = self.last_simulated_encoder_L
        right_encoder_measurement = -int(
            R*self.encoder_per_sec_to_rad_per_sec*deltaT) + last_r
        left_encoder_measurement = -int(
            L*self.encoder_per_sec_to_rad_per_sec*deltaT) + last_l
        self.last_simulated_encoder_R = right_encoder_measurement
        self.last_simulated_encoder_L = left_encoder_measurement

        return [left_encoder_measurement, right_encoder_measurement]

    #def make_headers(self):
    #    f = open(self.file_name, 'a+')
    #    f.write('{0} {4:^1} \n'
    #            .format('Range_r1', 'range_r2', 'R3', 'RW', 'LW'))
    #    # Old Lab 3 log headers.
    #    f.write('{6} {0} {1:^1} {2:^1} {3:^1} {4:^1} {5:^1} \n'.format('X', 'Y', 'Theta', 'Des_X', 'Des_Y','Des_Theta', 'time'))
    #    f.close()


        
    # def log_data(self):
    #    f = open(self.file_name, 'a+')
    #
    #    # edit this line to have data logging of the data you care about
    #    #data = [str(x) for x in [1,2,3,4,5]]
    #    # Old lab 3 log data.
    #    state_odo = self.environment.get_odo(self.robot_id)
    #    data = [str(x) for x in [time.time(), self.state_est.x,
    #                             self.state_est.y, self.state_est.theta,
    #                             state_odo.x, state_odo.y,
    #                             state_odo.theta,
    #                             self.environment.pf.particle_weight_sum,
    #                             self.environment.range_meas[self.robot_id],
    #                             self.environment.pf.numParticles]]
    #
    #    f.write(' '.join(data) + '\n')
    #    f.close()
        
    def set_manual_control_motors(self, R, L):
        self._manual_control_right_motor = int(R*256/100)
        self._manual_control_left_motor = int(L*256/100)

    def update_odometry(self, encoder_measurements, last_encoder_measurements):
        """From Lab 2, update the possible state from encoder measurements."""
        delta_s = 0
        delta_theta = 0

        # ****************** Additional Student Code: Start ************

        # Calculate encoder differences
        encoderL, encoderR = encoder_measurements
        lastencoderL, lastencoderR = last_encoder_measurements

        diffEncoderL = (encoderL - lastencoderL)
        diffEncoderL = diffEncoderL if diffEncoderL <= 1000 else 0
        diffEncoderR = (encoderR - lastencoderR)
        diffEncoderR = diffEncoderR if diffEncoderR <= 1000 else 0

        # remember for next time

        #self.last_encoder_measurements[0] = encoderL
        #self.last_encoder_measurements[1] = encoderR

        # Calculate distance travelled by each wheel.
        wheelDistanceL = (2*math.pi*diffEncoderL*self.wheel_radius)/self.encoder_resolution
        wheelDistanceR = (2*math.pi*diffEncoderR*self.wheel_radius)/self.encoder_resolution

        # Calculate delta_s
        delta_s = 0.5 * (wheelDistanceR + wheelDistanceL)
        # Calculate delta_theta (This only works if wheel0 is right wheel,
        # swap values if reversed)
        delta_theta = 0.5 * (wheelDistanceR - wheelDistanceL)/self.radius
        # ****************** Additional Student Code: End ************

        # keep this to return appropriate changes in distance, angle
        return delta_s, delta_theta

    def update_state(self, state, delta_s, delta_theta, ):
        """From Lab 2"""
        # ****************** Additional Student Code: Start ************
        state.x += delta_s * math.cos(state.theta + 0.5*delta_theta)
        state.y += delta_s * math.sin(state.theta + 0.5*delta_theta)
        state.theta = self.norm_theta(state.theta + delta_theta)
        # ****************** Additional Student Code: End ************

        # keep this to return the updated state
        return state
        
    def norm_theta(self, theta):
        """Normalise angles"""
        if theta > math.pi:
            return theta - 2*math.pi
        elif theta < -1*math.pi:
            return theta + 2*math.pi
        else:
            return theta    
        
    def norm_dotprod(self, x1, y1, x2, y2):
        """Calculate the normalised dot product between"""
        mag1 = math.sqrt(x1**2 + y1**2)
        mag2 = math.sqrt(x2**2 + y2**2)

        if mag1 == 0 or mag2 == 0:
            return 0

        nx1 = x1/mag1
        ny1 = y1/mag1

        nx2 = x2/mag2
        ny2 = y2/mag2

        return nx1*nx2 + ny1*ny2

    def cap_wheels(self, wheel_angular_vels, cap_mps):
        """Cap the angular wheel speed of the robot."""
        # Calculate the Angular Cap
        a_cap = cap_mps * self.angular_to_wheel_conv
        max_ind, max_v = max(enumerate(wheel_angular_vels),
                             key=lambda x: abs(x[1]))
        if max_v == 0:
            return [0] * len(wheel_angular_vels)

        # Scale all values so that the maximum wheel angular velocity is a_cap
        max_capped = min(a_cap, max(-a_cap, max_v))
        scaling = max_capped / max_v

        new_vels = []
        for w in wheel_angular_vels:
            new_vels.append(w * scaling)
        return new_vels

    def assign_path(self, path_points):
        self.path = path_points
        self.is_following_path = True

    def cancel_path(self):
        self.path = None
        self.is_following_path = False

    def length_along_path(self):
        if self.path is None:
            return 0
        start_x = self.state_est.x
        start_y = self.state_est.y
        init_dist = math.sqrt((start_x - self.path[0][0])**2 +
                              (start_y - self.path[0][1])**2)
        running_dist = 0
        for i in range(1, len(self.path)):
            p1 = self.path[i-1]
            p2 = self.path[i]
            running_dist += math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        return running_dist + init_dist

    def rotation_track(self, other_id):
        other_robot = self.environment.robots[other_id]
        to_angle = math.atan2(other_robot.state_est.y - self.state_est.y,
                              other_robot.state_est.x - self.state_est.x)
        delta_theta = self.angle_wrap(to_angle - self.state_est.theta)
        des_w = delta_theta * self.rotate_gain * 0.6
        des_v = 0
        print("Delta {}, W: {}".format(delta_theta, des_w))

        # TODO: Flip des_w if we are on a real robot. THIS IS A HACK.
        if (self.environment.robot_mode == "HARDWARE MODE"
                and self.robot_id == 1):
            # Flip rotations only for this one robot
            des_w = -des_w
        return self.get_des_wheel(des_v, des_w)
