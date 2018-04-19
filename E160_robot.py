
from E160_state import *
from E160_PF import E160_PF
import E160_rangeconv
import math
import datetime
import time

class E160_robot:

    def __init__(self, environment, address, robot_id):
        self.debug = False

        self.environment = environment
        self.state_est = E160_state()
        self.state_est.set_state(0,0,0)
        self.state_des = E160_state()
        self.state_des.set_state(0, 0, 0)

        # Where are we drawing the robot on the GUI?
        self.state_draw = E160_state()
        self.state_odo = E160_state()
        self.state_odo.set_state(0, 0, 0) # real position for simulation
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
        self.manual_control_left_motor = 0
        self.manual_control_right_motor = 0
        self.file_name = 'Log/Bot' + str(self.robot_id) + '_'\
            + datetime.datetime.now()\
            .replace(microsecond=0)\
            .strftime('%y-%m-%d %H.%M.%S') + '.txt'
        self.make_headers()

        self.encoder_resolution = 1440

        self.last_encoder_measurements = [0, 0]
        self.encoder_measurements = [0, 0]
        self.range_measurements = [0, 0, 0]
        # Orientations of the sensors on the robot
        self.sensor_orientation = [-math.pi/4, 0, math.pi/4]
        self.last_simulated_encoder_R = 0
        self.last_simulated_encoder_L = 0
        
        self.Kpho = 1#1.0
        self.Kalpha = 2#2.0
        self.Kbeta = -0.5#-0.5
        self.max_velocity = 1#0.05
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
        self.points = [[0.2,-0.1,-1.57]] 
                      # [0.0, 0.0,0.0], 
                      # [0.20,0.0,1.57], 
                      # [0.20, 0.20, 1.57],
                      # [0.0, 0.0, 0.0]]
        self.is_hardcoded = False

    def update(self, deltaT):
        # get sensor measurements
        self.encoder_measurements, self.range_measurements =\
            self.update_sensor_measurements(deltaT)

        # update odometry
        delta_s, delta_theta = self.update_odometry(self.encoder_measurements)

        # update simulated real position, find ground truth for simulation
        self.state_odo = self.localize(self.state_odo, delta_s, delta_theta,
                                       self.range_measurements)

        # localize with particle filter
        if self.environment.robot_mode == "HARDWARE MODE":
            self.state_est = self.environment.pf.LocalizeEstWithParticleFilter(
                self.encoder_measurements,
                [self.range_measurements[0]],
                self
            )
        else:
            self.state_est = self.environment.pf.LocalizeEstWithParticleFilter(
                self.encoder_measurements,
                self.range_measurements,
                self
            )

        self.last_encoder_measurements = self.encoder_measurements
        # to out put the true location for display purposes only.
        self.state_draw = self.state_odo

        # call motion planner
        #self.motion_planner.update_plan()
        
        # determine new control signals
        self.R, self.L = self.update_control()
        
        # send the control measurements to the robot
        self.send_control(self.R, self.L, deltaT)

    def update_sensor_measurements(self, deltaT):
        
        if self.environment.robot_mode == "HARDWARE MODE":
            command = '$S @'
            self.environment.xbee.tx(dest_addr = self.address, data = command)
            
            update = self.environment.xbee.wait_read_frame()
            
            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            # We flip these because the robot is driving backwards technically.
            encoder_measurements = list(reversed(data[-2:]))
            range_measurements = data[:-2]
            
        elif self.environment.robot_mode == "SIMULATION MODE":
            encoder_measurements = self.simulate_encoders(self.R, self.L,
                                                          deltaT)
            # New in lab 4
            range_measurements = []
            for o in self.sensor_orientation:
                new_reading = self.simulate_range_finder(self.state_odo, o)
                range_measurements.append(new_reading)
            range_measurements = list(map(E160_rangeconv.m2range,
                                          range_measurements))
        return encoder_measurements, range_measurements

    def localize(self, state_est, delta_s, delta_theta, range_measurements):
        # New lab 4 state estimate function. We must be given delta_s and
        # delta_theta now.
        state_est = self.update_state(state_est, delta_s, delta_theta)
        #delta_s, delta_theta = self.update_odometry(encoder_measurements)
        #state_est = self.update_state(state_est, delta_s, delta_theta)

        return state_est
    
    def angle_wrap(self, a):
        while a > math.pi:
            a = a - 2*math.pi
        while a < -math.pi:
            a = a + 2*math.pi
            
        return a
        
    def update_control(self):
        
        if self.environment.control_mode == "MANUAL CONTROL MODE":
            desiredWheelSpeedR = self.manual_control_right_motor
            desiredWheelSpeedL = self.manual_control_left_motor
            
        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":   
            if self.is_hardcoded:
                desiredWheelSpeedR, desiredWheelSpeedL = self.path_tracker_control()
            else: 
                desiredWheelSpeedR, desiredWheelSpeedL = self.point_tracker_control()
            
        return desiredWheelSpeedR, desiredWheelSpeedL
  
    def path_tracker_control(self):
        """ Takes in a list of points, where each point consists of x, y 
            and theta. Iteratively calls point_tracker_control() to take
            robot on path."""

        if len(self.points) == 0:
            return 0,0
        else:
            self.state_des.x = self.points[0][0]
            self.state_des.y = self.points[0][1]
            self.state_des.theta = self.points[0][2]
            self.point_tracked = False
            print("go to first point: ", self.state_des.x)

            desiredWheelSpeedR, desiredWheelSpeedL = self.point_tracker_control()
            if self.debug:
                print("Desired Wheel Speeds:", desiredWheelSpeedR,
                    desiredWheelSpeedL)
            if desiredWheelSpeedR == 0 and desiredWheelSpeedL == 0:
                self.points = self.points[1:]

        return desiredWheelSpeedR, desiredWheelSpeedL
    
    def point_tracker_control(self):
        """Set desired wheel positions from a given desired point"""
        # Local gains, set to environment variables later
        distGain = 1.5
        rotateGain = 12
        alphaGain = 10
        betaGain = -0.5

        velNom = 0.4

        # What are the thresholds for reaching the target? When are we close
        # enough?
        distThresh = 0.02
        thetaThresh = 0.06

        # Shorthands for our current state
        curX = self.state_est.x
        curY = self.state_est.y
        curTheta = self.state_est.theta
        delx, dely, deltheta = (self.state_des.x - curX,
                                self.state_des.y - curY,
                                self.norm_theta(self.state_des.theta
                                                - curTheta))

        # Get distance to target.
        distTarget = math.sqrt(delx**2 + dely**2)
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

        print("rho      :", distTarget)
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
                desV = (distGain*distTarget + velNom)
            else:
                # Reverse the angle if the target is behind us.
                alpha = self.norm_theta(
                    -curTheta + self.norm_theta(math.atan2(dely * dot,
                                                           delx * dot)))
                # Reverse the drive direction.
                desV = (distGain*distTarget + velNom) * -1

            # Get reverse change of angle.
            beta = self.norm_theta(self.norm_theta(-curTheta - alpha)+self.state_des.theta)
            # Compute the desired angular velocity.
            desW = alphaGain*alpha + betaGain*beta

            print("alpha    :", math.degrees(alpha))
            print("beta     :", math.degrees(beta))

            if onlyRotate:
                print("ONLY ROTATING------")
                desW = deltheta * rotateGain
                desV = 0

            # Add nominal angular velocity
            #desW = desW + math.copysign(desW, 1) * rotNom

            print("desW     :", desW)
            print("desV     :", desV)

            desiredWheelSpeedR = 2*(desV + desW*self.width) \
                    /self.wheel_radius * -1
            desiredWheelSpeedL = 2*(-desV + desW*self.width) \
                    /self.wheel_radius

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
        self.last_simulated_encoder_R
        left_encoder_measurement = -int(
            L*self.encoder_per_sec_to_rad_per_sec*deltaT) + last_l
        self.last_simulated_encoder_R = right_encoder_measurement
        self.last_simulated_encoder_L = left_encoder_measurement

        return [left_encoder_measurement, right_encoder_measurement]

    def simulate_range_finder(self, state, sensorT):
        """Simulate range readings, given a simulated ground truth state"""
        p = self.environment.pf.Particle(((state.x, state.y, state.theta),
                                          (0, 0, 0)), 0)
        #p = self.PF.Particle(state.x, state.y, state.theta, 0)
        return self.environment.pf.FindMinWallDistance(
            p,
            self.environment.walls,
            sensorT
        )

    def make_headers(self):
        f = open(self.file_name, 'a+')
        f.write('{0} {1:^1} {2:^1} {3:^1} {4:^1} \n'.format('R1', 'R2', 'R3', 'RW', 'LW'))
        # Old Lab 3 log headers.
        #f.write('{6} {0} {1:^1} {2:^1} {3:^1} {4:^1} {5:^1} \n'.format('X', 'Y', 'Theta', 'Des_X', 'Des_Y','Des_Theta', 'time'))
        f.close()


        
    def log_data(self):
        f = open(self.file_name, 'a+')
        
        # edit this line to have data logging of the data you care about
        #data = [str(x) for x in [1,2,3,4,5]]
        # Old lab 3 log data.
        data = [str(x) for x in [time.time(), self.state_est.x,
                                 self.state_est.y, self.state_est.theta,
                                 self.state_odo.x, self.state_odo.y,
                                 self.state_odo.theta,
                                 self.environment.pf.particle_weight_sum,
                                 self.range_measurements[0],
                                 self.environment.pf.numParticles]]
        
        f.write(' '.join(data) + '\n')
        f.close()
        
        
    def set_manual_control_motors(self, R, L):
        
        self.manual_control_right_motor = int(R*256/100)
        self.manual_control_left_motor = int(L*256/100)                                                         
   


    def update_odometry(self, encoder_measurements):
        """From Lab 2, update the possible state from encoder measurements."""
        delta_s = 0
        delta_theta = 0

        # ****************** Additional Student Code: Start ************

        # Calculate encoder differences
        encoderL, encoderR = encoder_measurements
        lastencoderL, lastencoderR = self.last_encoder_measurements

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

    def update_state(self, state, delta_s, delta_theta):
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
