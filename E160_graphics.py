import math
from tkinter import *
from E160_robot import *
from PIL import Image, ImageTk
import time
from pynput.keyboard import Key, Listener

import E160_rangeconv
import E160_leap

class E160_graphics:
    
    def __init__(self, environment):
        self.environment = environment
        self.tk = Tk()
        #self.north_east_frame = Frame(self.tk)
        #self.north_east_frame.pack(anchor = NE)
        self.north_west_frame = Frame(self.tk)
        self.north_west_frame.pack(anchor=W)
        #self.north_frame = Frame(self.tk)
        #self.north_frame.pack(anchor = N)
        
        self.bottom_frame = Frame(self.tk)
        self.bottom_frame.pack(side = BOTTOM)
        
        self.scale = 100
        self.canvas = Canvas(self.tk, width=self.environment.width*self.scale,
                             height=self.scale*self.environment.height)
        self.tk.title("E160 - Autonomous Robot Navigation")
        self.canvas.bind("<Button-1>", self.callback)
        self.canvas.pack()
        self.gui_stopped = False
        self.last_rotate_control = 0
        self.last_forward_control = 0
        self.R = 0
        self.L = 0

        self.forward_c = 0
        self.turn_c = 0

        
        # add motor control slider
        self.forward_control = Scale(self.bottom_frame, from_=-40, to=40,
                                     length  = 400,label="Forward Control",
                                     tickinterval=5, orient=HORIZONTAL)
        self.forward_control.pack(side=LEFT)
        
        # add rotation control slider
        self.rotate_control = Scale(self.bottom_frame, from_=-30, to=30,
                                    length = 400,label="Rotate Control",
                                    tickinterval=5, orient=HORIZONTAL)
        self.rotate_control.pack(side=RIGHT)
        
        # add track point button
        self.track_point_button = Button(self.bottom_frame, text="Track Point", anchor="s", wraplength=100, command=self.track_point)
        self.track_point_button.pack()
        
        # add stop button
        self.stop_button = Button(self.bottom_frame, text="Stop", anchor="s",
                               wraplength=100, command=self.stop).pack()
  
        # add stop button
        self.quit_button = Button(self.bottom_frame, text="Quit", anchor="s",
                               wraplength=100, command=self.quit).pack()
 
        # add reset button
        self.reset_button = Button(self.bottom_frame, text = "Reset", anchor =
        "s", wraplength = 100, command=self.reset).pack()
 
        # add range sensor measurements
        self.range_sensor_var_1 = StringVar()
        self.range_sensor_var_2 = StringVar()
        self.range_sensor_var_3 = StringVar()
        self.range_sensor_label_1 = Label(self.north_west_frame, textvariable = self.range_sensor_var_1).pack()
        self.range_sensor_label_2 = Label(self.north_west_frame, textvariable = self.range_sensor_var_2).pack()
        self.range_sensor_label_3 = Label(self.north_west_frame, textvariable = self.range_sensor_var_3).pack()

        # add encoder sensor measurements
        self.encoder_sensor_var_0 = StringVar()
        self.encoder_sensor_var_1 = StringVar()
        
        self.encoder_sensor_label_0 = Label(self.north_west_frame, textvariable = self.encoder_sensor_var_0).pack()
        self.encoder_sensor_label_1 = Label(self.north_west_frame, textvariable = self.encoder_sensor_var_1).pack()

        # add range sensor measurements
        self.x = StringVar()
        self.y = StringVar()
        self.theta = StringVar()
        self.x_label = Label(self.north_west_frame, textvariable = self.x).pack()
        self.y_label = Label(self.north_west_frame, textvariable = self.y).pack()
        self.theta_label = Label(self.north_west_frame, textvariable = self.theta).pack()

        # add text entry for desired X
        #self.x_des_label = Label(self.north_frame, text="X desired")
        #self.x_des_label.pack()
        self.x_des_entry = Entry(self.north_west_frame, justify = RIGHT)
        self.x_des_entry.insert(10, "0.0")
        self.x_des_entry.pack()
        
        # add text entry for desired Y
        #self.y_des_label = Label(self.north_west_frame, text="Y desired")
        #self.y_des_label.pack()
        self.y_des_entry = Entry(self.north_west_frame, justify = RIGHT)
        self.y_des_entry.insert(10, "0.0")
        self.y_des_entry.pack()
        
        # add text entry for desired Theta
        #self.theta_des_label = Label(self.north_west_frame, text="Theta desired")
        #self.theta_des_label.pack()
        self.theta_des_entry = Entry(self.north_west_frame, justify = RIGHT)
        self.theta_des_entry.insert(10, "0.0")
        self.theta_des_entry.pack()

        # Setup leap id field
        self.leap_id_entry = Entry(self.north_west_frame, justify=RIGHT)
        self.leap_id_entry.insert(10, "0")
        self.leap_id_entry.pack()


        # initilize particle representation
        self.particles_dot = [self.canvas.create_oval(0,0,0,0, fill ='black')
                              for x in
                              range(self.environment.pf.numParticles
                                    * len(self.environment.robots))
        ]
        self.angles_dots = [self.canvas.create_line(0, 0, 0, 0, fill='green')
                            for _ in range(self.environment.pf.numParticles)]

        self.state_est_particles = [self.canvas.create_oval(0, 0, 0, 0,
                                                           fill='black')
                                   for _ in self.environment.robots]
        self.sensor_rays = [[self.canvas.create_line(0, 0, 0, 0, fill='black')
                            for q in self.environment.range_meas[0]]
                            for r in self.environment.robots]
        self.path_points = []

        # =====================================================================
        # Graphical Settings

        # Colours
        self.headingcolor = "green"
        # Lengths
        self.headinglen = 0.1
        self.sensor_color = "purple"

        # ====================================================================
        # Draw the stuffs initially!

        # draw static environment
        for w in self.environment.walls:
            self.draw_wall(w)
            
        # draw first robot
        for r in self.environment.robots:
            self.initial_draw_robot(r)    
    
        self.count = 0
        self.current_leap = 1

        self.pressed_track = False
        self.last_track_time = 0

    def draw_wall(self, wall):

        wall_points = self.scale_points(wall.points, self.scale)
        wall.poly = self.canvas.create_polygon(wall_points, fill='black')

    def scale_points(self, points, scale):
        scaled_points = []
        for i in range(len(points)-1):

            if i % 2 == 0:
                # for x values, just multiply times scale factor to go from meters to pixels
                scaled_points.append(self.environment.width/2*scale + points[i]*scale)

                # only flip y for x,y points, not for circle radii
                scaled_points.append(self.environment.height/2*scale - points[i+1]*scale)

        return scaled_points


    def reverse_scale_points(self, points, scale):
        reverse_scaled_points = []
        for i in range(len(points)-1):

            if i % 2 == 0:
                # for x values, just multiply times scale factor to go from meters to pixels
                reverse_scaled_points.append(-self.environment.width/2 + points[i]/scale)

                # only flip y for x,y points, not for circle radii
                reverse_scaled_points.append(self.environment.height/2 - points[i+1]/scale)

        return reverse_scaled_points


    def initial_draw_robot(self, robot):
        # open image
        robot.robot_gif = Image.open("E160_robot_image.gif").convert('RGBA')
        robot.robot_gif.thumbnail((20, 20), Image.ANTIALIAS)

    def draw_robot(self, robot):
        # gif update
        robot.tkimage = ImageTk.PhotoImage(robot.robot_gif.rotate(180/3.14*robot.state_draw.theta))
        robot.image = self.canvas.create_image(robot.state_draw.x, robot.state_draw.y, image=robot.tkimage)
        robot_points = self.scale_points([robot.state_draw.x, robot.state_draw.y], self.scale)
        self.canvas.coords(robot.image, *robot_points)

    def draw_path(self, path_points):
        for dp in self.path_points:
            self.canvas.delete(dp)

        for p in path_points:
            scaled_p = self.scale_points(p, self.scale)
            rect = (scaled_p[0]-2, scaled_p[1]-2, scaled_p[0]+2, scaled_p[1]+2)
            new_point = self.canvas.create_rectangle(rect[0], rect[1],
                                                     rect[2], rect[3],
                                                     fill="red")
            self.path_points.append(new_point)

    def draw_particles(self, robot_id, color="red"):
        numparticles = self.environment.pf.numParticles
        for i in range(numparticles):
            # TODO: Change this from only showing the first robot
            pf_point = [self.environment.pf.particles[i].get_x(robot_id),
                        self.environment.pf.particles[i].get_y(robot_id)]
            pf_angle = self.environment.pf.particles[i].get_theta(robot_id)
            #h = str(hex(int((min(max(robot.PF.particles[i].weight
            #                         *robot.PF.numParticles, 0.0),
            #                          255.0)))))[2:]
            h = "ff"
            if len(h) < 2:
                h += "0"
            point = self.scale_points(pf_point, self.scale)
            self.canvas.delete(self.particles_dot[i + robot_id * numparticles])
            self.particles_dot[i + robot_id * numparticles] =\
                self.canvas.create_oval(point[0] - 2,
                                        point[1] - 2,
                                        point[0] + 2,
                                        point[1] + 2,
                                        fill=color)

            self.canvas.delete(self.angles_dots[i])
            # We need to flip the sign on these since in the GUI, the top
            # left corner is the origin, and down is a positive y.
            lineto = (self.headinglen*self.scale*math.cos(-pf_angle)
                      + point[0],
                      self.headinglen*self.scale*math.sin(-pf_angle) +
                      point[1])
            self.angles_dots[i] = self.canvas.create_line(
                point[0],
                point[1],
                lineto[0],
                lineto[1],
                fill=self.headingcolor)

    def draw_est(self, robot_id, colour="blue"):
        robot = self.environment.robots[robot_id]
        pf_point = [robot.state_est.x, robot.state_est.y]
        point = self.scale_points(pf_point, self.scale)
        self.canvas.delete(self.state_est_particles[robot_id])
        self.state_est_particles[robot_id] = \
            self.canvas.create_oval(point[0] - 4,
                                    point[1] - 4,
                                    point[0] + 4,
                                    point[1] + 4,
                                    fill=colour)

    def draw_sensors(self, robot_id):
        robot = self.environment.robots[robot_id]
        est = robot.state_est
        s = self.environment.range_meas[robot_id]
        for i, o in enumerate(robot.sensor_orientation):
            ran = E160_rangeconv.range2m(s[i], robot_id)
            self.canvas.delete(self.sensor_rays[robot_id][i])
            inter_x = math.cos(o+est.theta)*ran + est.x
            inter_y = math.sin(o+est.theta)*ran + est.y
            points = self.scale_points([est.x, est.y, inter_x, inter_y],
                                       self.scale)
            self.sensor_rays[robot_id][i] = (self.canvas.create_line(
                points[0],
                points[1],
                points[2],
                points[3],
                fill=self.sensor_color))


    def track_point(self):
        #self.environment.control_mode = "AUTONOMOUS CONTROL MODE"
        self.environment.control_mode = "LEAP PATH CONTROL MODE"

        # update sliders on gui
        self.forward_control.set(0)
        self.rotate_control.set(0)
        self.last_forward_control = 0
        self.last_rotate_control = 0
        self.R = 0
        self.L = 0

        leaping_robot_id = self.current_leap # int(self.leap_id_entry.get())

        # draw robots
        for i, r in enumerate(self.environment.robots):
            x_des = float(self.x_des_entry.get())
            y_des = float(self.y_des_entry.get())
            theta_des = float(self.theta_des_entry.get())
            # TODO: Temporary hack to show leapfrogging.
            if i == leaping_robot_id:
                path = E160_leap.get_leap_path(self.environment.robots, i,
                                               ((self.count+1)*0.5 + 0.5, 0),
                                               fidelity=3)
                r.assign_path(path)
                r.point_tracked = False
                r.is_rotation_tracking = False
            else:
                r.cancel_path()
                r.is_rotation_tracking = True
            #r.state_des.set_state(x_des,y_des,theta_des)
            self.pressed_track = True
            self.last_track_time = time.time()

    def stop(self):
        self.environment.control_mode = "MANUAL CONTROL MODE"
        
        # update sliders on gui
        self.forward_control.set(0)
        self.rotate_control.set(0)       
        self.last_forward_control = 0
        self.last_rotate_control = 0  
        self.R = 0
        self.L = 0
        
    def quit(self):
        self.environment.control_mode = "MANUAL CONTROL MODE"
        self.forward_control.set(0)
        self.rotate_control.set(0)  
        self.gui_stopped = True
    
    def reset(self):
        self.environment.reset()

    def callback(self, event):
        desired_points = self.reverse_scale_points([float(event.x), float(event.y)], self.scale)
        robot = self.environment.robots[0]
        robot.state_des.set_state(desired_points[0],desired_points[1],0)
        print("New desired robot state", robot.state_des.x, robot.state_des.y)
        
    def send_robot_commands(self):
        # check to see if forward slider has changed
        if abs(self.forward_control.get()-self.last_forward_control) > 0:
            self.rotate_control.set(0)       
            self.last_forward_control = self.forward_control.get()
            self.last_rotate_control = 0
            self.environment.control_mode = "MANUAL CONTROL MODE"

            # extract what the R and L motor signals should be
            self.R = -self.forward_control.get()
            self.L = -self.forward_control.get()
  
        # check to see if rotate slider has changed
        elif abs(self.rotate_control.get()-self.last_rotate_control) > 0:
            self.forward_control.set(0)       
            self.last_rotate_control = self.rotate_control.get()
            self.last_forward_control = 0         
            self.environment.control_mode = "MANUAL CONTROL MODE"

            # extract what the R and L motor signals should be
            self.R = self.rotate_control.get()
            self.L = -self.rotate_control.get()
        
        # if manual mode, set motors
        if self.environment.control_mode == "MANUAL CONTROL MODE":
            for r in self.environment.robots:
                r.cancel_path()
            # tell robot what the values should be
            robot = self.environment.robots[0]
            robot.set_manual_control_motors(self.R, self.L)

    def update_labels(self):
        self.range_sensor_var_1.set("Range 1 (m):  " + str(
            self.environment.range_meas[0][0]))
        try:
            self.range_sensor_var_2.set(
                "Range 2 (m):  " + str(self.environment.range_meas[0][1]))
        except IndexError:
            self.range_sensor_var_2.set("Range 2 (m):  NaN")

        try:
            self.range_sensor_var_3.set("Range 3 (m):  " + str(
                self.environment.range_meas[0][2]))
        except IndexError:
            self.range_sensor_var_3.set("Range 3 (m):  NaN")
            pass
                
        self.encoder_sensor_var_0.set("Encoder 0 (m):  " + str(
            self.environment.encoder_meas[0][0]))
        self.encoder_sensor_var_1.set("Encoder 1 (m):  " + str(
            self.environment.encoder_meas[0][1]))

        self.x.set("X_est (m):  " + str(self.environment.robots[0].state_est.x))
        self.y.set("Y_est (m):  " + str(self.environment.robots[0].state_est.y))
        self.theta.set("Theta_est (rad):  " + str(self.environment.robots[0].state_est.theta))
        
        
        
    # called at every iteration of main loop
    def update(self):
        #print(self.pressed_track, self.last_track_time)
        if self.pressed_track and time.time() - self.last_track_time > 0.25:
            self.count += 1
            self.current_leap = (self.current_leap + 1) % 2
            print("Next Leap: {}".format(self.current_leap))
            print("Next point: {}".format(self.count + 1))
            self.pressed_track = False
        # update gui labels
        self.update_labels()
        
        # draw robots
        for r in self.environment.robots:
            self.draw_robot(r)     
        
        # draw particles
        self.draw_particles(0, "red")

        self.draw_est(0, "blue")

        # draw sensors
        self.draw_sensors(0)
        if self.environment.num_robots > 1:
            self.draw_particles(1, "orange")
            self.draw_est(1, "purple")
            self.draw_sensors(1)

        if self.environment.robots[0].path is not None:
            self.draw_path(self.environment.robots[0].path)

        if self.environment.robots[1].path is not None:
            self.draw_path(self.environment.robots[1].path)

        # update the graphics
        self.tk.update()

        # send commands tp robots
        self.send_robot_commands()

        # check for quit
        if self.gui_stopped:
            self.environment.quit()
            return False
        else:
            return True

    #def turn(self, v):
    #    self.turn_c = v

    #def forward(self, v):
    #    self.forward_c = v
