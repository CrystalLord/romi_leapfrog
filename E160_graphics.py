import math
from tkinter import *
from E160_robot import *
from PIL import Image, ImageTk
from pynput.keyboard import Key, Listener

import E160_rangeconv

class E160_graphics:
    
    def __init__(self, environment):
        self.environment = environment
        self.tk = Tk()
        #self.north_east_frame = Frame(self.tk)
        #self.north_east_frame.pack(anchor = NE)
        self.north_west_frame = Frame(self.tk)
        print("Ran here!")
        self.north_west_frame.pack(anchor=W)
        #self.north_frame = Frame(self.tk)
        #self.north_frame.pack(anchor = N)
        
        self.bottom_frame = Frame(self.tk)
        self.bottom_frame.pack(side = BOTTOM)
        
        self.scale = 200
        self.canvas = Canvas(self.tk, width=self.environment.width*self.scale, height=self.scale* self.environment.height)
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
        self.track_point_button = Button(self.bottom_frame, text="Track Point", anchor="s", wraplength=100, command=self.track_point).pack()
        
        # add stop button
        self.track_point_button = Button(self.bottom_frame, text="Stop", anchor="s", wraplength=100, command=self.stop).pack()
  
        # add stop button
        self.track_point_button = Button(self.bottom_frame, text="Quit", anchor="s", wraplength=100, command=self.quit).pack()
 
        # add reset button
        self.track_point_button = Button(self.bottom_frame, text = "Reset", anchor = "s", wraplength = 100, command=self.reset).pack()
 
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
        self.x_des_entry.insert(10,"0.0")
        self.x_des_entry.pack()
        
        # add text entry for desired Y
        #self.y_des_label = Label(self.north_west_frame, text="Y desired")
        #self.y_des_label.pack()
        self.y_des_entry = Entry(self.north_west_frame, justify = RIGHT)
        self.y_des_entry.insert(10,"0.0")
        self.y_des_entry.pack()
        
        # add text entry for desired Theta
        #self.theta_des_label = Label(self.north_west_frame, text="Theta desired")
        #self.theta_des_label.pack()
        self.theta_des_entry = Entry(self.north_west_frame, justify = RIGHT)
        self.theta_des_entry.insert(10,"0.0")
        self.theta_des_entry.pack()


        # initilize particle representation
        self.particles_dot = [self.canvas.create_oval(0,0,0,0, fill ='black')
                              for x in range(self.environment.pf.numParticles)]
        self.angles_dots = [self.canvas.create_line(0, 0, 0, 0, fill='green')
                            for _ in range(self.environment.pf.numParticles)]
        self.state_est_particle = self.canvas.create_oval(0, 0, 0, 0,
                                                          fill='black')
        self.sensor_rays = [self.canvas.create_line(0, 0, 0, 0, fill='black')
                            for _ in self.environment.robots[0]
                                .range_measurements]
        # =====================================================================
        # Graphical Settings

        # Colours
        self.headingcolor = "green"
        # Lengths
        self.headinglen = 0.1
        self.sensor_color = "purple"

        # draw static environment
        for w in self.environment.walls:
            self.draw_wall(w)
            
        # draw first robot
        for r in self.environment.robots:
            self.initial_draw_robot(r)    
    
    

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
        robot.robot_gif.thumbnail((50, 50), Image.ANTIALIAS)

    def draw_robot(self, robot):
        
        # gif update
        robot.tkimage = ImageTk.PhotoImage(robot.robot_gif.rotate(180/3.14*robot.state_draw.theta))
        robot.image = self.canvas.create_image(robot.state_draw.x, robot.state_draw.y, image=robot.tkimage)
        robot_points = self.scale_points([robot.state_draw.x, robot.state_draw.y], self.scale)
        self.canvas.coords(robot.image, *robot_points)
            
    def get_inputs(self):
        pass

    def draw_particles(self):
        for i in range(self.environment.pf.numParticles):
            # TODO: Change this from only showing the first robot
            pf_point = [self.environment.pf.particles[i].get_x(0),
                        self.environment.pf.particles[i].get_y(0)]
            pf_angle = self.environment.pf.particles[i].get_theta(0)
            #h = str(hex(int((min(max(robot.PF.particles[i].weight
            #                         *robot.PF.numParticles, 0.0),
            #                          255.0)))))[2:]
            h = "ff"
            if len(h) < 2:
                h += "0"
            color = ('#' + h + "0000")
            point = self.scale_points(pf_point, self.scale)
            self.canvas.delete(self.particles_dot[i]) 
            self.particles_dot[i] = self.canvas.create_oval(point[0] - 2,
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

    def draw_est(self, robot):
        pf_point = [robot.state_est.x, robot.state_est.y]
        point = self.scale_points(pf_point, self.scale)
        self.canvas.delete(self.state_est_particle)
        self.state_est_particle = self.canvas.create_oval(point[0] - 4,
                                                          point[1] - 4,
                                                          point[0] + 4,
                                                          point[1] + 4,
                                                          fill='blue')

    def draw_sensors(self, robot):
        est = robot.state_est
        s = robot.range_measurements
        for i, o in enumerate(robot.sensor_orientation):
            ran = E160_rangeconv.range2m(s[i])  # Get real world ranges
            self.canvas.delete(self.sensor_rays[i])
            inter_x = math.cos(o+est.theta)*ran + est.x
            inter_y = math.sin(o+est.theta)*ran + est.y
            points = self.scale_points([est.x, est.y, inter_x, inter_y],
                                       self.scale)
            self.sensor_rays[i] = (self.canvas.create_line(
                points[0],
                points[1],
                points[2],
                points[3],
                fill=self.sensor_color))


    def track_point(self):
        self.environment.control_mode = "AUTONOMOUS CONTROL MODE"
                
        # update sliders on gui
        self.forward_control.set(0)
        self.rotate_control.set(0)
        self.last_forward_control = 0
        self.last_rotate_control = 0
        self.R = 0
        self.L = 0
        
        # draw robots
        for r in self.environment.robots:
            x_des = float(self.x_des_entry.get())
            y_des = float(self.y_des_entry.get())
            theta_des = float(self.theta_des_entry.get())
            r.state_des.set_state(x_des,y_des,theta_des)
            r.point_tracked = False 
        
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
        for r in self.environment.robots:
            r.state_odo.set_state(0,0,0) 
        self.environment.pf.InitializeParticles()
 
    def callback(self, event):
        desired_points = self.reverse_scale_points([float(event.x), float(event.y)], self.scale)
        robot = self.environment.robots[0]
        robot.state_des.set_state(desired_points[0],desired_points[1],0)
        print("New desired robot state", robot.state_des.x, robot.state_des.y)
        
    def send_robot_commands(self):

        #if self.forward_c != 0:
        #    self.L = self.forward_c
        #    self.R = self.forward_c
        #    return
        #elif self.turn_c != 0:
        #    self.L = -self.turn_c
        #    self.R = self.turn_c
        #    return

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
            # tell robot what the values should be
            robot = self.environment.robots[0]
            robot.set_manual_control_motors(self.R, self.L)
        
        
    def update_labels(self):
        self.range_sensor_var_1.set("Range 1 (m):  " + str(self.environment.robots[0].range_measurements[0]))
        try:
            self.range_sensor_var_2.set(
                "Range 2 (m):  " + str(self.environment.robots[
                                       0].range_measurements[1]))
        except IndexError:
            self.range_sensor_var_2.set("Range 2 (m):  NaN")

        try:
            self.range_sensor_var_3.set("Range 3 (m):  " + str(self.environment.robots[0].range_measurements[2]))
        except IndexError:
            self.range_sensor_var_3.set("Range 3 (m):  NaN")
            pass
                
        self.encoder_sensor_var_0.set("Encoder 0 (m):  " + str(self.environment.robots[0].encoder_measurements[0]))
        self.encoder_sensor_var_1.set("Encoder 1 (m):  " + str(self.environment.robots[0].encoder_measurements[1]))

        self.x.set("X_est (m):  " + str(self.environment.robots[0].state_est.x))
        self.y.set("Y_est (m):  " + str(self.environment.robots[0].state_est.y))
        self.theta.set("Theta_est (rad):  " + str(self.environment.robots[0].state_est.theta))
        
        
        
    # called at every iteration of main loop
    def update(self):
        
        # update gui labels
        self.update_labels()
        
        # draw robots
        for r in self.environment.robots:
            self.draw_robot(r)     
        
        # draw particles
        self.draw_particles()

        self.draw_est(self.environment.robots[0])
        # draw sensors
        self.draw_sensors(self.environment.robots[0])

        for i in self.environment.get_walls(0):
            self.draw_wall(i)

        # update the graphics
        self.tk.update()

        # check for gui buttons
        self.get_inputs()

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
