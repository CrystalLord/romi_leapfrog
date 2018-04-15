import math
from tkinter import *
from E160_robot import *
from PIL import Image, ImageTk

class E160_graphics:
    
    def __init__(self, environment):
        self.environment = environment
        self.tk = Tk()
        self.scale = 500
        self.canvas = Canvas(self.tk, width=self.environment.width*self.scale, height=self.scale* self.environment.height)
        self.tk.title("E160 - Autonomous Robot Navigation")
        self.canvas.bind("<Button-1>", self.callback)
        self.canvas.pack()
        self.gui_stopped = False
        
        # draw static environment
        for w in self.environment.walls:
            self.draw_wall(w)
            
        # draw first robot
        for r in self.environment.robots:
            self.initial_draw_robot(r)   


    def callback(self, event):
        #print "clicked at", event.x, event.y
        self.gui_stopped = True
        

    def draw_wall(self, wall):
        
        wall_points = self.scale_points(wall.points, self.scale)
        wall.poly = self.canvas.create_polygon(wall_points, fill='gray')
        
    def scale_points(self, points, scale):
        scaled_points = []
        for i in range(len(points)-1):
            
            if i % 2 == 0:
                # extract x,y
                x = points[i]
                y = points[i+1]

                # for x values, just multiply times scale factor to go from meters to pixels
                scaled_points.append(self.environment.width/2*scale + x*scale)   
                
                # only flip y for x,y points, not for circle radii
                scaled_points.append(self.environment.height/2*scale - y*scale)   
                    
        return scaled_points
    
    def initial_draw_robot(self, robot):
    
        # draw circular base 
        #base_points = self.draw_base(robot)
        #robot.draw_circle = self.canvas.create_oval(base_points,fill='black')
        
        # draw rectangular PCB
        #pcb_points = self.draw_board(robot)
        #robot.draw_pcb = self.canvas.create_polygon(pcb_points,fill='red')
        
        # draw wheel well
        #wheel_well_points = self.draw_wheel_well(robot)
        #robot.draw_wheel_well = self.canvas.create_polygon(wheel_well_points,fill='white')
        
        # draw wheel
        #wheel_points = self.draw_wheel(robot)
        #robot.draw_wheel = self.canvas.create_polygon(wheel_points,fill='black')
        
        # open image
        self.robot_image = Image.open("E160_robot_image.gif").convert('RGBA') 

        
        # gif draw
        #robot.tkimage = PhotoImage(file = "E160_robot_image.gif")
        rotated_image = self.robot_image.rotate(45)
        robot.tkimage = ImageTk.PhotoImage(self.robot_image.rotate(40))
        robot.image = self.canvas.create_image(robot.state.x, robot.state.y, image=robot.tkimage)

        
    
         
    
    def draw_robot(self, robot):
        
        # update the base position
        #base_points = self.draw_base(robot)
        #self.canvas.coords(robot.draw_circle, *base_points)
        
        # update the pcb position
        #pcb_points = self.draw_board(robot)
        #self.canvas.coords(robot.draw_pcb, *pcb_points)

        # update the wheel well position
        #wheel_well_points = self.draw_wheel_well(robot)
        #self.canvas.coords(robot.draw_wheel_well, *wheel_well_points)
    
        # update the wheel well position
        #wheel_points = self.draw_wheel(robot)
        #self.canvas.coords(robot.draw_wheel, *wheel_points)

        # gif update
        robot.tkimage = ImageTk.PhotoImage(self.robot_image.rotate(180/3.14*robot.state.theta))
        robot.image = self.canvas.create_image(robot.state.x, robot.state.y, image=robot.tkimage)
        robot_points = self.scale_points([robot.state.x, robot.state.y], self.scale)
        self.canvas.coords(robot.image, *robot_points)
            

        
    
    # called at every iteration of main loop
    def update(self):
        
        # draw robots
        for r in self.environment.robots:
            self.draw_robot(r)     
        
        # draw particles
        
        
        # draw sensors
        
 
        # update the graphics
        self.tk.update()

        
        # check for quit
        if self.gui_stopped:
            return False
        else:
            return True
        
        
        
    
    
    
   




    def rotate_points(self, points, theta):

        theta = -theta
        rotated_points = []
        for i in range(len(points)-1):
            
            if i % 2 == 0:
                # extract x,y
                x = points[i]
                y = points[i+1]

                # apply rotation matrix to x,y
                rotated_points.append(x*math.cos(theta)+y*math.sin(theta))
                rotated_points.append(-x*math.sin(theta)+y*math.cos(theta))

        return rotated_points


    def translate_points(self, points, dx, dy):

        translated_points = []
        for i in range(len(points)-1):
            
            if i % 2 == 0:
                # extract x,y
                x = points[i]
                y = points[i+1]

                # apply translation to x,y
                translated_points.append(x+dx)
                translated_points.append(y+dy)
            
        return translated_points

    

        
        
        
    def draw_base(self, robot):
        base_points = [robot.state.x-robot.radius, robot.state.y-robot.radius, robot.state.x+robot.radius, robot.state.y+robot.radius]
        base_points = self.scale_points(base_points, self.scale)
        
        return base_points
    
    def draw_board(self, robot):
        pcb_points = [-robot.radius/3,robot.radius/3,
                     -robot.radius*2/3,robot.radius/3,
                     -robot.radius*2/3,-robot.radius/3,
                     -robot.radius/3,-robot.radius/3]
        
        pcb_points = self.rotate_points(pcb_points, robot.state.theta)
        pcb_points = self.translate_points(pcb_points, robot.state.x, robot.state.y)
        pcb_points = self.scale_points(pcb_points, self.scale)
        
        return pcb_points
        
    def draw_wheel_well(self, robot):
        wheel_well_points = [robot.wheel_radius*1.1,robot.radius*1.1,
                     -robot.wheel_radius*1.1,robot.radius*1.1,
                     -robot.wheel_radius*1.1,robot.radius*0.8,
                     robot.wheel_radius*1.1,robot.radius*0.8]
        
        wheel_well_points = self.rotate_points(wheel_well_points, robot.state.theta)
        wheel_well_points = self.translate_points(wheel_well_points, robot.state.x, robot.state.y)
        wheel_well_points = self.scale_points(wheel_well_points, self.scale)
        
        return wheel_well_points
        
         
    def draw_wheel(self, robot):
        wheel_points = [robot.wheel_radius*1.0,robot.radius*1.0,
                     -robot.wheel_radius*1.0,robot.radius*1.0,
                     -robot.wheel_radius*1.0,robot.radius*0.9,
                     robot.wheel_radius*1.0,robot.radius*0.9]
        
        wheel_points = self.rotate_points(wheel_points, robot.state.theta)
        wheel_points = self.translate_points(wheel_points, robot.state.x, robot.state.y)
        wheel_points = self.scale_points(wheel_points, self.scale)
        
        return wheel_points
        

