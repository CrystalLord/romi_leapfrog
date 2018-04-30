import math


class Ray(object):
    def __init__(self, origin, direction):
        self.origin = origin
        self.dir = self._normalise(direction)

    def _normalise(self, dir):
        mag = 0
        for i in dir:
            mag += i**2
        mag = math.sqrt(mag)
        if mag != 0:
            new_dir = [i/mag for i in dir]
        else:
            new_dir = [0 for i in dir]
        return new_dir

    def along(self, distance):
        new_point = []
        for i in range(len(self.origin)):
            new_point.append(self.origin[i] + self.dir[i]*distance)
        return new_point

    def rotate(self, angle, ccw):
        """
        In place rotation of the ray.
        Args:
            angle: Angle (in radians) to rotate the ray.
        Returns:
        """
        current_angle = math.atan2(self.dir[1], self.dir[0])
        new_angle = current_angle + angle
        self.dir = [math.cos(new_angle), math.sin(new_angle)]


def get_leap_path(robots, robot_to_move_id, des_point, fidelity=4):
    """
    Args:
        robots:
        robot_to_move_id:
        distance:
        fidelity:
    Returns:
        Returns a list of waypoints
    """
    robot = robots[robot_to_move_id]
    #other_robot = robots[(robot_to_move_id+1) % 2]
    direction = [des_point[0] - robot.state_est.x,
                 des_point[1] - robot.state_est.y]
    ray = Ray([robot.state_est.x, robot.state_est.y], direction)
    distance = math.sqrt((robot.state_est.x-des_point[0])**2 +
                         (robot.state_est.y-des_point[1])**2)
    centre = ray.along(distance*0.5)
    arc_ray = Ray(centre, [-d for d in direction])
    track_points = []
    for i in range(fidelity):
        arc_ray.rotate(math.pi/fidelity, True)
        xy = arc_ray.along(distance*0.5)
        # Add direction to each point.
        if i == 0:
            point = xy + [math.atan2(xy[1]-robot.state_est.y,
                                     xy[0]-robot.state_est.x)]
        elif i+1 != fidelity:
            point = xy + [math.atan2(xy[1]-track_points[i-1][1],
                                     xy[0]-track_points[i-1][0])]
        else:
            point = xy + [math.atan2(direction[1], direction[0])]
        track_points.append(point)
    return track_points
