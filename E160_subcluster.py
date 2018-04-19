import math

DOWNSAMPLE = 2


def subcluster(particles, neighbourhood_radius, robot_id):
    current_max_dens = -1
    current_best_centre = particles[0]
    for p in particles:
        new_dens = get_density(p, particles, neighbourhood_radius, robot_id)
        if new_dens > current_max_dens:
            current_best_centre = p
            current_max_dens = new_dens
    return current_best_centre


def get_density(i, particles, neighbourhood_radius, r):
    density = 0
    for j in range(len(particles)//DOWNSAMPLE):
        p = particles[j*DOWNSAMPLE]
        d = (i.get_x(r) - p.get_x(r))**2 + (i.get_y(r) - p.get_y(r))**2
        power = -d/((neighbourhood_radius/2)**2)
        density += math.exp(power)
    return density
