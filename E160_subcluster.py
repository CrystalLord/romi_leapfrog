import math

DOWNSAMPLE = 2

def subcluster(particles, neighbourhood_radius):
    current_max_dens = -1
    current_best_centre = particles[0]
    for p in particles:
        new_dens = get_density(p, particles, neighbourhood_radius)
        if new_dens > current_max_dens:
            current_best_centre = p
            current_max_dens = new_dens
    return current_best_centre

def get_density(i, particles, neighbourhood_radius):
    density = 0
    for j in range(len(particles)//DOWNSAMPLE):
        p = particles[j*DOWNSAMPLE]
        d = (i.x - p.x)**2 + (i.y - p.y)**2
        power = -d/((neighbourhood_radius/2)**2)
        density += math.exp(power)
    return density
