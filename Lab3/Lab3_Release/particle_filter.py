# Saumya Jain, Zachary Wilson
import grid
from particle import Particle
import utils
import setting
import math
import numpy as np

def motion_update(particles, odom, grid):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*
        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used for boundary checking
        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """

    motion_particles = []
    dh = odom[2]

    for particle in particles:
        x_coor, y_coor, h_coor = particle.xyh
        dx, dy = utils.rotate_point(odom[0], odom[1], h_coor)

        new_noisy_x, new_noisy_y, new_noisy_h = utils.add_odometry_noise((x_coor + dx, y_coor + dy, h_coor + dh), setting.ODOM_HEAD_SIGMA, setting.ODOM_TRANS_SIGMA)
        new_noisy_particle = Particle(new_noisy_x, new_noisy_y, new_noisy_h)
        motion_particles.append(new_noisy_particle)

    # motion_particles = particles
    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before measurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    # Size of Samples and lists for all data
    SAMPLE_SIZE = 200
    updated_measurement_particles, particles_with_probability, particle_weights, probability_of_particles = [], [], [], []
    normalizing_factor = 0.0

    # updating the weight for each particle 
    for particle in particles:

        if (grid.is_free(particle.x, particle.y) == False) or (grid.is_in(particle.x, particle.y) == False):
            # This is the case when particle is out of range from our grid, we make the weight 0
            particle_weights.append(0.0)
            particles_with_probability.append(particle)
        
        else:
            # This is the case when the particle is in the actual grid
            list_of_markers = particle.read_markers(grid)
            # Pairs of the types (robot_marker, particle_marker)
            pairs = []

            for r_marker in measured_marker_list: 
                minimum_distance = float('inf')

                if len(list_of_markers) > 0:
                    closest_robot_marker = r_marker
                    closest_particle_marker = list_of_markers[0]

                    for p_marker in list_of_markers:
                        distance = utils.grid_distance(r_marker[0], r_marker[1], p_marker[0], p_marker[1])
                        # Replace the closest marker if calculated distance is less than our current minimum distance
                        if distance < minimum_distance:
                            closest_robot_marker = r_marker
                            closest_particle_marker = p_marker
                            minimum_distance = distance
                    
                    # Add our pair to the list and remove from list of markers left to analyze
                    pairs.append((closest_robot_marker, closest_particle_marker))
                    list_of_markers.remove(closest_particle_marker)

            # We start with full probability
            probability = 1.0

            for r_marker, p_marker in pairs:
                distance = utils.grid_distance(r_marker[0], r_marker[1], p_marker[0], p_marker[1])
                angle = utils.diff_heading_deg(r_marker[2], p_marker[2])
                distance_measure = (distance ** 2) / (2 * (setting.MARKER_TRANS_SIGMA ** 2))
                angle_measure = (angle ** 2) / (2 * (setting.MARKER_ROT_SIGMA ** 2))

                # Calculating probabilities there is a match and no match. Use the higher of the 2 values to adjust probability with.
                probability_of_match = math.exp(-1 * (distance_measure + angle_measure))
                probability_of_no_match = setting.SPURIOUS_DETECTION_RATE * setting.DETECTION_FAILURE_RATE

                probability *= max((probability_of_match, probability_of_no_match))

            probability *= setting.SPURIOUS_DETECTION_RATE ** (len(measured_marker_list) - len(pairs))
            probability *= setting.DETECTION_FAILURE_RATE ** len(list_of_markers)
            
            particle_weights.append(probability)
            normalizing_factor += probability
            particles_with_probability.append(particle)

    # Normalizing all the particle probabilities we have just calculated
    if normalizing_factor == 0.0:
        probability_of_particles = [1/len(particle_weights)] * len(particle_weights)
    else:
        for particle_weight in particle_weights:
            probability_of_particles.append(particle_weight/normalizing_factor)

    # Resampling based on the calculated weights
    updated_measurement_particles = np.random.choice(particles_with_probability, size = len(particles_with_probability) - SAMPLE_SIZE, replace = True, p = probability_of_particles).tolist()
    particles_random = Particle.create_random(count = SAMPLE_SIZE, grid = grid)

    return (updated_measurement_particles + particles_random)