from pfilter import (
    ParticleFilter,
    gaussian_noise,
    cauchy_noise,
    squared_error,
    independent_sample,
)
from scipy.stats import norm, gamma, uniform
import numpy as np

import sim as vrep
import time
import math
import numpy as np
import sys
import time
from cleanup_vrep import cleanup_vrep
from vrchk import vrchk
from youbot_init import youbot_init
from youbot_drive import youbot_drive
from youbot_hokuyo_init import youbot_hokuyo_init
from youbot_hokuyo import youbot_hokuyo
from youbot_xyz_sensor import youbot_xyz_sensor
from beacon import beacon_init, youbot_beacon
from utils_sim import angdiff
from vrchk import vrchk

alpha = 0


def observation(x):
    # if type(x) == list:
    #     tmp = np.zeros((1, 2))
    #     tmp[0][0] = x[0]
    #     tmp[0][1] = x[1]
    #     x = tmp
    xC = [0.0, 7.300000190734863, -7.300000190734863]
    yC = [-7.300000190734863, 0.0, 5.0]
    distance = np.zeros((x.shape[0], 3))

    # if len(x) == 3:
    #     xdist1 = abs(x[0] - xC[0])
    #     xdist2 = abs(x[0] - xC[1])
    #     xdist3 = abs(x[0] - xC[2])
    #     ydist1 = abs(x[1] - yC[0])
    #     ydist2 = abs(x[1] - yC[1])
    #     ydist3 = abs(x[1] - yC[2])
    #
    #     dist1 = math.sqrt(xdist1**2+ydist1**2)
    #     dist2 = math.sqrt(xdist2**2+ydist2**2)
    #     dist3 = math.sqrt(xdist3**2+ydist3**2)
    #     distance = np.array([dist1, dist2, dist3])

    # else:
    for i, particle in enumerate(x):

        xdist1 = abs(particle[0] - xC[0])
        xdist2 = abs(particle[0] - xC[1])
        xdist3 = abs(particle[0] - xC[2])
        ydist1 = abs(particle[1] - yC[0])
        ydist2 = abs(particle[1] - yC[1])
        ydist3 = abs(particle[1] - yC[2])

        dist1 = math.sqrt(xdist1**2+ydist1**2)
        dist2 = math.sqrt(xdist2**2+ydist2**2)
        dist3 = math.sqrt(xdist3**2+ydist3**2)
        distance[i] = np.array([dist1, dist2, dist3])
    return distance


# very simple linear dynamics: x += dx
def velocity(x):
    forwbackvel = -1
    timestep = .05
    xp = np.array(x)
    xp[0][0] += 0.03*math.sin(alpha) # forwbackvel*math.sin(alpha)*timestep
    xp[0][1] += -0.03*math.cos(alpha) # -forwbackvel*math.cos(alpha)*timestep
    return xp


def example_filter(x, distance_beacon, alpha):
    map_size = 7.5

    # names (this is just for reference for the moment!)
    columns = ["x", "y"]

    alpha = alpha

    # prior sampling function for each variable
    # (assumes x and y are coordinates in the range 0-map_size)
    prior_fn = independent_sample(
        [
         # norm(loc=0, scale=7.5).rvs,
         # norm(loc=0, scale=7.5).rvs,
         norm(loc=x[0][0], scale=0.02).rvs,
         norm(loc=x[0][1], scale=0.02).rvs,
        ]
    )

    # create the filter
    pf = ParticleFilter(
        prior_fn=prior_fn,
        observe_fn=observation,
        n_particles=1000,
        dynamics_fn=velocity,
        n_eff_threshold=0.9,
        noise_fn=lambda x: cauchy_noise(x, sigmas=[0.001, 0.001]),
        weight_fn=lambda x, y: squared_error(x, y, sigma=1),
        resample_proportion=0.6,
        column_names=columns,
    )

    # distance_observed = observation(x)
    # print('***************')
    # print('distance_observed', distance_observed)
    pf.update(observed=distance_beacon)
    # print('weights', pf.weights)
    # print('_______________')

    youbot_position = pf.mean_state
    return youbot_position


if __name__ == "__main__":
    example_filter(x)
