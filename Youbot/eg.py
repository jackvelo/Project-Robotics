from pfilter import ParticleFilter, gaussian_noise, squared_error, independent_sample
from scipy.stats import norm, gamma, uniform
columns = ["x", "y", "radius", "dx", "dy"]

# prior sampling function for each variable
# (assumes x and y are coordinates in the range 0-32)
prior_fn = independent_sample([uniform(loc=0, scale=32).rvs,
                               uniform(loc=0, scale=32).rvs,
                               gamma(a=2, loc=0, scale=10).rvs,
                               norm(loc=0, scale=0.5).rvs,
                               norm(loc=0, scale=0.5).rvs])


def blob(x):
    """Given an Nx3 matrix of blob positions and size,
    create N img_size x img_size images, each with a blob drawn on
    them given by the value in each row of x

    One row of x = [x,y,radius]."""
    y = np.zeros((x.shape[0], img_size, img_size))
    for i, particle in enumerate(x):
        rr, cc = skimage.draw.circle(
            particle[0], particle[1], max(particle[2], 1), shape=(img_size, img_size)
        )
        y[i, rr, cc] = 1
    return y


# very simple linear dynamics: x += dx
def velocity(x):
    xp = np.array(x)
    xp[0:2] += xp[3:5]
    return xp


# create the filter
pf = ParticleFilter(
    prior_fn=prior_fn,
    observe_fn=blob,
    n_particles=200,
    dynamics_fn=velocity,
    noise_fn=lambda x:
        gaussian_noise(x, sigmas=[0.2, 0.2, 0.1, 0.05, 0.05]),
    weight_fn=lambda x, y: squared_error(x, y, sigma=2),
    resample_proportion=0.1,
    column_names=columns)

# assuming image of the same dimensions/type as blob will produce
pf.update(image)
