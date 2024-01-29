from scipy.ndimage import gaussian_filter1d
from numpy.random import default_rng
from numpy import asarray
import matplotlib.pyplot as plt
from interpolation import cosine_interp_list, linear_interp_list
# from os import getcwd

###### NOISE PARAMS ######
NPOINTS = 101
SIGMA = 4
NUM_INTERP = 49
SCALING = 30

FILE_INDEX = 3

# file 1: sigma = 2, scaling = 50
# file 2: sigma = 3, scaling = 50
# file 3: sigma = 4, scaling = 30


def generate_noise(npoints: int, sigma: int):
    rng = default_rng()
    x = rng.normal(size=npoints)/SCALING
    y = gaussian_filter1d(x, sigma)
    print("Successfully generated %d noise datapoints!" % len(y))
    return x, y


def main():
    
    original, smoothed = generate_noise(NPOINTS, SIGMA)
    
    smoothed_arr = asarray(smoothed)
    smoothed_arr.tofile('robot_noise/noise_csv_files/noise'+str(FILE_INDEX)+'.csv', sep=',')
    print("Successfully saved noise datapoints to file = noise%d.csv!" % FILE_INDEX)
    
    old_indices = [i+1 for i in range(len(smoothed))]
    new_indices = linear_interp_list(old_indices, NUM_INTERP)
    # new_noise = cosine_interp_list(smoothed, NUM_INTERP)
    new_noise = linear_interp_list(smoothed, NUM_INTERP)

    print("Total number of new datapoints = %d" % len(new_noise))
    
    plt.plot(old_indices, original, 'k', label='original')
    plt.plot(old_indices, smoothed, 'bx-', label='smoothed')
    
    plt.plot(new_indices, new_noise, 'ro-', label='cosine interp')
    
    plt.legend()
    plt.grid()
    plt.show()
    


if __name__ == "__main__":
    main()