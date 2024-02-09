from scipy.ndimage import gaussian_filter1d
from numpy.random import default_rng
from numpy import asarray
import matplotlib.pyplot as plt
# from interpolation import cosine_interp_list, linear_interp_list
# from os import getcwd

###### NOISE PARAMS ######
NPOINTS = 101
SIGMA = 4
NUM_INTERP = 49
SCALING = 10

NUM_FILES = 10

TARGET_ERR = 0.024   # averaged from participants data [m]
ERR_TOLERANCE = 0.003     # variance in participant error [m]
START_TOLERANCE = 0.01   # first error point < 0.01 m


def compute_my_rmse(noise_list):
    abs_vals_list = [abs(num) for num in noise_list]
    mean_err = sum(abs_vals_list) / len(abs_vals_list)
    return mean_err


def generate_noise(npoints: int, sigma: int):
    rng = default_rng()
    x = rng.normal(size=npoints)/SCALING
    y = gaussian_filter1d(x, sigma)
    # print("Successfully generated %d noise datapoints!" % len(y))
    return x, y


def main():
    
    valid_files = 0
    file_index = 1
    
    while valid_files < NUM_FILES:
        original, smoothed = generate_noise(NPOINTS, SIGMA)
        smoothed_arr = asarray(smoothed)
        noise_list = smoothed_arr.tolist()      # in [m]
        my_rmse = compute_my_rmse(noise_list)   # in [m]

        if abs(my_rmse - TARGET_ERR) < ERR_TOLERANCE and abs(noise_list[0]) < START_TOLERANCE:
            smoothed_arr.tofile('robot_noise/candidate_noise/noise'+str(file_index)+'.csv', sep=',')
            print("My RMSE = %.3f, Saved to file = noise%d.csv !" % (my_rmse, file_index))
            valid_files += 1
            file_index += 1
        else:
            print("Failed to generate noise to within the error tolerance!")
        
        # old_indices = [i+1 for i in range(len(smoothed))]
        # new_indices = linear_interp_list(old_indices, NUM_INTERP)
        # # new_noise = cosine_interp_list(smoothed, NUM_INTERP)
        # new_noise = linear_interp_list(smoothed, NUM_INTERP)
        
        # plt.plot(old_indices, original, 'k', label='original')
        # plt.plot(old_indices, smoothed, 'bx-', label='smoothed')
        
        # plt.plot(new_indices, new_noise, 'ro-', label='cosine interp')
        
        # plt.legend()
        # plt.grid()
        # plt.show()
    


if __name__ == "__main__":
    main()