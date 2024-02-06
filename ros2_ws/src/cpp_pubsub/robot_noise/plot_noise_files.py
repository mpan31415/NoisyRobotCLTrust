from pandas import read_csv
import matplotlib.pyplot as plt
from os import getcwd

files_folder = getcwd() + "/robot_noise/noise_csv_files/"

plt.figure(figsize=(14,4))

for file_index in range(1, 11):
    raw_noise_list = read_csv(files_folder+"noise"+str(file_index)+".csv").columns.tolist()
    noise_list = [float(n) for n in raw_noise_list]
    plt.subplot(2, 5, file_index)
    plt.plot(noise_list)
    plt.title("Noise%d.csv" % file_index)
    
plt.show()