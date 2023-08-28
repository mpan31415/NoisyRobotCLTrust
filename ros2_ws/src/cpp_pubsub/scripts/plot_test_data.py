from pandas import read_csv
import matplotlib.pyplot as plt

file_name = "/home/michael/HRI/ros2_ws/src/cpp_pubsub/data_logging/csv_logs/part0/trial4.csv"

df = read_csv(file_name, header=None)

rxs = df.iloc[:,6].tolist()
rys = df.iloc[:,7].tolist()
rzs = df.iloc[:,8].tolist()

print(len(rxs))

ax = plt.figure().add_subplot(projection='3d')
ax.plot(rxs, rys, rzs, label="robot_positions")
ax.legend()

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

plt.show()