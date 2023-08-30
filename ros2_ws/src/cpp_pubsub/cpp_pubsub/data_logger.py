from csv import writer, DictReader, DictWriter
from os.path import isfile


#####################################################################################################
class DataLogger:

    def __init__(self, csv_dir, part_id, auto_id, traj_id, hxs, hys, hzs, rxs, rys, rzs, txs, tys, tzs, times_from_start, times, datetimes):
        
        self.csv_dir = csv_dir
        self.part_id = part_id
        self.auto_id = auto_id
        self.traj_id = traj_id
        
        self.hxs = hxs
        self.hys = hys
        self.hzs = hzs
        self.num_points = len(self.hxs)

        self.rxs = rxs
        self.rys = rys
        self.rzs = rzs

        self.txs = txs
        self.tys = tys
        self.tzs = tzs

        self.times_from_start = times_from_start
        self.times = times
        self.datetimes = datetimes

        self.header_file_name = self.csv_dir + "part" + str(self.part_id) + "_header.csv"
        self.header_field_names = ['trial_number', 'auto_id', 'traj_id']


    ##############################################################################
    def write_header(self):

        # check if the header file already exists
        file_exists = isfile(self.header_file_name)
        
        # initialize the trial ID to 1 if the file doesn't exist
        trial_id = 1
        
        # if the file exists, read the last trial ID and increment it
        if file_exists:
            with open(self.header_file_name, 'r', newline='') as f:
                reader = DictReader(f)
                for row in reader:
                    trial_id = int(row[self.header_field_names[0]]) + 1

        # define the file name of the new trial
        self.data_file_name = self.csv_dir + "trial" + str(trial_id) + ".csv"

        # open the file in append mode
        with open(self.header_file_name, 'a', newline='') as f:

            # dictionary that we want to add as a new row
            new_trial_data = {self.header_field_names[0]: trial_id,
                              self.header_field_names[1]: self.auto_id,
                              self.header_field_names[2]: self.traj_id
            }

            writer = DictWriter(f, fieldnames=self.header_field_names)
            # write the header row if the file doesn't exist
            if not file_exists:
                writer.writeheader()
            
            # write the data to the file
            writer.writerow(new_trial_data)

        print("\nSuccesfully opened file %s to write header !!!\n" % self.header_file_name)


    ##############################################################################
    def log_data(self):
        
        with open(self.data_file_name, 'w', newline='') as file:

            wr = writer(file)
            # write datapoints [recorded trajectory points]
            for i in range(self.num_points):
                wr.writerow([self.hxs[i], self.hys[i], self.hzs[i], self.rxs[i], self.rys[i], self.rzs[i],
                             self.txs[i], self.tys[i], self.tzs[i], self.times_from_start[i], self.times[i], self.datetimes[i]])

            print("\nSuccesfully opened file %s to log data !!!\n" % self.data_file_name)

    

# ##############################################################################
# def main():

#     # data for testing
#     part_id = 1
#     traj_id = 1
#     auto_id = 3

#     csv_dir = "/home/michael/HRI/ros2_ws/src/cpp_pubsub/data_logging/csv_logs/part" + str(part_id) + "/"

#     # initialize the data logger object, and write to files
#     michael = DataLogger(csv_dir, part_id, traj_id, auto_id, [1, 1, 1, 1, 1], [2, 2, 2, 2, 2], [3, 3, 3, 3, 3])

#     # note, we must call write_header before log_data, since header generates the data_file_name
#     michael.write_header()
#     michael.log_data()



# if __name__ == "__main__":
#     main()
