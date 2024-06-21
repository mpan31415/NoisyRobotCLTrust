from pandas import read_csv
from os import getcwd
import matplotlib.pyplot as plt

# globals
CSV_LOGS_DIR = getcwd() + "\\ros2_ws\\src\\cpp_pubsub\\data_logging\\csv_logs\\"
PRIMARY_TASK_DIR = getcwd() + "\\study2_results\\primary_task\\"

FIRST_PART_ID = 2
LAST_PART_ID = 25
NUM_PARTICIPANTS = 24


##########################################################################################
def get_raw_data(part_id):
    my_data_file = CSV_LOGS_DIR + "\part" + str(part_id) + "\part" + str(part_id) + "_header.csv"
    df = read_csv(my_data_file)
    print("Successfully read participant %d's data" % part_id)
    return df
    
    
##########################################################################################
def err_within_round():
    
    trial_list = [1, 2, 3, 4, 5]
    
    round0_err_list = [0, 0, 0, 0, 0]
    round1_err_list = [0, 0, 0, 0, 0]
    round2_err_list = [0, 0, 0, 0, 0]
    round3_err_list = [0, 0, 0, 0, 0]
    round4_err_list = [0, 0, 0, 0, 0]
    round5_err_list = [0, 0, 0, 0, 0]
    rounds_err_lists = [round0_err_list, round1_err_list, round2_err_list, round3_err_list, round4_err_list, round5_err_list]
    
    # get summation
    for part_id in range(FIRST_PART_ID, LAST_PART_ID+1):
        # get overall dataframe
        raw = get_raw_data(part_id)
        
        for round_index in range(6):
            this_group = raw.iloc[round_index*5:round_index*5+5, :]
            ave_error_list = this_group['overall_ave'].apply(lambda x: x*100).tolist()   # convert into [cm]
            for trial_index in range(5):
                rounds_err_lists[round_index][trial_index] += ave_error_list[trial_index]
    
    # compute averages
    for round_index in range(6):
        for trial_index in range(5):
            rounds_err_lists[round_index][trial_index] /= NUM_PARTICIPANTS
                
    for round_index in range(6):
        plt.subplot(2, 3, round_index+1)
        plt.plot(trial_list, rounds_err_lists[round_index], color='r', label='average_error')
        plt.scatter(trial_list, rounds_err_lists[round_index], color='b', label='average_error')
        plt.ylim([1, 4])
        # Naming the x-axis, y-axis and the whole graph
        plt.xlabel("Trial ID")
        plt.ylabel("Average error (cm)")
        plt.title("Average Error vs. Trial Number (Round " + str(round_index) + ")")
        plt.legend()
        
    plt.show()
    
    print(rounds_err_lists)
            
        

# ##########################################################################################
# def compare_to_baseline():
    
#     # get overall dataframe
#     raw = get_raw_data()
    
#     # get baseline tracking performance
#     base_group = raw.iloc[0:5, :]
#     base_traj_list = (base_group['traj_id'].tolist())
#     base_err_list = (base_group['overall_ave'].tolist())

#     # sort
#     base_traj_list, base_err_list = (list(t) for t in zip(*sorted(zip(base_traj_list, base_err_list))))
    
#     for i in range(1, 6):
        
#         this_group = raw.iloc[i*5:i*5+5, :]
#         this_alpha_id = (this_group['alpha_id'].tolist())[0]
#         if this_alpha_id == 5:
#             traj_id_list = (this_group['traj_id'].tolist())
#             this_err_list = (this_group['overall_ave'].tolist())
#             break
    
#     # sort them for plotting
#     traj_id_list, this_err_list = (list(t) for t in zip(*sorted(zip(traj_id_list, this_err_list))))
    
#     # Plotting both the curves simultaneously
#     plt.plot(base_traj_list, base_err_list, color='r', label='baseline_error')
#     plt.plot(traj_id_list, this_err_list, color='g', label='average_error')
    
#     # Naming the x-axis, y-axis and the whole graph
#     plt.xlabel("Traj ID")
#     plt.ylabel("Average error (m)")
#     plt.title("Error in Trajectory Tracking vs. Trajectories")
    
#     plt.legend()
#     plt.show()


##########################################################################################
if __name__ == "__main__":
    
    err_within_round()
    