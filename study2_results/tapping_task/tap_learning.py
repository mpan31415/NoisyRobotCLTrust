from pandas import read_csv, DataFrame
from os import getcwd
import matplotlib.pyplot as plt
from ast import literal_eval


# globals
TAPPING_CSV_DIR = getcwd() + "\\windows\\secondary task\\data\\"
TAPPING_TASK_DIR = getcwd() + "\\study2_results\\tapping_task\\"

FIRST_PART_ID = 2
LAST_PART_ID = 25
NUM_PARTICIPANTS = 24


### modes ###
PLOTTING = True
SHORT = False


##########################################################################################
def get_raw_data(part_id):
    df = read_csv(TAPPING_CSV_DIR + "part" + str(part_id) + ".csv")
    df = df.fillna(0)
    print("Finished reading participants %d's raw csv file!" % part_id)
    return df

############################################################################################
def get_separate_errp(errp_list):
    short_errp_list = []
    long_errp_list = []
    for i in range(len(errp_list)):
        if i%2==0:
            short_errp_list.append(abs(errp_list[i]))
        else:
            long_errp_list.append(abs(errp_list[i]))
    short_errp_ave = sum(short_errp_list) / len(short_errp_list)
    long_errp_ave = sum(long_errp_list) / len(long_errp_list)
    return short_errp_ave, long_errp_ave

##########################################################################################
def err_within_round(auto_id: int):
    
    trial1_err_list = []
    trial2_err_list = []
    trial3_err_list = []
    trial4_err_list = []
    trial5_err_list = []
    trial_err_lists = [trial1_err_list, trial2_err_list, trial3_err_list, trial4_err_list, trial5_err_list]
    
    # get summation
    for part_id in range(FIRST_PART_ID, LAST_PART_ID+1):
        # get raw dataframe
        raw = get_raw_data(part_id)
        # first_index = raw.index[raw['alpha_id']==int(5-auto_id)].tolist()[0]-1
        first_index = raw.index[raw['alpha_id']==int(5-auto_id)].tolist()[0]
        this_alpha_group = raw.iloc[first_index:first_index+6, :]
        this_err_list = this_alpha_group['ave. error (%)'].tolist()
        this_base_err = this_err_list[0]
        del this_err_list[0]
        # get short tap & long tap error lists
        short_err_list = []
        long_err_list = []
        for i in range(1, 6):
            this_row = this_alpha_group.iloc[i, :]
            this_errp_list = this_row['errp_list']
            short_errp_ave, long_errp_ave = get_separate_errp(literal_eval(this_errp_list))
            short_err_list.append(short_errp_ave)
            long_err_list.append(long_errp_ave)
        
        short_rel_err_list = [(x - this_base_err) for x in short_err_list]
        long_rel_err_list = [(x - this_base_err) for x in long_err_list]
        
        if SHORT:
            ################### only generate for SHORT tapping ####################
            for trial_id in range(1, 6):
                trial_err_lists[trial_id-1].append(short_rel_err_list[trial_id-1])
        else:
            #################### only generate for LONG tapping ####################
            for trial_id in range(1, 6):
                trial_err_lists[trial_id-1].append(long_rel_err_list[trial_id-1])
    
    # compute averages for each trial (for this autonomy level)
    trial_ave_err_list = []
    for trial_id in range(1, 6):
        trial_err_list = trial_err_lists[trial_id-1]
        
        trial_ave_err = sum(trial_err_list) / len(trial_err_list)
        trial_ave_err_list.append(trial_ave_err)
                
    return trial_err_lists, trial_ave_err_list


pid_low_list = []
pid_high_list = []


############################################################
for auto_id in range(5):
    
    trial_err_lists, trial_ave_err_list = err_within_round(auto_id)
    
    pid_big_list = []
    trial_id_big_list = []
    err_big_list = []
    for trial_id in range(1, 6):
        for part_id in range(1, 25):
            pid_big_list.append(part_id)
            trial_id_big_list.append(trial_id)
            err_big_list.append(trial_err_lists[trial_id-1][part_id-1])
    
    # generate new dataframe
    df_dict = {
        'pid': pid_big_list,
        'trial_id': trial_id_big_list,
        'error': err_big_list
    }
    this_auto_df = DataFrame(df_dict)
    
    # write new dataframe to csv file
    if SHORT:
        dest_path = TAPPING_TASK_DIR + '\\tap_learning_csvs\\tapshort' + str(auto_id) + '.csv'
    else:
        dest_path = TAPPING_TASK_DIR + '\\tap_learning_csvs\\taplong' + str(auto_id) + '.csv'
    this_auto_df.to_csv(dest_path, index=False)
    
    print(" Successfully written pre-processed data to csv file! \n")
    
    
    if PLOTTING:
        plt.subplot(2, 3, auto_id+1)
        plt.plot(trial_ave_err_list, color='r', label='average errors')
        plt.ylim([-10.0, 30.0])
        # Naming the x-axis, y-axis and the whole graph
        plt.xlabel("Trial ID")
        plt.ylabel("Average Error (%)")
        if SHORT:
            plt.title("Tapping SHORT Error vs. Trial ID (Auto " + str(auto_id) + ")")
        else:
            plt.title("Tapping LONG Error vs. Trial ID (Auto " + str(auto_id) + ")")
        plt.legend()
    
    
if PLOTTING:
    plt.show()