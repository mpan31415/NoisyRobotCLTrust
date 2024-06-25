from pandas import read_csv, DataFrame
from os import getcwd
from numpy import zeros


CSV_LOGS_DIR = getcwd() + "\\study1_results\\traj_csv_logs\\"
PRIMARY_TASK_DIR = getcwd() + "\\study1_results\\traj_csv_logs\\"

FIRST_PART_ID = 1
LAST_PART_ID = 24


##########################################################################################
def get_raw_data(part_id):
    df = read_csv(CSV_LOGS_DIR + "part" + str(part_id) + "\\part" + str(part_id) + "_header.csv")
    df.fillna(0, inplace=True)
    print("Finished reading participants %d's raw csv file!" % part_id)
    return df

####################################################################################
def remove_med_auto(df:DataFrame):
    index_med_auto = df[ (df['autonomy'] == 0.4) ].index
    df.drop(index_med_auto, inplace=True)
    return df

####################################################################################
def get_grouped_auto_list(auto_list):
    grouped_list = []
    for auto in auto_list:
        if auto > 0.4:
            grouped_list.append("high")
        else:
            grouped_list.append("low")
    return grouped_list

############################################################################################
def list_times_five(old_list):
    old_len = len(old_list)
    new_list = []
    for i in range(old_len):
        for j in range(5):
            new_list.append(old_list[i])
    return new_list

##########################################################################################
def ave_of_list(my_list, skip_first=0):
    sum = 0.0
    count = 0
    for i in range(skip_first, len(my_list)):
        sum += my_list[i]
        count += 1
    return sum / count


##########################################################################################
def preprocess():
    
    # separate into alpha level groups and get average errors
    part_id_list = []
    auto_id_list = []
    
    human_ave_err_list = []
    robot_ave_err_list = []
    overall_ave_err_list = []
    
    for part_id in range(FIRST_PART_ID, LAST_PART_ID+1):
        
        # get overall dataframe from each participant's csv file
        raw = get_raw_data(part_id)
        raw = raw[['alpha_id', 'human_ave', 'overall_ave']]
        raw = raw.iloc[5:30, :]
    
        for n in range(5):
            
            this_alpha_group = raw.iloc[n*5:n*5+5, :]
            this_alpha_id = (this_alpha_group['alpha_id'].tolist())[0]
            this_auto_id = round(1.0 - (this_alpha_id / 5.0), 1)
            
            # get error lists
            human_ave_errs = this_alpha_group['human_ave'].tolist()
            robot_ave_errs = zeros(len(human_ave_errs)).tolist()
            overall_ave_errs = this_alpha_group['overall_ave'].tolist()
            
            # compute the average of each error list
            human_ave_err = ave_of_list(human_ave_errs, skip_first=1) * 100
            robot_ave_err = ave_of_list(robot_ave_errs, skip_first=1) * 100
            overall_ave_err = ave_of_list(overall_ave_errs, skip_first=1) * 100
            
            # add to storage lists
            part_id_list.append(part_id)
            auto_id_list.append(this_auto_id)
            
            human_ave_err_list.append(human_ave_err)
            robot_ave_err_list.append(robot_ave_err)
            overall_ave_err_list.append(overall_ave_err)
            
    
    # get the categorical autonomy variable list
    grouped_auto_list = get_grouped_auto_list(auto_id_list)
    
    # generate new dataframe
    df_dict = {
        'pid': part_id_list,
        'autonomy': auto_id_list,
        'auto_grouped': grouped_auto_list,
        'human_traj_err': human_ave_err_list,
        'robot_traj_err': robot_ave_err_list,
        'overall_traj_err': overall_ave_err_list
    }
    processed_df = DataFrame(df_dict)
    
    # remove the first round (medium autonomy condition)
    # processed_df = remove_med_auto(processed_df)
    
    # write new dataframe to csv file
    dest_path = PRIMARY_TASK_DIR + '\\primary_task.csv'
    processed_df.to_csv(dest_path, index=False)
    
    print(" Successfully written pre-processed data to csv file! \n")



##########################################################################################
def main():
    
    preprocess()


    
##########################################################################################
if __name__ == "__main__":
    
    main()