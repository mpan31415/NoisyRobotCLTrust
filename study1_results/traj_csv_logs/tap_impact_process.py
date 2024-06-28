# imports
from os import getcwd
from pandas import read_csv, DataFrame
# import matplotlib.pyplot as plt


# globals
CSV_LOGS_DIR = getcwd() + "\\study1_results\\traj_csv_logs\\"
PRIMARY_TASK_DIR = getcwd() + "\\study1_results\\traj_csv_logs\\"

FIRST_PART_ID = 1
LAST_PART_ID = 24


##########################################################################################
def get_raw_data(part_id):
    my_data_file = CSV_LOGS_DIR + "\part" + str(part_id) + "\part" + str(part_id) + "_header.csv"
    df = read_csv(my_data_file)
    return df


##########################################################################################
def scale_list(mylist, alpha):
    return [alpha*mylist[i] for i in range(len(mylist))]


##########################################################################################
def compare_to_baseline(part_id):
    
    # get overall dataframe
    raw = get_raw_data(part_id)
    
    # get baseline tracking performance
    base_group = raw.iloc[0:5, :]
    base_err_list = (base_group['overall_ave'].tolist())
    ave_baseline_err = sum(base_err_list) / len(base_err_list)
    
    # get errors from the 2 interested autonomy levels
    ave_first_round_err = 0.0
    ave_no_autonomy_err = 0.0
    for i in range(1, 6):
        this_group = raw.iloc[i*5:i*5+5, :]
        this_alpha_id = (this_group['alpha_id'].tolist())[0]
        if this_alpha_id == 5:
            no_auto_err_list = (this_group['overall_ave'].tolist())
            ave_no_autonomy_err = sum(no_auto_err_list) / len(no_auto_err_list)
        if this_alpha_id == 3:
            med_auto_err_list = (this_group['overall_ave'].tolist())
            ave_first_round_err = sum(med_auto_err_list) / len(med_auto_err_list)
        
    return ave_baseline_err, ave_first_round_err, ave_no_autonomy_err


##########################################################################################
def main():
    
    index_list = []
    base_err_list = []
    first_round_err_list = []
    no_autonomy_err_list = []
    
    for part_id in range(FIRST_PART_ID, LAST_PART_ID+1):
        base, first, no = compare_to_baseline(part_id)
        base_err_list.append(base)
        first_round_err_list.append(first)
        no_autonomy_err_list.append(no)
        
        index_list.append(part_id)
    
    # convert error unit to [cm]
    base_err_list = scale_list(base_err_list, 100)
    first_round_err_list = scale_list(first_round_err_list, 100)
    no_autonomy_err_list = scale_list(no_autonomy_err_list, 100)
    
    # generate new dataframe
    df_dict = {
        'pid': index_list,
        'base_err': base_err_list,
        'round1_err': first_round_err_list,
        'no_auto_err': no_autonomy_err_list
    }
    processed_df = DataFrame(df_dict)
    
    # write new dataframe to csv file
    dest_path = PRIMARY_TASK_DIR + '\\tap_impact.csv'
    processed_df.to_csv(dest_path, index=False)
    
    print(" Successfully written pre-processed data to csv file! \n")
    
        
        
##########################################################################################
if __name__ == "__main__":
    main()