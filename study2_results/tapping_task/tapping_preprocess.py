from pandas import read_csv, DataFrame
from os import getcwd
from ast import literal_eval


TAPPING_CSV_DIR = getcwd() + "\\windows\\secondary task\\data\\"
TAPPING_TASK_DIR = getcwd() + "\\study2_results\\tapping_task\\"

FIRST_PART_ID = 2
LAST_PART_ID = 10


##########################################################################################
def get_raw_data(part_id):
    df = read_csv(TAPPING_CSV_DIR + "part" + str(part_id) + ".csv")
    df = df.fillna(0)
    print("Finished reading participants %d's raw csv file!" % part_id)
    return df

####################################################################################
def remove_med_auto(df):
    index_med_auto = df[ (df['autonomy'] == 0.4) ].index
    df = df.drop(index_med_auto)
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
def get_separate_errp(errp_list):
    short_errp_list = []
    long_errp_list = []
    for i in range(len(errp_list)):
        if i%2==0:
            short_errp_list.append(abs(errp_list[i]))
        else:
            long_errp_list.append(abs(errp_list[i]))
    short_errp_ave = ave_of_list(short_errp_list)
    long_errp_ave = ave_of_list(long_errp_list)
    return short_errp_ave, long_errp_ave

############################################################################################
def duplicate_list(old_list, times):
    old_len = len(old_list)
    new_list = []
    for i in range(old_len):
        for j in range(times):
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
    auto_id_list = []
    short_ave_err_list = []
    long_ave_err_list = []
    ave_err_list = []
    
    for part_id in range(FIRST_PART_ID, LAST_PART_ID+1):
        
        # get overall dataframe from each participant's csv file
        raw = get_raw_data(part_id)
        raw = raw[['alpha_id', 'ave. error (%)', 'errp_list']]
    
        for n in range(5):
            
            this_alpha_group = raw.iloc[n*6:n*6+6, :]
            
            this_alpha_id = (this_alpha_group['alpha_id'].tolist())[1]
            this_auto_id = round(1.0 - (this_alpha_id / 5.0), 1)
            
            this_err_list = this_alpha_group['ave. error (%)'].tolist()
            this_base_err = this_err_list[0]
            del this_err_list[0]
            this_ave_err = ave_of_list(this_err_list, skip_first=1)
            this_rel_ave_err = this_ave_err - this_base_err
            
            this_errp_lists_list = this_alpha_group['errp_list'].tolist()
            this_base_errp_list = literal_eval(this_errp_lists_list[0])
            this_base_short_errp, this_base_long_errp = get_separate_errp(this_base_errp_list)
            
            rel_short_sum = 0.0
            rel_long_sum = 0.0
            for i in range(2, len(this_errp_lists_list)):
                errp_list = literal_eval(this_errp_lists_list[i])
                short_errp, long_errp = get_separate_errp(errp_list)
                rel_short_errp = short_errp - this_base_short_errp
                rel_long_errp = long_errp - this_base_long_errp
                rel_short_sum += rel_short_errp
                rel_long_sum += rel_long_errp
            rel_short_ave = rel_short_sum / 4
            rel_long_ave = rel_long_sum / 4
            
            # add to lists
            auto_id_list.append(this_auto_id)
            short_ave_err_list.append(rel_short_ave)
            long_ave_err_list.append(rel_long_ave)
            ave_err_list.append(this_rel_ave_err)
    
    grouped_auto_list = get_grouped_auto_list(auto_id_list)
    
    
    # generate new dataframe
    df_dict = {
        'pid': duplicate_list([p for p in range(FIRST_PART_ID, LAST_PART_ID+1)], times=5),
        'autonomy': auto_id_list,
        'auto_grouped': grouped_auto_list,
        'tapping_err_short': short_ave_err_list,
        'tapping_err_long': long_ave_err_list,
        'tapping_err_ave': ave_err_list
    }
    processed_df = DataFrame(df_dict)
    
    processed_df = remove_med_auto(processed_df)
    # write new dataframe to csv file
    dest_path = TAPPING_TASK_DIR + 'tapping_task.csv'
    processed_df.to_csv(dest_path, index=False)
    
    print(" Successfully written pre-processed data to csv file! \n")
    


##########################################################################################
def main():
    
    preprocess()
    


    
##########################################################################################
if __name__ == "__main__":
    
    main()
    