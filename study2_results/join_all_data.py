from pandas import read_csv, DataFrame, merge
from os import getcwd
from numpy import repeat


DEMO_DATA_DIR = getcwd() + "\\study2_results\\forms\\demo\\"
MAIN_FORM_DIR = getcwd() + "\\study2_results\\forms\\main\\"
PRIMARY_TASK_DIR = getcwd() + "\\study2_results\\primary_task\\"
TAPPING_TASK_DIR = getcwd() + "\\study2_results\\tapping_task\\"
PUPIL_DATA_DIR = getcwd() + "\\study2_results\\pupil_index\\"
ALL_DATA_DIR = getcwd() + "\\study2_results\\"


##########################################################################################
def get_demo_data():
    df = read_csv(DEMO_DATA_DIR + "demo_cleaned.csv")
    print("Finished reading demographics data!")
    return df

##########################################################################################
def get_main_data():
    df = read_csv(MAIN_FORM_DIR + "main_cleaned.csv")
    print("Finished reading main form data!")
    return df

##########################################################################################
def get_primary_task_data():
    df = read_csv(PRIMARY_TASK_DIR + "primary_task.csv")
    print("Finished reading primary task data!")
    return df

##########################################################################################
def get_tapping_task_data():
    df = read_csv(TAPPING_TASK_DIR + "tapping_task.csv")
    print("Finished reading tapping task data!")
    return df

##########################################################################################
def get_pupil_data():
    df = read_csv(PUPIL_DATA_DIR + "pupil_index.csv")
    print("Finished reading pupil index data!")
    return df


##########################################################################################
def main():
    
    # get processed dataframes
    demo_df = get_demo_data()
    main_df = get_main_data()
    traj_df = get_primary_task_data()
    tap_df = get_tapping_task_data()
    pupil_df = get_pupil_data()
    
    # join dataframes
    m1 = merge(traj_df, tap_df, how="inner", on=['pid', 'autonomy', 'auto_grouped'])
    m2 = merge(m1, pupil_df, how="inner", on=['pid', 'autonomy', 'auto_grouped'])
    m3 = merge(m2, main_df, how="inner", on=['pid', 'autonomy', 'auto_grouped'])
    joined_df = merge(m3, demo_df, how="inner", on=['pid'])    # automatically casted
    
    # perform back slicing - only keep the second round of each autonomy block (2 rounds)
    low_auto = 0.2
    high_auto = 0.8
    joined_df = joined_df[(joined_df['autonomy']==low_auto) | (joined_df['autonomy']==high_auto)]
    
    # write new dataframe to csv file
    dest_path = ALL_DATA_DIR + 'all_data.csv'
    joined_df.to_csv(dest_path, index=False)
    
    print(" \n Successfully written ALL DATA csv file! \n")
    
    
##########################################################################################
if __name__ == "__main__":
    main()