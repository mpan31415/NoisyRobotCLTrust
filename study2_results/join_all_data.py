from pandas import read_csv, merge
from os import getcwd


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
def replace_outliers_with_mean(series):
    Q1 = series.quantile(0.25)
    Q3 = series.quantile(0.75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
    # Find the mean of the non-outliers
    mean_value = series[(series >= lower_bound) & (series <= upper_bound)].mean()
    # Replace outliers with the mean
    series = series.apply(lambda x: mean_value if x < lower_bound or x > upper_bound else x)
    return series

##########################################################################################
def replace_outliers_within_group(df, group_columns, target_column):
    # Reset index to ensure group_columns are not in the index
    df = df.reset_index(drop=True)

    # Function to calculate and replace outliers within each group
    def replace(group):
        Q1 = group[target_column].quantile(0.25)
        Q3 = group[target_column].quantile(0.75)
        IQR = Q3 - Q1
        lower_bound = Q1 - 1.5 * IQR
        upper_bound = Q3 + 1.5 * IQR
        mean_value = group[(group[target_column] >= lower_bound) & (group[target_column] <= upper_bound)][target_column].mean()
        group[target_column] = group[target_column].apply(lambda x: mean_value if x < lower_bound or x > upper_bound else x)
        return group

    # Apply the function to each subgroup
    return df.groupby(group_columns).apply(replace).reset_index(drop=True)


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
    
    
    ############# REPLACE OUTLIERS FOR EACH OF THE MEASURES #############
    cols_not_to_check = ['pid','autonomy','auto_grouped','order','trust_tech','play_games','play_music']
    all_cols = list(joined_df.columns)
    
    ############ loop through all columns of dataframe and remove outliers ############
    for col in all_cols:
        if col in cols_not_to_check:
            continue
        else:
            print("Removing outliers for column = %s" % col)
            joined_df = replace_outliers_within_group(joined_df, ['auto_grouped', 'order'], col)
            
    
    # write new dataframe to csv file
    dest_path = ALL_DATA_DIR + 'all_data.csv'
    joined_df.to_csv(dest_path, index=False)
    
    print(" \n Successfully written ALL DATA csv file! \n")
    
    
##########################################################################################
if __name__ == "__main__":
    main()