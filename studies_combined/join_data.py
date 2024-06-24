from os import getcwd
from pandas import read_csv, concat, Series


##########################################################################################
def read_data(study_no):
    file_path = getcwd() + "\\study" + str(study_no) + "_results\\all_data.csv"
    df = read_csv(file_path)
    print("\nSuccessfully read study %d results!\n" % study_no)
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



df1 = read_data(1)
df2 = read_data(2)

# add "noise" columns
df1['noisy_robot'] = Series(["no" for _ in range(df1.shape[0])])
df2['noisy_robot'] = Series(["yes" for _ in range(df1.shape[0])])

############# REPLACE OUTLIERS FOR EACH OF THE MEASURES #############
cols_not_to_check = ['pid','autonomy','auto_grouped','order','trust_tech','play_games','play_music','noisy_robot']
all_cols = list(df1.columns)

############ loop through all columns of dataframe and remove outliers ############
for col in all_cols:
    if col in cols_not_to_check:
        continue
    else:
        print("Removing outliers for column = %s" % col)
        df1 = replace_outliers_within_group(df1, ['auto_grouped', 'order'], col)

df2['pid'] = df2['pid'].apply(lambda x: int(x+23))

concat_df = concat([df1, df2])

# write joined data to csv file
concat_df.to_csv(getcwd() + "\\studies_combined\\combined_data.csv", index=False)

print("\nFinished writing data to new csv file!\n")