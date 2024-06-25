from os import getcwd
from pandas import read_csv, concat, Series


##########################################################################################
def read_data(study_no):
    all_data_path = getcwd() + "\\study" + str(study_no) + "_results\\all_data.csv"
    sliced_data_path = getcwd() + "\\study" + str(study_no) + "_results\\sliced_data.csv"
    all_df = read_csv(all_data_path)
    sliced_df = read_csv(sliced_data_path)
    print("\nSuccessfully read study %d results, both ALL and SLICED!\n" % study_no)
    return all_df, sliced_df


##########################################################################################
all_df1, sliced_df1 = read_data(1)
all_df2, sliced_df2 = read_data(2)

# add "noise" columns
all_df1['noisy_robot'] = Series(["no" for _ in range(all_df1.shape[0])])
sliced_df1['noisy_robot'] = Series(["no" for _ in range(sliced_df1.shape[0])])

all_df2['noisy_robot'] = Series(["yes" for _ in range(all_df2.shape[0])])
sliced_df2['noisy_robot'] = Series(["yes" for _ in range(sliced_df2.shape[0])])

all_df2['pid'] = all_df2['pid'].apply(lambda x: int(x+23))
sliced_df2['pid'] = sliced_df2['pid'].apply(lambda x: int(x+23))


################## CONCATENATE DATAFRAMES ##################
all_concat_df = concat([all_df1, all_df2])
sliced_concat_df = concat([sliced_df1, sliced_df2])

# write joined data to csv file
all_concat_df.to_csv(getcwd() + "\\studies_combined\\all_combined_data.csv", index=False)
sliced_concat_df.to_csv(getcwd() + "\\studies_combined\\sliced_combined_data.csv", index=False)

print("\nFinished writing data to new csv file!\n")