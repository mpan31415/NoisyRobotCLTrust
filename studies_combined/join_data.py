from os import getcwd
from pandas import read_csv, concat, Series


def read_data(study_no):
    file_path = getcwd() + "\\study" + str(study_no) + "_results\\all_data.csv"
    df = read_csv(file_path)
    print("\nSuccessfully read study %d results!\n" % study_no)
    return df


df1 = read_data(1)
df2 = read_data(2)

# add "noise" columns
df1['noisy_robot'] = Series(["no" for _ in range(df1.shape[0])])
df2['noisy_robot'] = Series(["yes" for _ in range(df1.shape[0])])

df2['pid'] = df2['pid'].apply(lambda x: int(x+23))

concat_df = concat([df1, df2])

# write joined data to csv file
concat_df.to_csv(getcwd() + "\\studies_combined\\combined_data.csv", index=False)

print("\nFinished writing data to new csv file!\n")