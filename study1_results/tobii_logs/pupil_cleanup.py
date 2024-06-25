from os import getcwd
from pandas import read_csv


PUPIL_DATA_DIR = getcwd() + "\\study1_results\\tobii_logs\\"


def read_data():
    file_path = PUPIL_DATA_DIR + "\\tobii_preprocessed.csv"
    df = read_csv(file_path)
    # index_med_auto = df[ (df['autonomy'] == 0.4) ].index
    # df = df.drop(index_med_auto)
    return df


raw = read_data()

raw.rename(columns={"new_left": "left_index", "new_right": "right_index", "new_ave": "pupil_index"}, inplace=True)

raw = raw[['pid', 'autonomy', 'auto_grouped', 'pupil_diameter', 'left_index', 'right_index', 'pupil_index']]

raw.to_csv(PUPIL_DATA_DIR + "\\pupil_index.csv", index=False)

print("\n Successfully written new csv file to tobii logs folder! \n")