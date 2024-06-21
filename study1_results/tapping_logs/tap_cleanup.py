from pandas import read_csv
from os import getcwd


TAPPING_DATA_DIR = getcwd() + "\\study1_results\\tapping_logs\\"


def read_data():
    file_path = TAPPING_DATA_DIR + "tap_preprocessed.csv"
    df = read_csv(file_path)
    index_med_auto = df[ (df['autonomy'] == 0.4) ].index
    df = df.drop(index_med_auto)
    return df


raw = read_data()
raw = raw[['pid', 'autonomy', 'auto_grouped', 'tapping_err_short', 'tapping_err_long', 'tapping_err_ave']]

raw.to_csv(TAPPING_DATA_DIR + "tapping_task.csv", index=False)

print(" Successfully written pre-processed data to csv file! \n")