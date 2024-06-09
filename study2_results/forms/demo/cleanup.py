from pandas import read_csv, DataFrame
from os import getcwd
from re import sub


DEMO_DATA_DIR = getcwd() + "\\study2_results\\forms\\demo\\"


############################################################################################
def get_raw_df():
    raw_results_path = DEMO_DATA_DIR + "demo_raw.csv"
    raw_df = read_csv(raw_results_path)
    raw_df = raw_df.iloc[1:, 17:25]
    raw_df = raw_df[['Q1', 'Q2', 'Q3', 'Q4', 'Q5_1', 'Q5_2', 'Q5_3']]
    raw_df = raw_df.iloc[2:26, :]
    return raw_df


############################################################################################
def extract_info(df):
    
    part_id_list = df['Q1'].tolist()
    for i in range(len(part_id_list)):
        part_id_list[i] = int(sub("P", "", part_id_list[i]))
        
    tech_list = df['Q5_1'].tolist()
    games_list = df['Q5_2'].tolist()
    music_list = df['Q5_3'].tolist()
    
    order_list = []
    for part_id in part_id_list:
        if part_id % 2 == 1:
            order_list.append("inc")
        else:
            order_list.append("dec")
        
    # generate new dataframe
    df_dict = {
        'pid': part_id_list,
        'order': order_list,
        'trust_tech': tech_list,
        'play_games': games_list,
        'play_music': music_list
    }
    extracted_df = DataFrame(df_dict)
    return extracted_df


############################################################################################
def cleanup():
    
    raw_df = get_raw_df()
    new_df = extract_info(raw_df)
    
    print(new_df)
    
    print("Successfully read the raw csv results file!\n")
    
    new_df.to_csv(DEMO_DATA_DIR + "demo_cleaned.csv", index=False)
    
    
############################################################################################
def main():
    
    cleanup()
    
    
    
    
############################################################################################
if __name__ == "__main__":
    main()