from pandas import read_csv, DataFrame
from os import getcwd
from re import sub
from math import isnan


MAIN_DATA_DIR = getcwd() + "\\study1_results\\questionnaire_logs\\"

FIRST_PART_ID = 1
LAST_PART_ID = 24


##########################################################################################
def get_raw_data():
    big_df = read_csv(MAIN_DATA_DIR + "Results.csv")
    num_cols = big_df.shape[1]
    df = big_df.iloc[:, 17:num_cols]
    df = df.fillna(1)
    print("\n Finished reading raw csv file! \n")
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
def list_times_five(old_list):
    old_len = len(old_list)
    new_list = []
    for i in range(old_len):
        for j in range(5):
            new_list.append(old_list[i])
    return new_list


##########################################################################################
def preprocess():
    
    # get overall dataframe
    raw = get_raw_data()
    # demo_df = get_demo_data()
    # order_list = demo_df['order'].tolist()
    # trust_tech_list = demo_df['trust_tech'].tolist()
    # play_games_list = demo_df['play_games'].tolist()
    # play_music_list = demo_df['play_music'].tolist()
    
    # get first 2 columns as lists
    part_id_list = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 0].tolist()
    for i in range(len(part_id_list)):
        part_id_list[i] = int(sub("P", "", part_id_list[i]))
    alpha_id_list_str = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 1].tolist()
    auto_id_list = [round(float(1.0 - float(alpha_id_list_str[i])/5.0), 1) for i in range(len(alpha_id_list_str))]
    
    # get categorical autonomy variable list
    grouped_auto_list = get_grouped_auto_list(auto_id_list)
    
    
    ##################################################
    #################### NASA-TLX ####################
    
    # get lists for each tlx dimension (as floating point numbers)
    tlx1_list = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 2].apply(lambda x: float(x)).tolist()
    tlx2_list = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 3].apply(lambda x: float(x)).tolist()
    tlx3_list = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 4].apply(lambda x: float(x)).tolist()
    tlx4_list = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 5].apply(lambda x: float(x)).tolist()
    tlx5_list = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 6].apply(lambda x: float(x)).tolist()
    tlx6_list = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 7].apply(lambda x: float(21.0 - float(x))).tolist()
    
    # get average values of TLX
    tlx_ave_list = []
    for i in range(len(tlx1_list)):
        tlx_ave_list.append(sum([tlx1_list[i], tlx2_list[i], tlx3_list[i], tlx4_list[i], tlx5_list[i], tlx6_list[i]]) / 6.0)
    
    
    ##############################################
    #################### MDMT ####################
    
    def adjust_mdmt(x:str):
        n = float(x) - 1
        if n == 8.0:
            return float("nan")
        return n
    
    def replace_nan_with_mean(lst):
        non_nan_values = [x for x in lst if not isnan(x)]
        lst_mean = sum(non_nan_values) / len(non_nan_values)
        return [lst_mean if isnan(x) else x for x in lst]
    
    # get list for each mdmt dimension
    mdmt1_list = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 8].apply(lambda x: adjust_mdmt(x)).tolist()
    mdmt2_list = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 9].apply(lambda x: adjust_mdmt(x)).tolist()
    mdmt3_list = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 10].apply(lambda x: adjust_mdmt(x)).tolist()
    mdmt4_list = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 11].apply(lambda x: adjust_mdmt(x)).tolist()
    mdmt5_list = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 12].apply(lambda x: adjust_mdmt(x)).tolist()
    mdmt6_list = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 13].apply(lambda x: adjust_mdmt(x)).tolist()
    mdmt7_list = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 14].apply(lambda x: adjust_mdmt(x)).tolist()
    mdmt8_list = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 15].apply(lambda x: adjust_mdmt(x)).tolist()
    
    # replace "nan" values with mean of each dimension
    mdmt1_list = replace_nan_with_mean(mdmt1_list)
    mdmt2_list = replace_nan_with_mean(mdmt2_list)
    mdmt3_list = replace_nan_with_mean(mdmt3_list)
    mdmt4_list = replace_nan_with_mean(mdmt4_list)
    mdmt5_list = replace_nan_with_mean(mdmt5_list)
    mdmt6_list = replace_nan_with_mean(mdmt6_list)
    mdmt7_list = replace_nan_with_mean(mdmt7_list)
    mdmt8_list = replace_nan_with_mean(mdmt8_list)
    
    # get sub-group average lists
    mdmt_reliable_ave_list = []
    mdmt_capable_ave_list = []
    for i in range(len(mdmt1_list)):
        reliable_ave = float((mdmt1_list[i] + mdmt3_list[i] + mdmt5_list[i] + mdmt7_list[i]) / 4.0)
        capable_ave = float((mdmt2_list[i] + mdmt4_list[i] + mdmt6_list[i] + mdmt8_list[i]) / 4.0)
        mdmt_reliable_ave_list.append(reliable_ave)
        mdmt_capable_ave_list.append(capable_ave)
        
    # get average values of MDMT
    mdmt_ave_list = []
    for i in range(len(mdmt1_list)):
        mdmt_ave = sum([mdmt1_list[i], mdmt2_list[i], mdmt3_list[i], mdmt4_list[i], mdmt5_list[i], mdmt6_list[i], mdmt7_list[i], mdmt8_list[i]]) / 8.0
        mdmt_ave_list.append(mdmt_ave)
    
    
    ############################################################
    #################### Perceived Autonomy ####################

    # convert perceived autonomy to integers
    per_auto_list = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 16].apply(lambda x: int(x)).tolist()
    
    
    ############################################################
    #################### Single-Scale Trust ####################
    
    # convert single-scale trust to integers
    single_trust_list = raw.iloc[(FIRST_PART_ID-1)*5+2:LAST_PART_ID*5+2, 17].apply(lambda x: int(x)).tolist()


    ################################################################
    #################### Generate New Dataframe ####################
    df_dict = {
        'pid': part_id_list,
        # 'trust_tech': list_times_five(trust_tech_list),
        # 'play_games': list_times_five(play_games_list),
        # 'play_music': list_times_five(play_music_list),
        # 'order': list_times_five(order_list),
        'autonomy': auto_id_list,
        'auto_grouped': grouped_auto_list,
        'tlx_mental': tlx1_list,
        'tlx_physical': tlx2_list,
        'tlx_hurried': tlx3_list,
        'tlx_insecure': tlx4_list,
        'tlx_hard': tlx5_list,
        'tlx_successful': tlx6_list,
        'tlx_ave': tlx_ave_list,
        'mdmt_reliable': mdmt1_list,
        'mdmt_capable': mdmt2_list,
        'mdmt_predictable': mdmt3_list,
        'mdmt_skilled': mdmt4_list,
        'mdmt_counton': mdmt5_list,
        'mdmt_competent': mdmt6_list,
        'mdmt_consistent': mdmt7_list,
        'mdmt_meticulous': mdmt8_list,
        'mdmt_reliable_ave': mdmt_reliable_ave_list,
        'mdmt_capable_ave': mdmt_capable_ave_list,
        'mdmt_ave': mdmt_ave_list,
        'per_auto': per_auto_list,
        'single_trust': single_trust_list
    }
    result_df = DataFrame(df_dict)
    
    ###### remove medium autonomy rows ######
    result_df = remove_med_auto(result_df)
    
    # write new dataframe to csv file
    dest_path = MAIN_DATA_DIR + 'main_cleaned.csv'
    result_df.to_csv(dest_path, index=False)
    
    print(" Successfully written pre-processed data to csv file! \n")




##########################################################################################
def main():
    
    preprocess()
    
    


    
##########################################################################################
if __name__ == "__main__":
    
    main()
    