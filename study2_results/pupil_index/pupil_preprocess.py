from pandas import read_csv, DataFrame
from os import getcwd

import math
import pywt
import numpy as np
from numpy import isnan


TOBII_DATA_DIR = getcwd() + "\\windows\\secondary task\\data_tobii\\"
PUPIL_RESULT_DIR = getcwd() + "\\study2_results\\pupil_index\\"

FIRST_PART_ID = 2
LAST_PART_ID = 25


##########################################################################################
def get_raw_data(part_id):
    df = read_csv(TOBII_DATA_DIR + "part" + str(part_id) + ".csv")
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
def list_times_five(old_list):
    old_len = len(old_list)
    new_list = []
    for i in range(old_len):
        for j in range(5):
            new_list.append(old_list[i])
    return new_list


################################################################
def clean_up_list(mylist: str):
    pre_list = mylist.strip('][').split(', ')
    pre_list = [float(pre_list[i]) for i in range(len(pre_list))]
    length = len(pre_list)
    return_list = []
    nans = 0
    for item in pre_list:
        if isnan(float(item)):
            nans += 1
        else:
            return_list.append(float(item))
    if len(return_list) == 0:
        return_list = [3.2 for i in range(nans)]
    mean = sum(return_list) / len(return_list)
    for i in range(nans):
        return_list.append(mean)
    return return_list, float(nans / length)


################################################################
def modmax(d):
    # compute signal modulus
    m = [0.0]*len(d)
    for i in range(len(d)):
        m[i] = math.fabs(d[i])
    # if value is larger than both neighbours, and strictly
    # larger than either, then it is a local maximum
    t = [0.0]*len(d)
    for i in range(len(d)):
        ll = m[i-1] if i >= 1 else m[i]
        oo = m[i]
        rr = m[i+1] if i < len(d)-2 else m[i]
        if (ll <= oo and oo >= rr) and (ll < oo or oo > rr):
            # compute magnitude
            t[i] = math.sqrt(d[i]**2)
        else:
            t[i] = 0.0
    return t


################################################################
def lhipa(d):
    # find max decomposition level
    w = pywt.Wavelet('sym16')
    maxlevel = pywt.dwt_max_level(len(d), filter_len=w.dec_len)
    # set high and low frequency band indices
    # hif, lof = 1, int(maxlevel/2)
    hif, lof = 1, 2
    # get detail coefficients of pupil diameter signal d
    cD_H = pywt.downcoef('d', d, 'sym16', 'per', level=hif)
    cD_L = pywt.downcoef('d', d, 'sym16', 'per', level=lof)
    # normalize by 1/sqrt(2j)
    cD_H[:] = [x / math.sqrt(2**hif) for x in cD_H]
    cD_L[:] = [x / math.sqrt(2**lof) for x in cD_L]
    # obtain the LH:HF ratio
    cD_LH = cD_L
    for i in range(len(cD_L)):
        my_index = int(((2**lof)/(2**hif))*i)
        cD_LH[i] = cD_L[i] / cD_H[my_index]
    # detect modulus maxima, see Duchowski et al. [15]
    cD_LHm = modmax(cD_LH)    # this is a list
    # threshold using universal threshold
    # where sigma is the standard deviation of the noise
    lambda_univ = np.std(cD_LHm) * math.sqrt(2.0*np.log2(len(cD_LHm)))      # numpy.float64
    cD_LHt = pywt.threshold(cD_LHm, lambda_univ, mode='less')               # numpy.ndarray
    # get signal duration (in seconds)
    # tt = d[-1].timestamp() - d[0].timestamp()
    tt = 10             # hard-code 10 seconds
    # compute LHIPA
    ctr = 0
    for i in range(len(cD_LHt)):
        if math.fabs(cD_LHt[i]) > 0:
            ctr += 1
    LHIPA = float(ctr)/tt
    return LHIPA


##########################################################################################
def ave_of_list(my_list, skip_first=0):
    sum = 0.0
    count = 0
    for i in range(skip_first, len(my_list)):
        sum += my_list[i]
        count += 1
    return sum / count


##########################################################################################
def ave_of_two_lists(list1, list2):
    ave_list = []
    for i in range(len(list1)):
        this_sum = 0
        zero_count = 2
        if list1[i] > 0:
            this_sum += list1[i]
            zero_count -= 1
        if list2[i] > 0:
            this_sum += list2[i]
            zero_count -= 1
        if zero_count == 0:
            this_ave = this_sum / 2
        else:
            this_ave = this_sum
        ave_list.append(this_ave)
    return ave_list



##########################################################################################
def preprocess():
    
    left_nan_p_list = []
    right_nan_p_list = []
    
    # separate into alpha level groups and get average errors
    auto_id_list = []
    ave_size_list = []
    
    new_left_size_list = []
    new_right_size_list = []
    new_ave_size_list = []
    
    for part_id in range(FIRST_PART_ID, LAST_PART_ID+1):
        
        # get overall dataframe from each participant's csv file
        raw = get_raw_data(part_id)
        raw = raw[['alpha_id', 'left_ave_size', 'right_ave_size', 'left_sizes', 'right_sizes']]
    
        for n in range(5):
            
            this_alpha_group = raw.iloc[n*6:n*6+6, :]
            
            this_alpha_id = (this_alpha_group['alpha_id'].tolist())[1]
            this_auto_id = round(1.0 - (this_alpha_id / 5.0), 1)
            
            this_left_list = this_alpha_group['left_ave_size'].tolist()
            this_right_list = this_alpha_group['right_ave_size'].tolist()
            del this_left_list[0]
            del this_right_list[0]
            
            left_list_of_lists = this_alpha_group['left_sizes'].tolist()
            right_list_of_lists = this_alpha_group['right_sizes'].tolist()
            del left_list_of_lists[0]
            del right_list_of_lists[0]
            
            new_left_list = []
            new_right_list = []
            new_ave_list = []
            
            for row_index in range(5):
                
                left_list = left_list_of_lists[row_index]
                right_list = right_list_of_lists[row_index]
                
                left_res_list, left_nan_p = clean_up_list(left_list)
                right_res_list, right_nan_p = clean_up_list(right_list)
                
                left_nan_p_list.append(left_nan_p)
                right_nan_p_list.append(right_nan_p)
                
                left_res = lhipa(left_res_list)
                right_res = lhipa(right_res_list)
                
                new_left_list.append(left_res)
                new_right_list.append(right_res)
                new_ave_list.append(float((left_res + right_res) / 2))
            
            # calculate average values for this autonomy level
            this_ave_list = ave_of_two_lists(this_left_list, this_right_list)
            this_ave_size = ave_of_list(this_ave_list, skip_first=1)
            # new measures
            new_left_ave = ave_of_list(new_left_list, skip_first=1)
            new_right_ave = ave_of_list(new_right_list, skip_first=1)
            new_ave_size = ave_of_list(new_ave_list, skip_first=1)
            
            # add to lists
            auto_id_list.append(this_auto_id)
            ave_size_list.append(this_ave_size)
            
            # add new measures to lists
            new_left_size_list.append(new_left_ave)
            new_right_size_list.append(new_right_ave)
            new_ave_size_list.append(new_ave_size)
    
    
    grouped_auto_list = get_grouped_auto_list(auto_id_list)
    
    # generate new dataframe
    df_dict = {
        'pid': list_times_five([p for p in range(FIRST_PART_ID, LAST_PART_ID+1)]),
        'autonomy': auto_id_list,
        'auto_grouped': grouped_auto_list,
        'pupil_diameter': ave_size_list,
        'left_index': new_left_size_list,
        'right_index': new_right_size_list,
        'pupil_index': new_ave_size_list
    }
    processed_df = DataFrame(df_dict)
    
    processed_df = remove_med_auto(processed_df)
    # write new dataframe to csv file
    dest_path = PUPIL_RESULT_DIR + "pupil_index.csv"
    processed_df.to_csv(dest_path, index=False)
    
    print(" Successfully written pre-processed data to csv file! \n")
    
    
    # calculate proportion of nan values
    ave_left_nan_p = sum(left_nan_p_list) / len(left_nan_p_list)
    ave_right_nan_p = sum(right_nan_p_list) / len(right_nan_p_list)
    
    print("left nan percentage = %.3f" % ave_left_nan_p)
    print("right nan percentage = %.3f" % ave_right_nan_p)
    
    ave_nan_p = (ave_left_nan_p + ave_right_nan_p) / 2
    print("The average nan percentage = %.4f %%" % float(ave_nan_p*100))



##########################################################################################
def main():
    
    preprocess()
    
    


    
##########################################################################################
if __name__ == "__main__":
    
    main()
    