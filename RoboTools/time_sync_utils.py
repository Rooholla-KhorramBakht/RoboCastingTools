import numpy as np 

def getSyncMap(timestamps_first, timestamps_second, threshold=0.005, add_nans=False): 
    '''
    getSyncMap function gets two list of time stamps and returns a list of synchronizing maps
    [ ...[first_index,corresponding_synchronizing_second_index]...]. if there are no indices
    in the second timestamp that is close enough to the indices in the first list (dt<threshold),
    nan will be used to indicate the situation (if add_nans is set to True).

    :param: timestamps_first:  a 1-D array of timestamps corresponding to sensor1
    :param: timestamps_second: a 1-D array of timestamps corresponding to sensor2
    :param: threshold:         How close should the stamps from the two sensors be?
    :param: add_nans:          Should you add nan for cases where no close enough stamps exist?
    :return map                A map that corresponds the index of sensor1 to sensor2
    '''
    map_list=[]
    for i in range(len(timestamps_first)):
        corresponding_second_index=np.argmin(np.abs(timestamps_second -timestamps_first[i]))
        min_dt=np.min(np.abs(timestamps_second -timestamps_first[i]))
        if min_dt<threshold:
            map_list.append((i,corresponding_second_index))
        elif add_nans:
             map_list.append((i,np.nan))

    return np.array(map_list)