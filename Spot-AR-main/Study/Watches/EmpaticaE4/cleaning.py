import pandas as pd
import numpy as np
import os
from os import listdir
from os.path import isfile, join, dirname
import tkinter as tk
from tkinter.filedialog import askopenfilename, askdirectory
tk.Tk().withdraw() # part of the import if you are not using other tkinter functions
import datetime
import zipfile


# lets get to the files
# ask user for file to open
# select the E4 folder
#fn = askdirectory(initialdir="\\", mustexist=True)
#print("user chose", fn)


# Choose zip file
filename = askopenfilename()
print(f"File selected: {filename}")
directory = os.path.dirname(filename)
print(f"Directory: {directory}")

#users = ['e4-01']
acc_raw, bvp_raw, eda_raw, hr_raw, temp_raw, tags = None, None, None, None, None, None

if zipfile.is_zipfile(filename):
    # unzip the zip files
    with zipfile.ZipFile(filename, 'r') as zip:
        zip.printdir()
        zipped_dir_path = os.path.join(directory, os.path.splitext(os.path.basename(filename))[0])
        print(f"Extracting files to: {zipped_dir_path}")
        zip.extractall(zipped_dir_path)
else:
    raise Exception("File selected was not a .zip file.")

#  first get all of the data into files and the easiest way to do that now is by keeping them separate based off of the type
zipped_files = [f for f in listdir(zipped_dir_path)]

for file in zipped_files:
    zipped_file_path = os.path.join(zipped_dir_path, file)
    print(zipped_file_path)

    if zipped_file_path.endswith('ACC.csv'):
        acc_raw = pd.read_csv(zipped_file_path, header=None)
        
    elif zipped_file_path.endswith('BVP.csv'):
        bvp_raw = pd.read_csv(zipped_file_path, header=None)

    elif zipped_file_path.endswith('HR.csv'):
        hr_raw = pd.read_csv(zipped_file_path, header=None)

    elif zipped_file_path.endswith('EDA.csv'):
        eda_raw = pd.read_csv(zipped_file_path, header=None)

    elif zipped_file_path.endswith('TEMP.csv'):
        temp_raw = pd.read_csv(zipped_file_path, header=None)
        
    # elif file.endswith('tags.csv'):
    #     tags_info = pd.read_csv(directory+'tags.csv', header=None)
    #     tags = tags_info.iloc[:,0]


#  now clean it up a bit because the data is done in a weird format where the first number is the timestamp and the second number is the sampling rate and the rest of the rows are the actual data
#acc, bvp, eda, hr, temp = {}, {}, {}, {}, {}
acc = {'start_time': acc_raw.iloc[0,0], 'sampling_freq': acc_raw.iloc[1,0], 
            'data': acc_raw.iloc[2:,:]}
acc['data'].columns = ['x','y','z']

bvp = {'start_time': bvp_raw.iloc[0,0], 'sampling_freq': bvp_raw.iloc[1,0], 
            'data': bvp_raw.iloc[2:,:]}
bvp['data'].columns = ['bvp']

eda = {'start_time': eda_raw.iloc[0,0], 'sampling_freq': eda_raw.iloc[1,0], 
            'data': eda_raw.iloc[2:,:]}
eda['data'].columns = ['eda']

hr = {'start_time': hr_raw.iloc[0,0], 'sampling_freq': hr_raw.iloc[1,0], 
           'data': hr_raw.iloc[2:,:]}
hr['data'].columns = ['bpm']

temp = {'start_time': temp_raw.iloc[0,0],  'sampling_freq': temp_raw.iloc[1,0], 
             'data': temp_raw.iloc[2:,:]}
temp['data'].columns = ['temp']

# now all the data into one object
all_data = {'ACC':acc, 'BVP':bvp, 'EDA':eda, 'HR':hr, 'TEMP':temp}

# e4_01, e4_02, e4_03 = {}, {}, {}

for i in all_data:
    sample_rate = all_data[i]['sampling_freq']
    start = all_data[i]['start_time']
    stop = start + (len(all_data[i]['data'])/sample_rate)
    time_series = np.linspace(start, stop, num=len(all_data[i]['data'])).tolist()
    data = all_data[i]['data']

    data['time_series'] = time_series

    # save as csv
    data.to_csv(os.path.join(zipped_dir_path, str(i)) + '.csv', index="time_series")
    #data.to_csv(filename+'\\'+str(i)+'.csv', index="time_series")

# for id in users:
#     dataList = [] 
#     for i in all_data:
#         sample_rate = all_data[i]['sampling_freq']
#         start = all_data[i]['start_time']
#         stop = start + (len(all_data[i]['data'])/sample_rate)
#         time_series = np.linspace(start, stop, num=len(all_data[i]['data'])).tolist()
#         data = all_data[i]['data']

#         data['time_series'] = time_series
        
#         dataList.append(data)

#         # save as csv
#     # fullData = pd.join(dataList, on="time_series")
#     fullData = pd.concat(dataList).reindex('time_series')
#     fullData.to_csv(fn + id +'NEW.csv', index=False)