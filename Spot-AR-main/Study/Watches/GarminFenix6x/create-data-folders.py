import os
from datetime import date, timedelta, datetime
import configparser


config = configparser.ConfigParser()
config.read('capture_dates.ini')
session = config.sections()[0]

print("Starting Point: ", config.get(session, 'start'))
print("Ending Point: ", config.get(session, 'end'))
StartDate = datetime.strptime(config.get(session, 'start'), '%m/%d/%y %H:%M:%S')
EndDate = datetime.strptime(config.get(session, 'end'), '%m/%d/%y %H:%M:%S')

#StartDate = date(2024, 2, 6)   # start date
#EndDate = date(2024, 2, 6)   # end date


Delta = EndDate - StartDate

BaseDirectory = "data"
BaseName = 'UCF.Garmin.'


try:
    # Create target Directory
    os.mkdir(BaseDirectory)
    print("Directory " , BaseDirectory ,  " Created ") 
except FileExistsError:
    print("Directory " , BaseDirectory ,  " already exists")



watches_for_experiment = [9]   #specific watch accounts

for i in watches_for_experiment:
    DirName = BaseDirectory + "/" + BaseName + str('{0:0>3}'.format(i))

    try:
        # Create target Directory
        os.mkdir(DirName)
        print("Directory " , DirName ,  " Created ") 
    except FileExistsError:
        print("Directory " , DirName ,  " already exists")


    for i in range(Delta.days + 1):
        day = StartDate.date() + timedelta(days=i)

        SubDirName = DirName + "/" + str(day)

        try:
            # Create target Sub Directories for the specified dates
            os.mkdir(SubDirName)
            print("    Sub Directory " , SubDirName ,  " Created ") 
        except FileExistsError:
            print("    Sub Directory " , SubDirName ,  " already exists")
