import logging, dateutil, datetime
import pandas as pd
import configparser
import time

import json
import logging
import os
import sys
from getpass import getpass

import readchar
import requests
from garth.exc import GarthHTTPError

from garminconnect import (
    Garmin,
    GarminConnectConnectionError,
    GarminConnectTooManyRequestsError,
    GarminConnectAuthenticationError,
)

from tcx_to_csv import tcx_to_df


#New Garmin Login code  (eye roll inserted here.)
def init_api(email, password, tokenstore, tokenstore_base64):
    """Initialize Garmin API with your credentials."""

    try:
        # Using Oauth1 and OAuth2 token files from directory
        print(
            f"Trying to login to Garmin Connect using token data from directory '{tokenstore}'...\n"
        )

        # Using Oauth1 and Oauth2 tokens from base64 encoded string
        # print(
        #     f"Trying to login to Garmin Connect using token data from file '{tokenstore_base64}'...\n"
        # )
        # dir_path = os.path.expanduser(tokenstore_base64)
        # with open(dir_path, "r") as token_file:
        #     tokenstore = token_file.read()

        garmin = Garmin()
        garmin.login(tokenstore)

    except (FileNotFoundError, GarthHTTPError, GarminConnectAuthenticationError):
        # Session is expired. You'll need to log in again
        print(
            "Login tokens not present, login with your Garmin Connect credentials to generate them.\n"
            f"They will be stored in '{tokenstore}' for future use.\n"
        )
        try:
            # Ask for credentials if not set as environment variables
            if not email or not password:
                email, password = get_credentials()

            garmin = Garmin(email, password)
            v= garmin.login()
            print ("status of login attempt.....  ", v)
            # Save Oauth1 and Oauth2 token files to directory for next login
            garmin.garth.dump(tokenstore)
            print(
                f"Oauth tokens stored in '{tokenstore}' directory for future use. (first method)\n"
            )
            # Encode Oauth1 and Oauth2 tokens to base64 string and safe to file for next login (alternative way)
            token_base64 = garmin.garth.dumps()
            dir_path = os.path.expanduser(tokenstore_base64)
            with open(dir_path, "w") as token_file:
                token_file.write(token_base64)
            print(
                f"Oauth tokens encoded as base64 string and saved to '{dir_path}' file for future use. (second method)\n"
            )
        except (FileNotFoundError, GarthHTTPError, GarminConnectAuthenticationError, requests.exceptions.HTTPError) as err:
            logger.error(err)
            return None

    return garmin



def convert_time_simple(time):
    return (datetime.datetime.utcfromtimestamp((time)/1000)-datetime.timedelta(hours=4)).strftime("%A, %B %d, %Y %I:%M:%S")


def convert_time_garmin(time):
    return (datetime.datetime.utcfromtimestamp((time+631065600 )/1000)-datetime.timedelta(hours=0)).strftime("%A, %B %d, %Y %I:%M:%S")


def main():

    tokenstore = "tokens"
    tokenstore_base64 = tokenstore = "tokens_base64"
    
    StartDate = None
    EndDate = None
    Delta = None
    DeltaTime = datetime.timedelta(hours=4) #timeset from UTC/GMT to America/NewYork
    BaseDirectory = "data"


    config_date = configparser.ConfigParser()
    config_date.read('capture_dates.ini')
    session = config_date.sections()[0]

    StartDate = datetime.datetime.strptime(config_date.get(session, 'start'), '%m/%d/%y %H:%M:%S').date()
    EndDate = datetime.datetime.strptime(config_date.get(session, 'end'), '%m/%d/%y %H:%M:%S').date()
    Delta = EndDate - StartDate



    config = configparser.ConfigParser()
    config.read('credentials_test.ini')
    #user = config.sections()[0]

    if not os.path.exists(tokenstore):
        os.makedirs(tokenstore)
    if not os.path.exists(tokenstore_base64):
        os.makedirs(tokenstore_base64)


    for user in config.sections():
        print(config.get(user, 'username'))
        username = config.get(user, 'username')
        password = config.get(user, 'password')
        usernameshrt = str(username.split("@")[0])

        try:
            #api = Garmin(username, password)   #old
            api = init_api(username, password, tokenstore, tokenstore_base64)  #new token thing.
            print("Loggin in...")
            #v = api.login()
            #print(v)

            #WATCH METADATA
            data = api.get_devices()
            df = pd.DataFrame(data).transpose()

            filename = usernameshrt + "-DEVICE"
            df.to_csv(BaseDirectory + '/'+ usernameshrt + "/" +  filename + ".csv", na_rep="None", header=False)
            with open(BaseDirectory + '/'+ usernameshrt + "/" +  'RAW' + filename + ".txt",'w') as f: 
                  f.write(str(data))

            dataset = api.get_device_settings(data[0]["deviceId"])
            df = pd.DataFrame.from_dict(dataset,'index')
            filename = usernameshrt + "-DEVICE-SETTINGS"
            df.to_csv(BaseDirectory + '/'+ usernameshrt + "/" +  filename + ".csv", na_rep="None", header=False)
            with open(BaseDirectory + '/'+ usernameshrt + "/" +  'RAW' + filename + ".txt",'w') as f: 
                  f.write(str(dataset))

            #tdate = datetime.date.today() #today
            #tdate = datetime.date(2022, 4, 10)

            for i in range(Delta.days + 1):
                tdate = StartDate + datetime.timedelta(days=i)

                 ## Get heart rate data for today 'YYYY-MM-DD'
                data = api.get_heart_rates(tdate.isoformat())
                print(data)
                df = pd.DataFrame(data["heartRateValues"])

                if  df.empty :
                    continue   #Bail if no heart rate data.

                df['2'] = df.apply(lambda df: convert_time_simple(df[0]), axis=1)
                filename = usernameshrt + "-heart_rate-"+ str(tdate) 
                df.to_csv(BaseDirectory + '/'+ usernameshrt + "/" + str(tdate) + "/" + filename + ".csv", na_rep="None", header=False)
                with open(BaseDirectory + '/'+ usernameshrt + "/" + str(tdate) + "/" + 'RAW' + filename + ".txt",'w') as f: 
                      f.write(str(data))

                #First Pull down the General Stats
                stats = api.get_stats(tdate.isoformat())
                df = pd.DataFrame.from_dict(stats,'index')
                filename = usernameshrt + "-General.Stats-"+ str(tdate) 
                df.to_csv(BaseDirectory + '/'+ usernameshrt + "/" + str(tdate) + "/" + filename + ".csv", na_rep="None", header=False)
                with open(BaseDirectory + '/'+ usernameshrt + "/" + str(tdate) + "/" + 'RAW' + filename + ".txt",'w') as f: 
                      f.write(str(stats))


                ## Get stress data for today 'YYYY-MM-DD' and body battery.
                data = api.get_stress_data(tdate.isoformat())
                df = pd.DataFrame(data["stressValuesArray"])
                df['2'] = df.apply(lambda df: convert_time_simple(df[0]), axis=1)
                filename = usernameshrt + "-stress-"+ str(tdate)
                df.to_csv(BaseDirectory + '/'+ usernameshrt + "/" + str(tdate) + "/" + filename + ".csv", na_rep="None", header=False)
                df = pd.DataFrame(data["bodyBatteryValuesArray"])
                df['4'] = df.apply(lambda df: convert_time_simple(df[0]), axis=1)
                filename = usernameshrt + "-body.battery-"+ str(tdate)
                df.to_csv(BaseDirectory + '/'+ usernameshrt + "/" + str(tdate) + "/" + filename + ".csv", na_rep="None", header=False)
                with open(BaseDirectory + '/'+ usernameshrt + "/" + str(tdate) + "/" + 'RAW' + filename + ".txt",'w') as f: 
                      f.write(str(data))

                ## Get Steps/Motion data for today 'YYYY-MM-DD'
                data = api.get_steps_data(tdate.isoformat())
                df = pd.DataFrame(data)
                filename = usernameshrt + "-motion.steps-"+ str(tdate)
                df['5'] = df.apply(lambda df: (dateutil.parser.isoparse(df[0])-DeltaTime).isoformat(), axis=1)
                df['6'] = df.apply(lambda df: (dateutil.parser.isoparse(df[1])-DeltaTime).isoformat(), axis=1)
                df.to_csv(BaseDirectory + '/'+ usernameshrt + "/" + str(tdate) + "/" + filename + ".csv", na_rep="None", header=False)
                with open(BaseDirectory + '/'+ usernameshrt + "/" + str(tdate) + "/" + 'RAW' + filename + ".txt",'w') as f: 
                      f.write(str(data))
                if data is None:
                    print("******** Errrrrroooorrr", + str(tdate) + " for the following user " + usernameshrt)

                ## Get Respiration data for today 'YYYY-MM-DD'
                data = api.get_respiration_data(tdate.isoformat())
                df = pd.DataFrame(data["respirationValuesArray"])
                df['2'] = df.apply(lambda df: convert_time_simple(df[0]), axis=1)
                filename = usernameshrt + "-respiration-"+ str(tdate) 
                df.to_csv(BaseDirectory + '/'+ usernameshrt + "/" + str(tdate) + "/" + filename + ".csv", na_rep="None", header=False)
                with open(BaseDirectory + '/'+ usernameshrt + "/" + str(tdate) + "/" + 'RAW' + filename + ".txt",'w') as f: 
                      f.write(str(data))

                #Other Stats
                data = api.get_rhr_day(tdate.isoformat())
                filename = usernameshrt + "-RHR-"+ str(tdate) 
                with open(BaseDirectory + '/'+ usernameshrt + "/" + str(tdate) + "/" + 'RAW' + filename + ".txt",'w') as f: 
                      f.write(str(data))



                data = api.get_activities_by_date(tdate.isoformat(),tdate.isoformat())  #need to switch for activity for data eventually.
                session_id = 1
                for d in data:
                    bytz=api.download_activity(d["activityId"])
                    csv = str(bytz)[2:-1]
                    csv = csv.replace('\\t', ',').replace('\\n', '\n')
                    filename = BaseDirectory + '/'+ usernameshrt + "/" + str(tdate) + "/" + usernameshrt + "-datalog-activity-"+ str(tdate) + "-SESSION-" +str(session_id)+ ".tcx"
                    print(csv, file=open(filename, 'w'))
    
                    # Garminconnect API returning empty values in CSV format. Get around this by converting the properly pulled TCX (XML) file manually.
                    '''
                    bytz=api.download_activity(d["activityId"], Garmin.ActivityDownloadFormat.CSV)
                    csv = str(bytz)[2:-1]
                    csv = csv.replace('\\t', ',').replace('\\n', '\n')
                    filename = BaseDirectory + '/'+ usernameshrt + "/" + str(tdate) + "/" + usernameshrt + "-datalog-activity-"+ str(tdate) + "-SESSION-" +str(session_id)+ ".csv"
                    print(csv, file=open(filename, 'w'))
                    '''
                    filename_input = BaseDirectory + '/'+ usernameshrt + "/" + str(tdate) + "/" + usernameshrt + "-datalog-activity-"+ str(tdate) + "-SESSION-" +str(session_id)+ ".tcx"
                    tcx_to_df(filename_input).to_csv(filename_input.replace(".tcx", "_CONV.csv"), index=False)
    
                    bytz=api.download_activity(d["activityId"], Garmin.ActivityDownloadFormat.ORIGINAL)
                    csv = str(bytz)[2:-1]
                    csv = csv.replace('\\t', ',').replace('\\n', '\n')
                    filename = BaseDirectory + '/'+ usernameshrt + "/" + str(tdate) + "/" + usernameshrt + "-datalog-activity-"+ str(tdate) + "-SESSION-" +str(session_id)+ ".fit"
                    print(csv, file=open(filename, 'w'))

                    session_id = session_id + 1


        except (
                GarminConnectConnectionError,
                GarminConnectAuthenticationError,
                GarminConnectTooManyRequestsError,
            ) as err:
            print("Error occurred during Garmin Connect communication: %s", err)

        time.sleep(10)  #its gets testy when you try them all too fast.  (Take this warning seriously or you get timeout issues.)


if __name__ == "__main__":
    main()