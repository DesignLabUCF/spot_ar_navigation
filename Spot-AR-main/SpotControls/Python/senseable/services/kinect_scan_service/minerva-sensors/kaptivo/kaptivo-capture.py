import cv2
import os
import time


# open the camera
capture = cv2.VideoCapture("https://192-168-0-106.ip.kaptivo.live/")

# create folder if it doesn't already exist
path = "kaptivo-images"

try: 
    os.mkdir(path) 
except OSError as error: 
    print(error)  

# save an intitial frame and get the time
ret, frame = capture.read()
curr_time = round(time.time()*1000)

refFrame = frame # refFrame holds the frame that will be used to compare against

imageName = path + '/' + str(curr_time) + '-kaptivo.png'

cv2.imwrite(imageName, refFrame)
# infinite loop that will be exited some way #TODO
while True:
    # get the frame
    ret, frame = capture.read()
    curr_time = round(time.time()*1000)


    # find the difference between the 2
    diff = cv2.absdiff(refFrame, frame)
    meandiff = cv2.mean(diff)

    # save image if the difference is big enough and save the new image as the reference for changing
    if meandiff > 1: # not sure what this number should be yet
        refFrame = frame # reset the comparison frame

        # change file name for new image
        imageName = path + '/' + curr_time + '-kaptivo.png'

        # save image
        cv2.imwrite(imageName, refFrame)
