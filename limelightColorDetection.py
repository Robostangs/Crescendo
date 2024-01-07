import cv2
import numpy as np
import time

# global variables go here:
testVar = 0

# To change a global variable inside a function,
# re-declare it with the 'global' keyword
def incrementTestVar():
    global testVar
    testVar = testVar + 1
    return testVar

# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    
    tic = time.perf_counter();
    # img = cv2.medianBlur(image,101)
  
    # BGR->HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # mask the img
    lower_bound = np.array([0, 110, 170])
    upper_bound = np.array([25, 245, 255])
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largestContour = np.array([[]])
    llpython = [0,0,0,0,0,0,0,0]

    if len(contours) > 0:
        cv2.drawContours(image, contours, -1, 255, 2)
        largestContour = max(contours, key=cv2.contourArea)
        x,y,w,h = cv2.boundingRect(largestContour)

        cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)
        llpython = [1,x,y,w,h,9,8,7]

       
    # make sure to return a contour,
    # an image to stream,
    # and optionally an array of up to 8 values for the "llpython"
    # networktables array
    toc = time.perf_counter();
    print("1 cycle ran in " + str(toc-tic) + " time");
    return largestContour, image, llpython
