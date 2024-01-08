import cv2
import numpy as np
import time

MIN_AREA = 500
MAX_AREA = 5000
MAX_RATIO = 0.7

# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    
    tic = time.perf_counter()

    llpython = [0, 0, 0, 0]  # Default values for tx, ty, ta, tl

    # BGR->HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # mask the img
    lower_bound = np.array([0, 130, 170])
    upper_bound = np.array([25, 245, 255])
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largestContour = np.array([[]])

    if len(contours) > 0:
        largestContour = max(contours, key=cv2.contourArea)
        x = 0
        y = 0
        w = 0
        h = 0

        contour_output = []
        # Fit an ellipse around the contour
        if len(largestContour) >= 5:
            ellipse = cv2.fitEllipse(largestContour)

            x, y, w, h = cv2.boundingRect(largestContour)

            # Filter contours based on aspect ratio and area
            rect_aspect_ratio = h / w
            contour_area = cv2.contourArea(largestContour)

            # Set aspect ratio and area 
            if rect_aspect_ratio < MAX_RATIO and MIN_AREA < contour_area < MAX_AREA:
                # Draw ellipse
                cv2.ellipse(image, ellipse, (0, 255, 0), 2)

                llpython = [x, y, w, h]
                contour_output = largestContour

    toc = time.perf_counter()
    print("1 cycle ran in " + str(toc - tic) + " time")
    
    return contour_output, image, llpython
