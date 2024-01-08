import cv2
import numpy as np
from limelightColorDetection import runPipeline
# global variables go here:
testVar = 0


# To change a global variable inside a function,
# re-declare it with the 'global' keyword
def incrementTestVar():
    global testVar
    testVar = testVar + 1
    return testVar


# runPipeline() is called every frame by Limelight's backend.

# remote


def remoteLoad():
    path = ""

    while True:
        path = input("Path to file (e to exit): ")

        if path.lower() == "e":
            return

        original = cv2.imread(path)

        if original is None:
            print("Invalid/missing/empty path!")
        else:
            original = cv2.resize(original, (320, 240))

            img = runPipeline(original, None)[1]
            img = cv2.resize(img, (0, 0), fx=3, fy=3)
            cv2.imshow("Result of " + path, img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()


if __name__ == "__main__":
    remoteLoad()
