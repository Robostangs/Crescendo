import cv2
import numpy as np
import os
from note_finder import runPipeline

exitCmds = ["e", "exit", "q", "quit"]
def remoteLoad():
    while True:
        path = input("File name in current directory (e to exit): ")
        if path.lower() in exitCmds:
            return
        original = cv2.imread("src\\main\\java\\frc\\robot\\Vision\\" + path)

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
