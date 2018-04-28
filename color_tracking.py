# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2

cameraMatrix = np.array([[1.1646262958385487e+03, 0, 3.1950000000000000e+02],
                [0,1.1646262958385487e+03, 2.3950000000000000e+02],
                [0, 0, 1]])

distCoeffs = np.array([9.7550341295192877e-01,-8.7766448487274591e+00,0, 0,
    -1.4075700223244935e+01])

class cameraTracking:
    def __init__(self, cameraNum): 
        # define the lower and upper boundaries of the "green"
        # ball in the HSV color space, then initialize the
        # list of tracked points
        self.redLower = (89, 128, 60)
        self.redUpper = (126, 255, 173)

        self.pts = deque(maxlen=64)
 
        self.camera = cv2.VideoCapture(cameraNum)
 
    def getAngle(self):
        # grab the current frame
        (grabbed, frame) = self.camera.read()
 
        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=1000)
        # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
        # construct a mask for the color "red", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, self.redLower, self.redUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
 
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


            #actualPoints = cv2.undistortPoints(np.array([center[0],center[1]]), cameraMatrix, distCoeffs)
            #ray = np.matmul(actualPoints,cameraMatrix)
            #print(ray)

            #if center[0] < 250:
            #    print("left")
            #elif center[0] > 350:
            #    print("right")
            #else:
            #    print("center")
 
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
 
        # update the points queue
        self.pts.appendleft(center)

        # show the frame to our screen
        cv2.imshow("Frame", frame)
        
        print(center)
        if center != None:
            return 0.0234*center[0] - 12.4236

    def tearDown(self):
        self.camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    trackOne = cameraTracking(1)
    while (1):
        trackOne.getAngle()
        #print(trackOne.getAngle())
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

