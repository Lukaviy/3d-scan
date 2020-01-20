import numpy as np
import cv2
import glob
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

cam = cv2.VideoCapture(0)

frame = None
while True:
    ret, frame = cam.read()
    if not ret:
        exit(1)
    cv2.imshow("test", frame)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)
    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv2.drawChessboardCorners(frame, (9, 6), corners2, ret)
        cv2.imshow('img', frame)

    k = cv2.waitKey(500)

    if k%256 == 32:
        # ESC pressed
        print("Space hit, closing...")
        break

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (640, 480), None, None)
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 1, (640, 480))

while True:
    ret, frame = cam.read()
    if not ret:
        exit(1)

    dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)

    cv2.imshow("undistorted", dst)
    cv2.imshow("raw", frame)

    k = cv2.waitKey(1)

    if k % 256 == 32:
        # ESC pressed
        print("Space hit, closing...")
        break

cv2.destroyAllWindows()
