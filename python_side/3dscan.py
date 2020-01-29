import numpy as np
import cv2
import matplotlib as mp
import pylab as p
import socket


UDP_IP = "127.0.0.1"
UDP_PORT = 7325

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)


def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img


def findBrightnessPoint(img):
    best_x = 0
    best_y = 0
    best_brightness = -1
    for i in range(len(img)):
        for j in range(len(img[i])):
            if i % 2 == 0 and j % 2 == 0:
                brightness = img[i][j][0] + img[i][j][1] + img[i][j][2]
                if brightness > best_brightness:
                    best_brightness = brightness
                    best_x = j
                    best_y = i
    return best_x, best_y


cam = cv2.VideoCapture(0)

with np.load('cam1calib.npz') as X:
    mtx, dist, newcameramtx, roi = [X[i] for i in ('mtx', 'dist', 'newcameramtx', 'roi')]

mp.use('Qt5Agg')

p.ion()
fig = p.figure()
ax = fig.add_subplot(111, projection='3d')

p.show()

pl = None

while True:
    ret, frame = cam.read()
    if not ret:
        exit(1)

    dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cv2.imshow('undistorted', dst)

    point = findBrightnessPoint(frame)
    frameWithBrightnessPoint = frame
    cv2.rectangle(frameWithBrightnessPoint, (point[0] - 5, point[1] - 5), (point[0] + 5, point[1] + 5), (255, 0, 0), 1)
    cv2.imshow('brightness', frameWithBrightnessPoint)

    ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)
    # If found, add object points, image points (after refining them)
    if ret:
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        ret, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)

        if ret:
            rotMatrix, jac = cv2.Rodrigues(rvecs)

            # camVec = np.array([[0, 0, 0], [0, 0, -10]])
            # camVec = np.add(camVec, np.transpose(tvecs))
            # camVec = np.dot(camVec, rotMatrix)
            # rotx90 = R.from_euler('x', 90, degrees=True).as_matrix()
            # roty90 = R.from_euler('y', 90, degrees=True).as_matrix()
            # rotz90 = R.from_euler('z', 90, degrees=True).as_matrix()
            # camVec = np.dot(camVec, rotx90)
            # camVec = np.dot(camVec, roty90)
            # camVec = np.dot(camVec, rotz90)

            # points = np.append(np.array([[0, 0, 0]]), camVec, 0)
            # points = np.hsplit(points, 3)
            #
            imgpts, _ = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

            img = draw(dst, corners2, imgpts)

            cv2.imshow('img', img)

            msg = np.array([rotMatrix, np.transpose(tvecs)]).tobytes('C')

            sock.sendto(np.array([np.transpose(rvecs), np.transpose(tvecs)], dtype=float).tostring(), (UDP_IP, UDP_PORT))

            p.draw()

    k = cv2.waitKey(1)

    if k % 256 == 32:
        # ESC pressed
        print("Space hit, closing...")
        break

cv2.destroyAllWindows()
