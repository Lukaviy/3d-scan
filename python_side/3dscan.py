import sys

import numpy as np
import scipy as sp
import cv2
import matplotlib as mp
import pylab as p
import PyQt5
from PyQt5 import Qt
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QLabel, QApplication, QWidget
from PyQt5.uic.properties import QtGui
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
import socket
from pickle import dumps
import glob
import time

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

def drawAxes(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (0, 0, 255), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (255, 0, 0), 5)
    return img

def drawCircles(img, corners, imgpts):
    for corner in corners:
        img = cv2.circle(img, tuple(corner.ravel()), 5, (0, 255, 0), 2)
    return img

cam1 = cv2.VideoCapture(0)
cam2 = cv2.VideoCapture(1)

# cam1.set(3, 1280)
# cam1.set(4, 960)

time.sleep(2)

cam1.set(15, -4.0)
cam1.set(cv2.CAP_PROP_AUTOFOCUS, 0)

# cam2.set(3, 1280)
# cam2.set(4, 960)

time.sleep(2)

cam2.set(15, -4.0)
cam2.set(cv2.CAP_PROP_AUTOFOCUS, 0)

time.sleep(2)

with np.load('cam1calib.npz') as X:
    mtx1, dist1, newcameramtx1, roi1 = [X[i] for i in ('mtx', 'dist', 'newcameramtx', 'roi')]

with np.load('cam2calib.npz') as X:
    mtx2, dist2, newcameramtx2, roi2 = [X[i] for i in ('mtx', 'dist', 'newcameramtx', 'roi')]

def estimateCameraPose(frame, dist, camMatrix, newCamMtx):
    undistorted = cv2.undistort(frame, camMatrix, dist, None, newCamMtx)
    gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

    if ret:
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        ret, rvec, tvec, inliers = cv2.solvePnPRansac(objp, corners2, newCamMtx, dist)

    img = None
    msg = None

    if ret:
        imgpts, _ = cv2.projectPoints(axis, rvec, tvec, newCamMtx, dist)
        drawCircles(undistorted, corners2, imgpts)
        img = drawAxes(undistorted, corners2, imgpts)
        cv2.line(img, (310, 240), (330, 240), (0, 0, 255), 2)
        cv2.line(img, (320, 230), (320, 250), (0, 0, 255), 2)
        mat, _ = cv2.Rodrigues(rvec)
        R = mat.transpose()
        pos = -R * tvec
        msg = np.append(np.array(mat).ravel(), np.transpose(tvec))

    if img is None:
        img = undistorted

    if msg is None:
        msg = np.array([0,0,0,0,0,0,0,0,0,0,0,0])

    return msg, img

def reprojectPoint(camMatrix, pointOnScreen):
    return -np.dot(np.linalg.inv(camMatrix), np.array([[pointOnScreen[0]], [pointOnScreen[1]], [1]]))

def findLightSpot(frame, dist, camMatrix, newCamMtx):
    undistorted = cv2.undistort(frame, camMatrix, dist, None, newCamMtx)
    gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

    _, maxVal, _, maxLoc = cv2.minMaxLoc(gray)

    cv2.line(undistorted, (maxLoc[0] - 10, maxLoc[1]), (maxLoc[0] + 10, maxLoc[1]), (0, 255, 0), 2)
    cv2.line(undistorted, (maxLoc[0], maxLoc[1] - 10), (maxLoc[0], maxLoc[1] + 10), (0, 255, 0), 2)

    cv2.putText(undistorted, str(maxVal), (10,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))

    pos = reprojectPoint(camMatrix, maxLoc)

    if maxVal > 50:
        return [pos[0],pos[1]], undistorted
    else:
        return [0,0], undistorted

while True:
    ret, frame1 = cam1.read()
    if not ret:
        exit(1)

    ret, frame2 = cam2.read()
    if not ret:
        exit(1)

    msg1, img1 = estimateCameraPose(frame1, dist1, mtx1, newcameramtx1)
    msg2, img2 = estimateCameraPose(frame2, dist2, mtx2, newcameramtx2)

    cv2.imshow('img', np.concatenate((img1, img2), axis=0))

    vec1 = reprojectPoint(mtx1, [320, 240])
    vec2 = reprojectPoint(mtx2, [320, 240])

    msg = np.append(msg1, np.array([vec1[0],vec1[1]]).transpose())
    msg = np.append(msg, msg2)
    msg = np.append(msg, np.array([vec2[0],vec2[1]]).transpose())

    sock.sendto(np.array(msg, dtype=float).tostring(), (UDP_IP, UDP_PORT))

    k = cv2.waitKey(1)

    if k % 256 == 32:
        # ESC pressed
        print("Space hit, closing...")
        break

while True:
    ret, frame1 = cam1.read()
    if not ret:
        exit(1)

    ret, frame2 = cam2.read()
    if not ret:
        exit(1)

    dir1, img1 = findLightSpot(frame1, dist1, mtx1, newcameramtx1)
    dir2, img2 = findLightSpot(frame2, dist2, mtx2, newcameramtx2)

    dummyMessage = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    msg = np.array(dummyMessage)
    msg = np.append(msg, dir1)
    msg = np.append(msg, dummyMessage)
    msg = np.append(msg, dir2)

    sock.sendto(np.array(msg, dtype=float).tostring(), (UDP_IP, UDP_PORT))

    cv2.imshow('img', np.concatenate((img1, img2), axis=0))

    k = cv2.waitKey(1)

    if k % 256 == 32:
        # ESC pressed
        print("Space hit, closing...")
        break

cv2.destroyAllWindows()
