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

with np.load('cam1calib.npz') as X:
    mtx1, dist1, newcameramtx1, roi1 = [X[i] for i in ('mtx', 'dist', 'newcameramtx', 'roi')]

with np.load('cam2calib.npz') as X:
    mtx2, dist2, newcameramtx2, roi2 = [X[i] for i in ('mtx', 'dist', 'newcameramtx', 'roi')]

while True:
    ret, frame1 = cam1.read()
    if not ret:
        exit(1)

    ret, frame2 = cam2.read()
    if not ret:
        exit(1)

    dst1 = cv2.undistort(frame1, mtx1, dist1, None, newcameramtx1)
    gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)

    dst2 = cv2.undistort(frame2, mtx2, dist2, None, newcameramtx2)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    ret1, corners1 = cv2.findChessboardCorners(gray1, (9, 6), None)
    ret2, corners2 = cv2.findChessboardCorners(gray2, (9, 6), None)
    # If found, add object points, image points (after refining them)

    if ret1:
        corners2_1 = cv2.cornerSubPix(gray1, corners1, (11, 11), (-1, -1), criteria)
        ret1, rvecs1, tvecs1, inliers1 = cv2.solvePnPRansac(objp, corners2_1, newcameramtx1, dist1)

    if ret2:
        corners2_2 = cv2.cornerSubPix(gray2, corners2, (11, 11), (-1, -1), criteria)
        ret2, rvecs2, tvecs2, inliers2 = cv2.solvePnPRansac(objp, corners2_2, newcameramtx2, dist2)

    img1 = None
    img2 = None

    msg1 = None
    msg2 = None



    if ret1:
        imgpts1, _ = cv2.projectPoints(axis, rvecs1, tvecs1, mtx1, dist1)
        drawCircles(dst1, corners2_1, imgpts1)
        img1 = drawAxes(dst1, corners2_1, imgpts1)
        cv2.line(img1, (310, 240), (330, 240), (0, 0, 255), 2)
        cv2.line(img1, (320, 230), (320, 250), (0, 0, 255), 2)
        msg1 = np.array([np.transpose(rvecs1), np.transpose(tvecs1)])

    if ret2:
        imgpts2, _ = cv2.projectPoints(axis, rvecs2, tvecs2, mtx2, dist2)
        drawCircles(dst2, corners2_2, imgpts2)
        img2 = drawAxes(dst2, corners2_2, imgpts2)
        cv2.line(img2, (310, 240), (330, 240), (0, 0, 255), 2)
        cv2.line(img2, (320, 230), (320, 250), (0, 0, 255), 2)
        msg2 = np.array([np.transpose(rvecs2), np.transpose(tvecs2)])

    if img1 is None:
        img1 = dst1

    if img2 is None:
        img2 = dst2

    if msg1 is None:
        msg1 = np.array([[[0,0,0]],[[0,0,0]]])

    if msg2 is None:
        msg2 = np.array([[[0,0,0]],[[0,0,0]]])

    cv2.imshow('img', np.concatenate((img1, img2), axis=1))

    msg = np.array([msg1, msg2]).ravel()

    sock.sendto(np.array(msg, dtype=float).tostring(), (UDP_IP, UDP_PORT))

    k = cv2.waitKey(1)

    if k % 256 == 32:
        # ESC pressed
        print("Space hit, closing...")
        break

cv2.destroyAllWindows()
