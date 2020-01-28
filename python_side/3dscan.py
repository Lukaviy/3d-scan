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


def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img


def pomoika(img):
    mask = cv2.inRange(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), np.array([0, 50, 50]), np.array([10, 255, 255]))
    return cv2.bitwise_and(img, img, mask=mask)


def detectCircles(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_blurred = cv2.blur(gray, (3, 3))
    cv2.imshow('q', gray_blurred)
    # TODO
    detected_circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=100)
    print(detected_circles)
    return detected_circles


def drawDetectedCircles(img, detected_circles):
    if detected_circles is not None:
        for circle in detected_circles:
            x = int(circle[0][0])
            y = int(circle[0][1])
            r = int(circle[0][2])
            cv2.rectangle(img, (x - r - 5, y - r - 5), (x + r + 5, y + r + 5), (255, 0, 0), 2)


cam = cv2.VideoCapture(0)

with np.load('cam1calib.npz') as X:
    mtx, dist, newcameramtx, roi = [X[i] for i in ('mtx', 'dist', 'newcameramtx', 'roi')]

mp.use('Qt5Agg')

p.ion()
fig = p.figure()
ax = fig.add_subplot(111, projection='3d')

p.show()

# class Thread(QThread):
#     changePixmap = pyqtSignal(QImage)
#
#     def run(self):
#         cap = cv2.VideoCapture(0)
#         while True:
#             ret, frame = cap.read()
#             if ret:
#                 rgbImage = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#                 h, w, ch = rgbImage.shape
#                 bytesPerLine = ch * w
#                 convertToQtFormat = QtGui.QImage(rgbImage.data, w, h, bytesPerLine, QtGui.QImage.Format_RGB888)
#                 p = convertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
#                 self.changePixmap.emit(p)
#
# class App(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.initUI()
#
#     @pyqtSlot(QImage)
#     def setImage(self, image):
#         self.label.setPixmap(QPixmap.fromImage(image))
#
#     def initUI(self):
#         self.setWindowTitle(self.title)
#         self.setGeometry(self.left, self.top, self.width, self.height)
#         self.resize(1800, 1200)
#         # create a label
#         self.label = QLabel(self)
#         self.label.move(280, 120)
#         self.label.resize(640, 480)
#         th = Thread(self)
#         th.changePixmap.connect(self.setImage)
#         th.start()
#
# if __name__ == '__main__':
#     app = QApplication(sys.argv)
#     ex = App()
#     sys.exit(app.exec_())

pl = None

while True:
    ret, frame = cam.read()
    if not ret:
        exit(1)

    dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cv2.imshow('undistorted', dst)

    circlesFrame = pomoika(frame)
    circles = detectCircles(circlesFrame)
    drawDetectedCircles(circlesFrame, circles)
    cv2.imshow('RedDotWindow', circlesFrame)

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
