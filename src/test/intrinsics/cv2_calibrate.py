import numpy as np
import cv2
import glob
import os
import matplotlib.pyplot as plt
import matplotlib as mpl


def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((8 * 6, 3), np.float32)
objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2)
axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

images = glob.glob('imgs/*.png')
path = os.getcwd()
print(path)

for fname in images:
    img = cv2.imread(os.path.join(path, fname))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # print(gray.shape[::-1])
    # cv2.imshow('img',gray)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(image=gray,
                                             patternSize=(8, 6))

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,
                                    corners,
                                    (11, 11),
                                    (-1, -1),
                                    criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (8, 6), corners2, ret)
        # cv2.imshow('img', img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

# ret, camera_matrix, distortion_coef, rotation_vec, translation_vec = cv2.calibrateCamera(objpoints, imgpoints, (1920, 1200), None, None)
# newcameramtx, roi=cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coef, (1920, 1200), 1, (1920, 1200))
# print camera_matrix
# print distortion_coef
# K = [[f_x, 0,   c_x]
#      [0,   f_y, c_y]
#      [0,   0,   1]]
# d = [[k1, k2, p1, p2, k3]]

# >> Opencv calibration results
# camera_matrix = np.array(
#     [2798.91908, 0.00, 1050.47733],
#     [0.00, 2773.69518, 558.006858],
#     [0.00, 0.00, 1.00])
# distortion_coef = np.array([-0.24632269, -0.11917342, 0.00425248, 0.00420812, 1.62705478])

# >> Visp calibration results
# camera_matrix = np.array([
#     [2747.785993, 0.0, 1008.464956],
#     [0.0, 2737.721711, 598.176104],
#     [0.0, 0.0, 1.0]
# ])
# distortion_coef = np.array([-0.25515395, 0.0, 0.0, 0.0, 0.0])

# >> Zivid intrinsics record
# camera_matrix = np.array([
#     [2768.0908203125, 0.0, 949.384399414062],
#     [0.0, 2767.74096679688, 591.804870605469],
#     [0.0, 0.0, 1.0]
# ])
# distortion_coef = np.array([-0.27145066, 0.425173133, 0.000392628, -0.0009477, -0.5916040])

newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coef, (1920, 1200), 0)

for fname in images:
    img = cv2.imread(os.path.join(path, fname))
    # undistort with cv
    dst = cv2.undistort(img, camera_matrix, distortion_coef, None, newcameramtx)
    new_fname = 'buildin_intrinsics_rectified_' + fname
    cv2.imwrite(os.path.join(path, new_fname), dst)

    # Draw checker board frames
    # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    # ret, corners = cv2.findChessboardCorners(image=gray, patternSize=(8,6))
    # if ret:
    #     corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
    #     Find the rotation and translation vectors.
    #     done, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, newcameramtx, distortion_coef)
    #     project 3D points to image plane
    #     imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, newcameramtx, distortion_coef)
    #
    #     img = draw(img,corners2,imgpts)
    #     cv2.imshow('img',img)
    #     cv2.waitKey(0)
    #     cv2.destroyAllWindows()

# mean_error = 0
# for i in xrange(len(objpoints)):
#     imgpoints2, _ = cv2.projectPoints(objpoints[i], rotation_vec[i], translation_vec[i], camera_matrix, distortion_coef)
#     error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
#     mean_error += error
#
# print "total error: ", mean_error / len(objpoints)
