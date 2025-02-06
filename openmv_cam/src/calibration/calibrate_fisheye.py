# https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0
# https://rdmilligan.wordpress.com/2015/06/28/opencv-camera-calibration-and-pose-estimation-using-python/
# https://docs.opencv.org/3.1.0/dc/dbb/tutorial_py_calibration.html

import cv2
assert cv2.__version__[0] == '3', 'The fisheye module requires opencv version >= 3.0.0'

import numpy as np

import os
import glob

import getopt

def splitfn(fn):
    path, fn = os.path.split(fn)
    name, ext = os.path.splitext(fn)
    return path, name, ext

CHECKERBOARD = (9,6)
subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW

objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
_img_shape = None

objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

debug_dir = './output/'
images = glob.glob('./input/*.jpg')

if debug_dir and not os.path.isdir(debug_dir):
    os.mkdir(debug_dir)

for fname in images:
    img = cv2.imread(fname)
    if _img_shape == None:
        _img_shape = img.shape[:2]
    else:
        assert _img_shape == img.shape[:2], "All images must share the same size."
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
        imgpoints.append(corners)

        # draw output
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        cv2.drawChessboardCorners(vis, CHECKERBOARD, corners, ret)
        _path, name, _ext = splitfn(fname)
        outfile = os.path.join(debug_dir, name + '_chess.png')
        cv2.imwrite(outfile, vis)

N_OK = len(objpoints)
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

rms, _, _, _, _ = \
    cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        gray.shape[::-1],
        K,
        D,
        rvecs,
        tvecs,
        calibration_flags,
        (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    )

# Save the calibration values
np.savez('./output/calib_fisheye.npz', camera_matrix=K, dist_coefs=D, rvecs=rvecs, tvecs=tvecs)
    
print("Found " + str(N_OK) + " valid images for calibration")
print("DIM=" + str(_img_shape[::-1]))
print("\nRMS: " + str(rms))
print("K=np.array(" + str(K.tolist()) + ")") # camera_matrix
print("D=np.array(" + str(D.tolist()) + ")") # dist_coefs

# calculate the camera calibration error
error = 0
for i in range(len(objpoints)):
    imgPoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, D)
    error += cv2.norm(imgpoints[i], imgPoints2, cv2.NORM_L2) / len(imgpoints)

print("\nTotal error: ", error / len(objpoints))


# undistort the image with the calibration
print('')
DIM = gray.shape[::-1]
balance = 0.5 # 0 to 1
dim1 = img.shape[:2][::1] # dim1 is the dimension of the input image to un-distort
dim2 = None
dim3 = None
for fname in images if debug_dir else []:
    path, name, ext = splitfn(fname)
    img_found = os.path.join(debug_dir, name + '_chess.png')
    outfile = os.path.join(debug_dir, name + '_undistorted.png')

    img = cv2.imread(img_found)
    if img is None:
        continue
    h, w = img.shape[:2]

    """
    dim2 = dim1
    dim3 = dim1
    scaled_K = K * dim1[0] / DIM[0] # the values of K is to scale with image dimension
    scaled_K[2][2] = 1.0 # Except that K[2][2] is always 1.0
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
    """

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    

    """
    nk = K.copy()
    nk[0,0]=K[0,0]/2
    nk[1,1]=K[1,1]/2

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), nk, (800,600), cv2.CV_16SC2)  # Pass k in 1st parameter, nk in 4th parameter
    undistorted_img = cv2.remap( img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    """

    """
    nk, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1.5, (w, h))
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), nk, (800,600), cv2.CV_16SC2)  # Pass k in 1st parameter, nk in 4th parameter
    undistorted_img = cv2.remap( img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    """    

    """
    scaled_K = K.copy()
    scaled_K[0][0] = K[0][0] / 2
    scaled_K[1][1] = K[1][1] / 2
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), scaled_K, dim3, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    """
    """
    h, w = img.shape[:2]
    k_new, roi = cv2.getOptimalNewCameraMatrix(K, D, (w,h), 1, (w,h))
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, dim1, np.eye(3), balance)
    Knew = K.copy()
    Knew[(0,1),(0,1)] = 0.4 * Knew[(0,1),(0,1)]
    undistorted_img = cv2.fisheye.undistortImage(img, K, D, Knew)
    print(K)
    """

    print('Undistorted image written to: %s' % outfile)
    cv2.imwrite(outfile, undistorted_img)
