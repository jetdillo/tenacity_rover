#!/usr/bin/env python2.7

# Python libraries
import sys, serial, struct, time, os, argparse


# Import OpenCV
import cv2
import numpy as np

def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')
# Main
if __name__ == '__main__':

  # Import arguments
    parser = argparse.ArgumentParser(description='Test OpenMV Camera')
    parser.add_argument("-c","--cal", type=str2bool, nargs='?',
                        const=True, default=False,
                        help="Use calibration")
    parser.add_argument("-f","--fisheye", type=str2bool, nargs='?',
                        const=True, default=False,
                        help="Use fisheye calibration")

    args = parser.parse_args()

    port = '/dev/openmvcam'
    serial_port = serial.Serial(port, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
             xonxoff=False, rtscts=False, stopbits=serial.STOPBITS_ONE, timeout=None, dsrdtr=True)

    # Check for calibration data
    cal_path = './calibration/output/calib.npz'
    calibration = 0
    if args.cal:
        if args.fisheye:
            cal_path = './calibration/output/calib_fisheye.npz'
        if os.path.isfile(cal_path):
            cal = np.load(cal_path)
            K = cal['camera_matrix']
            D = cal['dist_coefs']
            calibration = 1

    while True:
        # Read data from the serial buffer
        serial_port.write("snap")
        serial_port.flush()
        size = struct.unpack('<L', serial_port.read(4))[0]
        buf = serial_port.read(size)
        
        # Use numpy to construct an array from the bytes
        x = np.fromstring(buf, dtype='uint8')

        # Decode the array into an image
        img = cv2.imdecode(x, cv2.IMREAD_UNCHANGED)
        
        # Undistort the image if calibration exists
        if calibration:
            if args.fisheye:
                DIM = img.shape[1::-1]
                map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
                img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            else:
                h, w = img.shape[:2]
                newcameramtx, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
                img = cv2.undistort(img, K, D, None, newcameramtx)

            

        cv2.imshow("Stream:", img)
        
        key = cv2.waitKey(20)
    
        if key == 27:
            #seial_port.close()
            cv2.destroyWindow("Stream:")
            break      

             
