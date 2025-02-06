#!/usr/bin/env python2.7

# Python libraries
import sys, serial, struct, time

# Import OpenCV
import cv2
import numpy as np
from datetime import datetime

port = '/dev/openmvcam'
serial_port = serial.Serial(port, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
         xonxoff=False, rtscts=False, stopbits=serial.STOPBITS_ONE, timeout=None, dsrdtr=True)

pattern_size = (9, 6)

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

    cv2.imshow("Stream:", img)
    key = cv2.waitKey(20)
    
    # save image to file, if pattern found
    ret, corners = cv2.findChessboardCorners(cv2.cvtColor(img,cv2.COLOR_BGR2GRAY), pattern_size)

    if ret == True:
        filename = datetime.now().strftime('%Y%m%d_%Hh%Mm%Ss%f') + '.jpg'
        cv2.imwrite("./input/" + filename, img)
        print("Saved {} \n").format(filename)
        cv2.drawChessboardCorners(img, pattern_size, corners, ret)
        cv2.imshow("Debug:", img)

    time.sleep(2)

    if key == 27:
        #seial_port.close()
        cv2.destroyWindow("Stream:")  
	break      

         
