# openMV_cam
[![Docker Stars](https://img.shields.io/docker/stars/wilselby/openmv_cam.svg)](https://hub.docker.com/r/wilselby/openmv_cam/)
[![Docker Pulls](https://img.shields.io/docker/pulls/wilselby/openmv_cam.svg)](https://hub.docker.com/r/wilselby/openmv_cam/)
[![GitHub](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/wilselby/openMV_cam/blob/master/LICENSE)
[![Build Status](https://travis-ci.org/wilselby/openMV_cam.svg?branch=master)](https://travis-ci.org/wilselby/openMV_cam)
[![FOSSA Status](https://app.fossa.io/api/projects/git%2Bgithub.com%2Fwilselby%2FopenMV_cam.svg?type=shield)](https://app.fossa.io/projects/git%2Bgithub.com%2Fwilselby%2FopenMV_cam?ref=badge_shield)

## Install OpenMV IDE
Full instructions here http://docs.openmv.io/openmvcam/tutorial/software_setup.html

wget http://github.com/openmv/openmv-ide/releases/download/v2.0.0/openmv-ide-linux-x86_64-2.0.0.run
chmod +x openmv-ide-linux-*.run
./openmv-ide-linux-*.run

To launch the IDE:
./openmvide

Follow the steps to update the camera firmware.

Use the IDE sample code to focus the lens

## Install main.py
Install the usb_vcp.py file as the camera's main.py file

Check the version of OpenCV
python
import cv2
cv2.__version__

The installed version of OpenCV must be greater than 3.1

Run cam_test.py to view the camera's output.
python cam_test.py

## Calibrate the camera
Print out a checkerboard to be used as the calibration target.

Acquire calibration images using the get_cal_images script
python get_cal_images

The script will save the files in the calibrate folder. It will only save images if the checkerboard wasdetect in the frame.

Once you have a set of calibration images we can run a calibration script. The script will output a series of images showing the detected checkerboard and the undistorted image. It will also output a file of calibration parameters.

If you have a fisheye lense, use the calibrate_fisheye.py file
python calibrate_fisheye './input/*.jpg'

If you have a noraml lense, use the regular calibrate.py file
python calibrate.py './input/*.jpg'

You can verify the calibration by running the cam_test.py file. It will load the calibration parameters if they are available and use them to undistort the images.

## Run the OpenMV Camera ROS node
Make sure your environment is sourced.
source /path/to/catkin_ws/devel/setup.bash

rosrun openMV_cam openmv_cam_node.py



