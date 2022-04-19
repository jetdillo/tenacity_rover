#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# PyAX-12

# The MIT License
#
# Copyright (c) 2010,2015 Jeremie DECOCK (http://www.jdhp.org)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
A PyAX-12 demo.

Move the first Dynamixel unit to various position angles.
This snippet moves the first Dynamixel unit to 0°, then -45°, -90°, -135°,
-150° (the maximum CW angle), +150° (the maximum CCW angle), +135°, +90°, +45°
and finally goes back to 0°.
"""

from pyax12.connection import Connection
from pyax12.argparse_default import common_argument_parser

import time

def main():
    """
    A PyAX-12 demo.
    Find all the currently connected AX12s and jog them back and forth through a series of turns
    """


    # Parse options
    parser = common_argument_parser(desc=main.__doc__)
    args = parser.parse_args()

    # Connect to the serial port
    serial_connection = Connection(port=args.port,
                                   baudrate=args.baudrate,
                                   timeout=args.timeout,
                                   rpi_gpio=args.rpi)


    # Ping the dynamixel unit(s)
    ids_available = serial_connection.scan()

    print("Found %d servos with ids %s" % (len(ids_available),ids_available))

    prev_pos=[int(serial_connection.get_present_position(i,degrees=False)) for i in ids_available]
    print(prev_pos)
    #type(prev_pos[0])
    for dynamixel_id in ids_available:
       
      print("Starting position for servo %d is %d" % (dynamixel_id,serial_connection.get_present_position(dynamixel_id,degrees=True)))
     
    # Go to -45°
      serial_connection.goto(dynamixel_id, -45, speed=512, degrees=True)
      print("-45 angle for servo %d is %d" % (dynamixel_id,serial_connection.get_present_position(dynamixel_id,degrees=False)))
      time.sleep(1)    # Wait 1 second
      
    # Go to -90° (90° CW)
      serial_connection.goto(dynamixel_id, -90, speed=512, degrees=True)
      time.sleep(1)    # Wait 1 second
      print("-90 angle for servo %d is %d" % (dynamixel_id,serial_connection.get_present_position(dynamixel_id,degrees=False)))

    # Go to +90° (90° CCW)
      serial_connection.goto(dynamixel_id, 45, speed=512, degrees=True)
      time.sleep(1)    # Wait 1 second
      print("+45 angle for servo %d is %d" % (dynamixel_id,serial_connection.get_present_position(dynamixel_id,degrees=False)))
    
    # Go to +90° (90° CCW)
      serial_connection.goto(dynamixel_id, 90, speed=512, degrees=True)
      time.sleep(1)    # Wait 1 second
      print("+90 angle for servo %d is %d" % (dynamixel_id,serial_connection.get_present_position(dynamixel_id,degrees=False)))

      serial_connection.goto(dynamixel_id,340, speed=512, degrees=False)
      time.sleep(1)    # Wait 1 second
      print("Home angle for servo %d is %d" % (dynamixel_id,serial_connection.get_present_position(dynamixel_id,degrees=False)))

    # Close the serial connection
    serial_connection.close()

if __name__ == '__main__':
    main()
