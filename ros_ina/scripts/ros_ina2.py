#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import smbus2

# Assuming the INA260 is at I2C address 0x40
INA260_I2C_ADDRESS = 0x40

# INA260 Register Addresses
INA260_REG_CURRENT = 0x01
INA260_REG_BUS_VOLTAGE = 0x02

class INA260Node(object):
    def __init__(self):
        # Initialize the node
        rospy.init_node('ina260_node')
        
        # Set up the I2C interface
        self.bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1

        # Set up the publishers
        self.current_pub = rospy.Publisher('/power/ina260/current', Float32, queue_size=10)
        self.voltage_pub = rospy.Publisher('/power/ina260/bus_voltage', Float32, queue_size=10)

    def read_sensor_data(self):
        # Read current
        current_raw = self.bus.read_i2c_block_data(INA260_I2C_ADDRESS, INA260_REG_CURRENT, 2)
        current = self.convert_to_current(current_raw)
        
        # Read voltage
        voltage_raw = self.bus.read_i2c_block_data(INA260_I2C_ADDRESS, INA260_REG_BUS_VOLTAGE, 2)
        voltage = self.convert_to_voltage(voltage_raw)
        
        return current, voltage

    @staticmethod
    def convert_to_current(data):
        # Convert the raw data to current in amperes
        current = (data[0] << 8) | data[1]
        return current * 1.25 / 1000  # Example conversion factor

    @staticmethod
    def convert_to_voltage(data):
        # Convert the raw data to voltage in volts
        voltage = (data[0] << 8) | data[1]
        return voltage * 1.25 / 1000  # Example conversion factor

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            current, voltage = self.read_sensor_data()
            self.current_pub.publish(current)
            self.voltage_pub.publish(voltage)
            rate.sleep()

if __name__ == '__main__':
    node = INA260Node()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
