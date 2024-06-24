#!/usr/bin/env python2
import rospy
from std_msgs.msg import Float32
import smbus2

class INA260Node(object):
    def __init__(self):
        # Initialize the node

        self.ina_bus_name=rospy.get_namespace() 
        self.i2c_bus=rospy.get_param('ina_sensor_node/i2c_bus')
        self.device_type=rospy.get_param('ina_sensor_node/device_type')
        self.i2c_addr=int(rospy.get_param('ina_sensor_node/i2c_address'),0)
        print(self.ina_bus_name)
        print(self.i2c_addr)
        self.ina_node_str="ina260_"+str(self.i2c_addr)
        rospy.init_node(self.ina_node_str)
        
        # Set up the I2C interface
        self.bus = smbus2.SMBus(1)  # 1 indicates /dev/i2c-1

         # Define registers based on device type
        if self.device_type == 'ina260':
            self.reg_voltage = 0x02
            self.reg_current = 0x01
            self.reg_power = 0x03
        elif self.device_type == 'ina219':
            self.reg_voltage = 0x02
            self.reg_current = 0x04
            self.reg_power = 0x03
        else:
            raise ValueError('Unsupported device type')


        # Set up the publishers
        current_pub_str='/power'+self.ina_bus_name+'current'
        voltage_pub_str='/power'+self.ina_bus_name+'bus_voltage'
        power_pub_str='/power'+self.ina_bus_name+'bus_power'
        self.current_pub = rospy.Publisher(current_pub_str, Float32, queue_size=10)
        self.voltage_pub = rospy.Publisher(voltage_pub_str, Float32, queue_size=10)
        self.power_pub = rospy.Publisher(power_pub_str, Float32, queue_size=10)

     # Sensors need to be hooked up to the same voltage as the system they're monitoring.
    # 5V power when the data pins are 3.3V will give bad readings. 

    def read_voltage(self):
        raw_voltage = self._read_register(self.reg_voltage)
        #return (raw_voltage >> 3) * 1.25
        return (raw_voltage >> 3) * 10

    def read_current(self):
        raw_current = self._read_register(self.reg_current)
        if self.device_type == 'ina260':
            # INA260 uses a signed 16-bit value
            #return self._convert_twos_complement(raw_current, bits=16) * 1.25
            return self._convert_twos_complement(raw_current, bits=16) 
        elif self.device_type == 'ina219':
            # INA219 uses a signed 15-bit value
            return self._convert_twos_complement(raw_current, bits=15) * 0.01

    def read_power(self):
        raw_power = self._read_register(self.reg_power)
        if self.device_type == 'ina260':
            return (raw_power * 10)
        elif self.device_type == 'ina219':
            return raw_power * 2

    def _read_register(self, register):
        # Read 2 bytes from the given register
        data = self.bus.read_i2c_block_data(self.i2c_addr, register, 2)
        return data[0] << 8 | data[1]

    def _convert_twos_complement(self, value, bits):
        # Convert raw value to two's complement form
        if value & (1 << (bits - 1)):
            value -= 1 << bits
        return value

    def __del__(self):
        self.bus.close()

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.voltage = self.read_voltage()
            self.current = self.read_current()
            self.power   = self.read_power() 

            self.current_pub.publish(self.current)
            self.voltage_pub.publish(self.voltage)
            self.power_pub.publish(self.power)

            rate.sleep()

if __name__ == '__main__':
    node = INA260Node()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
