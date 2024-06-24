import rospy
from sensor_msgs.msg import BatteryState
import smbus2

class INA_Sensor:
    def __init__(self, device_type, i2c_address, i2c_bus=1):
        self.i2c_address = i2c_address
        self.bus = smbus2.SMBus(i2c_bus)
        self.device_type = device_type.lower()
        
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

    def read_voltage(self):
        raw_voltage = self._read_register(self.reg_voltage)
        # Conversion for both devices is the same
        return (raw_voltage >> 3) * 1.25

    def read_current(self):
        raw_current = self._read_register(self.reg_current)
        if self.device_type == 'ina260':
            # INA260 uses a signed 16-bit value
            return self._convert_twos_complement(raw_current, bits=16) * 1.25
        elif self.device_type == 'ina219':
            # INA219 uses a signed 15-bit value
            return self._convert_twos_complement(raw_current, bits=15) * 0.01

    def read_power(self):
        raw_power = self._read_register(self.reg_power)
        if self.device_type == 'ina260':
            return raw_power * 10
        elif self.device_type == 'ina219':
            return raw_power * 2

    def _read_register(self, register):
        # Read 2 bytes from the given register
        data = self.bus.read_i2c_block_data(self.i2c_address, register, 2)
        return data[0] << 8 | data[1]

    def _convert_twos_complement(self, value, bits):
        # Convert raw value to two's complement form
        if value & (1 << (bits - 1)):
            value -= 1 << bits
        return value

    def __del__(self):
        self.bus.close()

    def publish_battery_state(self, publisher):
        battery_state = BatteryState()
        
        voltage = self.read_voltage()
        current = self.read_current()
        power = self.read_power()

        battery_state.voltage = voltage / 1000.0  # Convert mV to V
        battery_state.current = current / 1000.0  # Convert mA to A
        battery_state.power = power  # Power is already in Watts

        # Set additional fields if known
        battery_state.present = True  # Assuming the sensor is always present
        # battery_state.capacity = <Known capacity value>
        # battery_state.design_capacity = <Known design capacity value>
        # battery_state.percentage = <Known state of charge percentage>

        publisher.publish(battery_state)

    def sensor_node(self):
        rospy.init_node('ina_sensor_node')
        device_type = rospy.get_param('~device_type', 'ina260')
        i2c_address = int(rospy.get_param('~i2c_address', '0x40'), 16)
        i2c_bus = int(rospy.get_param('~i2c_bus', '1'))
        
        sensor = INA_Sensor(device_type, i2c_address, i2c_bus)
        pub = rospy.Publisher('battery_state', BatteryState, queue_size=10)
        rate = rospy.Rate(1)  # 1Hz
    
        while not rospy.is_shutdown():
            sensor.publish_battery_state(pub)
            rate.sleep()
    
if __name__ == '__main__':
    try:
        ina = INA_Sensor('ina260',0x40)
        ina.sensor_node()
    except rospy.ROSInterruptException:
        pass
