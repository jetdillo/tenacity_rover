import smbus2
import time

class INA_Sensor:
    def __init__(self, device_type, i2c_address, i2c_bus=1):
        self.i2c_address = i2c_address
        self.bus = smbus2.SMBus(i2c_bus)
        self.device_type = device_type.lower()
        
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

    # Sensors need to be hooked up to the same voltage as the system they're monitoring.
    # 5V power when the data pins are 3.3V will give bad readings. 
    def read_voltage(self):
        raw_voltage = self._read_register(self.reg_voltage)
        return (raw_voltage >> 3) * 1.25

    def read_current(self):
        raw_current = self._read_register(self.reg_current)
        if self.device_type == 'ina260':
            # INA260 uses a signed 16-bit value
            return self._convert_twos_complement(raw_current, bits=16) * 1.25
            #return self._convert_twos_complement(raw_current, bits=16) 
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
        data = self.bus.read_i2c_block_data(self.i2c_address, register, 2)
        return data[0] << 8 | data[1]

    def _convert_twos_complement(self, value, bits):
        # Convert raw value to two's complement form
        if value & (1 << (bits - 1)):
            value -= 1 << bits
        return value

    def __del__(self):
        self.bus.close()

if __name__ == '__main__':

# Example usage:
# Create sensor instances for both INA260 and INA219
   ina260_sensorA = INA_Sensor('ina260', 0x40)
   ina260_sensorB = INA_Sensor('ina260', 0x44)
#ina219_sensor = INA_Sensor('ina219', 0x41)

# Read voltage, current, and power from each sensor
   voltage_ina260A = ina260_sensorA.read_voltage()*10
   current_ina260A = ina260_sensorA.read_current()/1000
   power_ina260A = ina260_sensorA.read_power()
   
   voltage_ina260B = ina260_sensorB.read_voltage()*10
   current_ina260B = ina260_sensorB.read_current()/1000
   power_ina260B = ina260_sensorB.read_power()

#voltage_ina219 = ina219_sensor.read_voltage()
#current_ina219 = ina219_sensor.read_current()
#power_ina219 = ina219_sensor.read_power()
print("Bus_A Voltage: {} mV".format(voltage_ina260A))
print("Bus_A Current: {} mA".format(current_ina260A))
print("Bus_A Power: {} mV".format(power_ina260A))

print("Bus_B Voltage: {} mV".format(voltage_ina260B))
print("Bus_B Current: {} mA".format(current_ina260B))
print("Bus_B Power: {} mV".format(power_ina260B))

#print(f"INA219 Voltage: {voltage_ina219} mV")
#print(f"INA219 Current: {current_ina219} mA")
#print(f"INA219 Power: {power_ina219} mW")
