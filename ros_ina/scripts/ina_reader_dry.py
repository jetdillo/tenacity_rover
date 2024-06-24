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

    sensor_d=[{"bus_name":'Bus_A',"sensor_type":'ina260',"i2c_addr":0x40},{"bus_name":'Bus_B',"sensor_type":'ina260',"i2c_addr":0x44}]
 
#Construct a sensor object
   
    ina260_l=[]
    ina260_vals=[]
    for s in range(0,len(sensor_d)):
       sensor=INA_Sensor(sensor_d[s]["sensor_type"],sensor_d[s]["i2c_addr"])
       val_d={"voltage":0.000,"current":0.000,"power":0.000} 
       ina260_l.append(sensor) 
       ina260_vals.append(val_d)  

#   ina260_sensorA = INA_Sensor('ina260', 0x40)
#   ina260_sensorB = INA_Sensor('ina260', 0x44)
#ina219_sensor = INA_Sensor('ina219', 0x41)

# Read voltage, current, and power from each sensor

    for s in range(0,len(sensor_d)):

          print("Bus:{} Voltage:{} mV".format(sensor_d[s]["bus_name"],ina260_l[s].read_voltage()*10))
          print("Bus:{} Current: {} mV".format(sensor_d[s]["bus_name"],ina260_l[s].read_current()/1000))
          print("Bus:{} Power: {} mV".format(sensor_d[s]["bus_name"],ina260_l[s].read_power()))
