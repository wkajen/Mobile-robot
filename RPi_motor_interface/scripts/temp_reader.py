import smbus, time

class AHT10:
    CONFIG = [0x08, 0x00]
    MEASURE = [0x33, 0x00]

    # init - class constructor (obviously)
    # bus - your I2C bus. You can watch it with "ls /dev | grep i2c-" command. If no output, enable I2C in raspi-config
    # addr - AHT10 I2C address. Can be switched by change resistor position on the bottom of your board. Default is 0x38
    def __init__(self, bus, addr=0x38):
        self.bus = smbus.SMBus(bus)
        self.addr = addr
        # self.bus.write_i2c_block_data(self.addr, 0xE1, self.CONFIG) this line causes crash 
        time.sleep(0.2) #Wait for AHT to do config (0.2ms from datasheet)

    # getData - gets temperature and humidity
    # returns tuple of collected data. getData[0] is Temp, getData[1] is humidity
    def getData(self):
        byte = self.bus.read_byte(self.addr)
        self.bus.write_i2c_block_data(self.addr, 0xAC, self.MEASURE)
        time.sleep(0.5)
        data = self.bus.read_i2c_block_data(self.addr, 0x00)
        temp_raw = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
        temp = ((temp_raw*200) / 1048576) - 50
        hum_raw = ((data[1] << 12) | (data[2] << 4) | (data[3] >> 4))
        hum = (hum_raw / 1048576.0) * 100.0
        # hum = (hum_raw / 1588576.0) * 100.0
        offset = 30.0
        if hum > offset:
            hum = hum - offset

        return (temp, hum)

if __name__ =="__main__":
    m = AHT10(1)
    data = m.getData()
    print("Temperature is {} Celsius\nHumidity is {}%".format(data[0], data[1]))