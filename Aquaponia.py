'''
Created on 27 dic. 2018

@author: Xabier
'''
#Imports buzzer
from gpiozero import Buzzer
from time import sleep
#
#Imports P.atmosferica
import smbus
import time
from ctypes import c_short
from ctypes import c_byte
from ctypes import c_ubyte
#
#Imports Luz
import math
import sys
import time
from grove.adc import ADC
#

# Imports Humedad
import sys
import Adafruit_DHT
from RPLCD import CharLCD

lcd = CharLCD(cols=16, rows=2, pin_rs=37, pin_e=35, pins_data=[33, 31, 29, 23])
#

#Imports Gas
import time
import math
from MCP3008 import MCP3008
#


#ESTOS SON LOS QUE HABRIA QUE ASIGNAR A LOS VALORES GLOBALES
#         print ("Temperature : ", temperature, "C")
#         print ("Pressure : ", pressure, "hPa")
#         print ("Humidity : ", humidity, "%")



#Buzzer
def buzzer():

    buzzer = Buzzer(16)
    cont=0
    while cont<6:
        buzzer.on()
        sleep(0.1)
        buzzer.off()
        sleep(0.1)
        cont+=1

    True
#P.atmosferica
def patmosferica(): 
    DEVICE = 0x77 # Default device I2C address
    
    
    bus = smbus.SMBus(1) # Rev 2 Pi, Pi 2 & Pi 3 uses bus 1; Rev 1 Pi uses bus 0
    
    def getShort(data, index):
        # return two bytes from data as a signed 16-bit value
        return c_short((data[index+1] << 8) + data[index]).value
    
    def getUShort(data, index):
        # return two bytes from data as an unsigned 16-bit value
        return (data[index+1] << 8) + data[index]
    
    def getChar(data,index):
        # return one byte from data as a signed char
        result = data[index]
        if result > 127:
            result -= 256
        return result
    
    def getUChar(data,index):
        # return one byte from data as an unsigned char
        result =  data[index] & 0xFF
        return result
    
    def readBME280ID(addr=DEVICE):
        # Chip ID Register Address
        REG_ID     = 0xD0
        (chip_id, chip_version) = bus.read_i2c_block_data(addr, REG_ID, 2)
        return (chip_id, chip_version)
    
    def readBME280All(addr=DEVICE):
        # Register Addresses
        REG_DATA = 0xF7
        REG_CONTROL = 0xF4
        REG_CONFIG  = 0xF5

        REG_CONTROL_HUM = 0xF2
        REG_HUM_MSB = 0xFD
        REG_HUM_LSB = 0xFE

        # Oversample setting - page 27
        OVERSAMPLE_TEMP = 2
        OVERSAMPLE_PRES = 2
        MODE = 1

        # Oversample setting for humidity register - page 26
        OVERSAMPLE_HUM = 2
        bus.write_byte_data(addr, REG_CONTROL_HUM, OVERSAMPLE_HUM)

        control = OVERSAMPLE_TEMP<<5 | OVERSAMPLE_PRES<<2 | MODE
        bus.write_byte_data(addr, REG_CONTROL, control)

        # Read blocks of calibration data from EEPROM
        # See Page 22 data sheet
        cal1 = bus.read_i2c_block_data(addr, 0x88, 24)
        cal2 = bus.read_i2c_block_data(addr, 0xA1, 1)
        cal3 = bus.read_i2c_block_data(addr, 0xE1, 7)

        # Convert byte data to word values
        dig_T1 = getUShort(cal1, 0)
        dig_T2 = getShort(cal1, 2)
        dig_T3 = getShort(cal1, 4)

        dig_P1 = getUShort(cal1, 6)
        dig_P2 = getShort(cal1, 8)
        dig_P3 = getShort(cal1, 10)
        dig_P4 = getShort(cal1, 12)
        dig_P5 = getShort(cal1, 14)
        dig_P6 = getShort(cal1, 16)
        dig_P7 = getShort(cal1, 18)
        dig_P8 = getShort(cal1, 20)
        dig_P9 = getShort(cal1, 22)

        dig_H1 = getUChar(cal2, 0)
        dig_H2 = getShort(cal3, 0)
        dig_H3 = getUChar(cal3, 2)

        dig_H4 = getChar(cal3, 3)
        dig_H4 = (dig_H4 << 24) >> 20
        dig_H4 = dig_H4 | (getChar(cal3, 4) & 0x0F)

        dig_H5 = getChar(cal3, 5)
        dig_H5 = (dig_H5 << 24) >> 20
        dig_H5 = dig_H5 | (getUChar(cal3, 4) >> 4 & 0x0F)

        dig_H6 = getChar(cal3, 6)

        # Wait in ms (Datasheet Appendix B: Measurement time and current calculation)
        wait_time = 1.25 + (2.3 * OVERSAMPLE_TEMP) + ((2.3 * OVERSAMPLE_PRES) + 0.575) + ((2.3 * OVERSAMPLE_HUM)+0.575)
        time.sleep(wait_time/1000)  # Wait the required time  

        # Read temperature/pressure/humidity
        data = bus.read_i2c_block_data(addr, REG_DATA, 8)
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        hum_raw = (data[6] << 8) | data[7]

        #Refine temperature
        var1 = ((((temp_raw>>3)-(dig_T1<<1)))*(dig_T2)) >> 11
        var2 = (((((temp_raw>>4) - (dig_T1)) * ((temp_raw>>4) - (dig_T1))) >> 12) * (dig_T3)) >> 14
        t_fine = var1+var2
        temperature = float(((t_fine * 5) + 128) >> 8);

        # Refine pressure and adjust for temperature
        var1 = t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * dig_P6 / 32768.0
        var2 = var2 + var1 * dig_P5 * 2.0
        var2 = var2 / 4.0 + dig_P4 * 65536.0
        var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * dig_P1
        if var1 == 0:
            pressure=0
        else:
            pressure = 1048576.0 - pres_raw
            pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
            var1 = dig_P9 * pressure * pressure / 2147483648.0
            var2 = pressure * dig_P8 / 32768.0
            pressure = pressure + (var1 + var2 + dig_P7) / 16.0

        # Refine humidity
        humidity = t_fine - 76800.0
        humidity = (hum_raw - (dig_H4 * 64.0 + dig_H5 / 16384.0 * humidity)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * humidity * (1.0 + dig_H3 / 67108864.0 * humidity)))
        humidity = humidity * (1.0 - dig_H1 * humidity / 524288.0)
        if humidity > 100:
            humidity = 100
        elif humidity < 0:
            humidity = 0

        return temperature/100.0,pressure/100.0,humidity

    def main():
        #Extraer los valores para asignarlos a los valores globales 
        (chip_id, chip_version) = readBME280ID()
        print ("Chip ID     :", chip_id)
        print ("Version     :", chip_version)
    
        temperature,pressure,humidity = readBME280All()
        #ESTO
        print ("Temperature : ", temperature, "C")
        print ("Pressure : ", pressure, "hPa")
        print ("Humidity : ", humidity, "%")
        #Return los 3 valores en un array y luego procesarlos
        if __name__=="__main__":
            main()
    
            
#Luz
def luz():
    class GroveLightSensor:

        def __init__(self, channel):
            self.channel = channel
            self.adc = ADC()

            @property
            def light(self):
                value = self.adc.read(self.channel)
                return value

    Grove = GroveLightSensor


    def main():
        if len(sys.argv) < 2:
            print('Usage: {} adc_channel'.format(sys.argv[0]))
            sys.exit(1)

        sensor = GroveLightSensor(int(sys.argv[1]))

        print('Detecting light...')
        while True:
            print('Light value: {0}'.format(sensor.light))
            time.sleep(1)

    if __name__ == '__main__':
        main()



#Gas
def gas():
    ''' Hemen dago kalibratzeko kodea :D
    Enough of the theory – we want to use the sensor now. For this purpose you can use the code I have customized, which is located in a GitHub repository. Also included is a class for reading the MCP3008. First we clone the directory:
git clone https://github.com/tutRPi/Raspberry-Pi-Gas-Sensor-MQ
    Then we change to the directory and run the existing Python test file.
cd Raspberry-Pi-Gas-Sensor-MQ
sudo python example.py
    The calibration is started automatically during initialization. It is important that the sensor is in good / fresh air as smoke / other gases would falsify the calibration.
    '''
    ######################### Hardware Related Macros #########################
    MQ_PIN                       = 0        # define which analog input channel you are going to use (MCP3008)
    RL_VALUE                     = 5        # define the load resistance on the board, in kilo ohms
    RO_CLEAN_AIR_FACTOR          = 9.83     # RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                            # which is derived from the chart in datasheet
 
    ######################### Software Related Macros #########################
    CALIBARAION_SAMPLE_TIMES     = 50       # define how many samples you are going to take in the calibration phase
    CALIBRATION_SAMPLE_INTERVAL  = 500      # define the time interval(in milisecond) between each samples in the
                                            # cablibration phase
    READ_SAMPLE_INTERVAL         = 50       # define the time interval(in milisecond) between each samples in
    READ_SAMPLE_TIMES            = 5        # define how many samples you are going to take in normal operation 
                                            # normal operation
 
    ######################### Application Related Macros ######################
    GAS_LPG                      = 0
    GAS_CO                       = 1
    GAS_SMOKE                    = 2

    def __init__(self, Ro=10, analogPin=0):
        self.Ro = Ro
        self.MQ_PIN = analogPin
        self.adc = MCP3008()
        
        self.LPGCurve = [2.3,0.21,-0.47]    # two points are taken from the curve. 
                                            # with these two points, a line is formed which is "approximately equivalent"
                                            # to the original curve. 
                                            # data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
        self.COCurve = [2.3,0.72,-0.34]     # two points are taken from the curve. 
                                            # with these two points, a line is formed which is "approximately equivalent" 
                                            # to the original curve.
                                            # data format:[ x, y, slope]; point1: (lg200, 0.72), point2: (lg10000,  0.15)
        self.SmokeCurve =[2.3,0.53,-0.44]   # two points are taken from the curve. 
                                            # with these two points, a line is formed which is "approximately equivalent" 
                                            # to the original curve.
                                            # data format:[ x, y, slope]; point1: (lg200, 0.53), point2: (lg10000,  -0.22)  
                
        print("Calibrating...")
        self.Ro = self.MQCalibration(self.MQ_PIN)
        print("Calibration is done...\n")
        print("Ro=%f kohm" % self.Ro)
    
    
    def MQPercentage(self):
        val = {}
        read = self.MQRead(self.MQ_PIN)
        val["GAS_LPG"]  = self.MQGetGasPercentage(read/self.Ro, self.GAS_LPG)
        val["CO"]       = self.MQGetGasPercentage(read/self.Ro, self.GAS_CO)
        val["SMOKE"]    = self.MQGetGasPercentage(read/self.Ro, self.GAS_SMOKE)
        return val
        
    ######################### MQResistanceCalculation #########################
    # Input:   raw_adc - raw value read from adc, which represents the voltage
    # Output:  the calculated sensor resistance
    # Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
    #          across the load resistor and its resistance, the resistance of the sensor
    #          could be derived.
    ############################################################################ 
    def MQResistanceCalculation(self, raw_adc):
        return float(self.RL_VALUE*(1023.0-raw_adc)/float(raw_adc));
     
     
    ######################### MQCalibration ####################################
    # Input:   mq_pin - analog channel
    # Output:  Ro of the sensor
    # Remarks: This function assumes that the sensor is in clean air. It use  
    #          MQResistanceCalculation to calculates the sensor resistance in clean air 
    #          and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
    #          10, which differs slightly between different sensors.
    ############################################################################ 
    def MQCalibration(self, mq_pin):
        val = 0.0
        for i in range(self.CALIBARAION_SAMPLE_TIMES):          # take multiple samples
            val += self.MQResistanceCalculation(self.adc.read(mq_pin))
            time.sleep(self.CALIBRATION_SAMPLE_INTERVAL/1000.0)
            
        val = val/self.CALIBARAION_SAMPLE_TIMES                 # calculate the average value

        val = val/self.RO_CLEAN_AIR_FACTOR                      # divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                                # according to the chart in the datasheet 

        return val;
      
      
    #########################  MQRead ##########################################
    # Input:   mq_pin - analog channel
    # Output:  Rs of the sensor
    # Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
    #          The Rs changes as the sensor is in the different consentration of the target
    #          gas. The sample times and the time interval between samples could be configured
    #          by changing the definition of the macros.
    ############################################################################ 
    def MQRead(self, mq_pin):
        rs = 0.0

        for i in range(self.READ_SAMPLE_TIMES):
            rs += self.MQResistanceCalculation(self.adc.read(mq_pin))
            time.sleep(self.READ_SAMPLE_INTERVAL/1000.0)

        rs = rs/self.READ_SAMPLE_TIMES

        return rs
     
    #########################  MQGetGasPercentage ##############################
    # Input:   rs_ro_ratio - Rs divided by Ro
    #          gas_id      - target gas type
    # Output:  ppm of the target gas
    # Remarks: This function passes different curves to the MQGetPercentage function which 
    #          calculates the ppm (parts per million) of the target gas.
    ############################################################################ 
    def MQGetGasPercentage(self, rs_ro_ratio, gas_id):
        if ( gas_id == self.GAS_LPG ):
            return self.MQGetPercentage(rs_ro_ratio, self.LPGCurve)
        elif ( gas_id == self.GAS_CO ):
            return self.MQGetPercentage(rs_ro_ratio, self.COCurve)
        elif ( gas_id == self.GAS_SMOKE ):
            return self.MQGetPercentage(rs_ro_ratio, self.SmokeCurve)
        return 0
     
    #########################  MQGetPercentage #################################
    # Input:   rs_ro_ratio - Rs divided by Ro
    #          pcurve      - pointer to the curve of the target gas
    # Output:  ppm of the target gas
    # Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
    #          of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
    #          logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
    #          value.
    ############################################################################ 
    def MQGetPercentage(self, rs_ro_ratio, pcurve):
        return (math.pow(10,( ((math.log(rs_ro_ratio)-pcurve[1])/ pcurve[2]) + pcurve[0])))
        
#Humedad
def humedad():
    while True:
    humidity, temperature = Adafruit_DHT.read_retry(11, 4)

    lcd.cursor_pos = (0, 0)
    lcd.write_string("Temp: %d C" % temperature)
    lcd.cursor_pos = (1, 0)
    lcd.write_string("Humidity: %d %%" % humidity)
#LCD
def lcd():
    True





def main():
    while True:
        #buzzer() SERIA ACTUADOR        
        #lcd() ACTUADOR
        a=[]
        a=patmosferica()
        
        luz()
        
        gas()
        
        humedad()

        