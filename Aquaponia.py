#Created on 27 dic. 2018

#@author: G8

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
import spidev # To communicate with SPI devices
from time import sleep  
from sys import argv, exit
#

# Imports Humedad
import RPi.GPIO as GPIO
import time


#

#Imports Gas
import RPi.GPIO as GPIO #Importamos la libreria GPIO
import time #Usaremos timer.sleep asi que hay que importar time
#
#Prueba commit
#Imports LCD
import time, sys
import smbus
import RPi.GPIO as GPIO
#

#Threads
import threading
#

#Corlysis
import requests

#ESTOS SON LOS QUE HABRIA QUE ASIGNAR A LOS VALORES GLOBALES
#         print ("Temperature : ", temperature, "C")
#         print ("Pressure : ", pressure, "hPa")
#         print ("Humidity : ", humidity, "%")


rev = GPIO.RPI_REVISION
if rev == 2 or rev ==3:
        bus = smbus.SMBus(1)
else:
        bus = smbus.SMBus(0)

# Este dispositivo tiene dos direcciones (Slave) para controlar el texto y el c$
DISPLAY_RGB_ADDR = 0x62
DISPLAY_TEXT_ADDR = 0x3e

# Set backlight to (R,G,B) (values from 0..255 for each)
def setRGB(r,g,b):
        bus.write_byte_data(DISPLAY_RGB_ADDR,0,0)
        bus.write_byte_data(DISPLAY_RGB_ADDR,1,0)
        bus.write_byte_data(DISPLAY_RGB_ADDR,0x08,0xaa)
        bus.write_byte_data(DISPLAY_RGB_ADDR,4,r)
        bus.write_byte_data(DISPLAY_RGB_ADDR,3,g)
        bus.write_byte_data(DISPLAY_RGB_ADDR,2,b)

# send command to display (no need for external use)
def textCommand(cmd):
        bus.write_byte_data(DISPLAY_TEXT_ADDR,0x80,cmd)

# set display text \n for second line (or auto wrap)
def setText(text):
        textCommand(0x01) # clear display
        time.sleep(.05)
        textCommand(0x08 | 0x04) # display on, no cursor
        textCommand(0x28) # 2 lines
        time.sleep(.05)
        count = 0
        row = 0

        for c in text:
                if c == '\n' or count == 16:
                        count = 0
                        row += 1
                        if row == 2:
                                break
                        textCommand(0xc0)
                        if c == '\n':
                                continue
                count += 1
                bus.write_byte_data(DISPLAY_TEXT_ADDR,0x40,ord(c))

#Update the display without erasing the display
def setText_norefresh(text):
        textCommand(0x02) # return home
        time.sleep(.05)
        textCommand(0x08 | 0x04) # display on, no cursor
        textCommand(0x28) # 2 lines
        time.sleep(.05)
        count = 0
        row = 0
        while len(text) < 32: #clears the rest of the screen
                text += ' '
                for c in text:
                        if c == '\n' or count == 16:
                                count = 0
                                row += 1
                                if row == 2:
                                        break
                                textCommand(0xc0)
                                if c == '\n':
                                        continue
                        count += 1
                        bus.write_byte_data(DISPLAY_TEXT_ADDR,0x40,ord(c))

# Create a custom character (from array of row patterns)
def create_char(location, pattern):
        location &= 0x07 # Make sure location is 0-7
        textCommand(0x40 | (location << 3))
        bus.write_i2c_block_data(DISPLAY_TEXT_ADDR, 0x40, pattern)

# example code
# if __name__=="__main__":
#         setText("Hello world, this is a test")
#         setRGB(0,128,64)
#         time.sleep(2)
#         for c in range(0,255):
#                 setText_norefresh("Going to sleep in {}...".format(str(c)))
#                 setRGB(c,255-c,0)
#                 time.sleep(0.1)
#                 setRGB(0,255,0)
#         setText("Bye bye, this should wrap onto next line")

#Buzzer PROBADO OK
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
#P.atmosferica PROBADO OK
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
    return temperature,pressure,humidity
    
            
#Luz PROBADO OK
def luz():
# Start SPI connection
    spi = spidev.SpiDev()
    spi.open(0,0)   

    # Read MCP3008 data
    def analogInput(channel):
        if ((channel > 7) or (channel < 0)):
            return -1
        spi.max_speed_hz = 1350000
        adc = spi.xfer2([1,(8+channel)<<4,0])
        data = ((adc[1]&3) << 8) + adc[2]
        return data
    
    try:
        #output = analogInput(int(argv[1]))
        output = analogInput(int(2)) # Reading from CH0
        print(output)
        return output
    except IndexError:
        print("please, introduce the ADC chanel in which you want to read from.")
        #exit(0)




#Gas OK
def gas():
    GPIO.setmode(GPIO.BCM) #Ajustamos la placa en modo BCM
    GPIO.setup(4, GPIO.IN) #Indicamos que el pin 4 sera de entrada
    GPIO.setup(7, GPIO.OUT) #Indicamos que el pin 7 sera de salida
    GPIO.output(7,True) #Indicamos que el pin 7 esta LOW (sin senal)
    try:
        if GPIO.input(4): #Si detectamos que el sensor se ha activado por la presencia de vapores
            print("Humo detectado") #Sacamos por pantalla HUMO DETECTADO
            GPIO.output(7, False) #Enviamos la senal de activacion al buzzer pin 7 HIGH
            time.sleep(5) #La senal (pitido) dura 5 segundos
            GPIO.output(7,True) #Cerramos la senal poniendo el pin 7 en LOW y el buzzer se calla.
        # Seguimos a la espera de otra senal por parte del sensor MQ-135
        else:
            print("No hay gases toxicos")
    except KeyboardInterrupt:
        print("El usuario ha forzado la detencion del script")
        GPIO.cleanup()
    #Con keyboardinterrupt detectamos si el usuario pulsa CONTROL + C y si es asi 
    #cerramos el script y "limpiamos" los pines GPIO con GPIO.cleanup()

#Humedad
def humedad():
    channel = 21
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(channel, GPIO.IN)
    def callback(channel):
            if GPIO.input(channel):
                    print("No hay humedad")
                    return 0
            else:
                    print("Hay suficiente humedad")
                    return 1
     
    GPIO.add_event_detect(channel, GPIO.BOTH, bouncetime=300)  # let us know when the pin goes HIGH or LOW
    GPIO.add_event_callback(channel, callback)  # assign function to GPIO PIN, Run function on change
     
    # infinite loop
    while True:
        time.sleep(1)



#LCD

def pantalla():
    global critico
    global warning
    url = 'https://corlysis.com:8086/write'
    params = {"db":"raspi8", "u":"token", "p":"d1a6c736ff5e9272d171f25ed60bf9b0"}
    while True:
        try:
            a0,a1,a2=patmosferica()
            payload = "meas_test,place=temperatura value="+a0+"\n"
            r = requests.post(url, params=params, data=payload)
            payload = "meas_test,place=presion value="+a1+"\n"
            r = requests.post(url, params=params, data=payload)
            payload = "meas_test,place=humedadAmbiente value="+a2+"\n"
            r = requests.post(url, params=params, data=payload)
        except:
            setText("Error lectura P.atmosferica")
            critico+=1
            time.sleep(1)
        #LUZ DE LA PLANTA 
        try:
            b=luz()
            payload = "meas_test,place=luz value="+b+"\n"
            r = requests.post(url, params=params, data=payload)
        except:
            setText("Error lectura de luz")
            critico+=1
            time.sleep(1)       
        #GASES EN EL AMBIENTE DE LA PLANTA
        try:
            c=gas()
            payload = "meas_test,place=gas value="+c+"\n"
            r = requests.post(url, params=params, data=payload)
        except:
            setText("Error lectura de gas")
            critico+=1
            time.sleep(1)   
        #HUMEDAD EN LAS RAICES
        try:
            d0,d1=humedad()
            payload = "meas_test,place=humedad value="+d+"\n"
            r = requests.post(url, params=params, data=payload)
        except:
            setText("Error lectura de humedad")
            critico+=1
            time.sleep(1)    





def main():
    start = time.time()
    
    alertaTemp=0 #SENSOR DE TEMPERATURA/PRESION/HUMEDAD AMBIENTE
    alertaPresion=0
    alertaHumedad=0
    
    alertaLuz=0 #SENSOR DE LUZ
    
    alertaGas=0 #SENSOR DE GAS
    
    #SENSOR TEMPERATURA RAICES/TIERRA
    alertaHumedadRaiz=0
    
    
    threads = list()
    thr = threading.Thread(target=pantalla)
    threads.append(thr)
    tr=0
    
    
    while True:
        #buzzer() SERIA ACTUADOR        
        #lcd() ACTUADOR
        
        #inicializar estos valores por si hay lectura errornea o lo que sea (pruebas)

        #Variables que seran sustituidos por valores de comparacion reales o los que tienen que ser
        
        global warning
        global critico
        
        #VALORES ADECUADOS
        
        #SENSOR DE TEMPERATURA/PRESION/HUMEDAD AMBIENTE
        p=0 #valor de temperatura
        q=0 #valor de presion
        r=0 #valor de humedad
        
        #SENSOR DE LUZ
        s=0 #valor de luz
        
        #SENSOR DE GAS (Hay que ver particulas)
        t=0 #valor de gas
        
        #SENSOR TEMPERATURA RAICES/TIERRA
        u=1 #valor de humedad
        
        
        
        end = time.time()
        diff=round(end-start)
#       print(round(end - start))
        
        if tr==0:
            thr.start()
            tr=1
        #ESTO SERIA EL CODIGO DEL HILO
#         try:
#             a=patmosferica()
#         except:
#             setText("Error lectura P.atmosferica")
#             critico+=1
#             time.sleep(1)
#         #LUZ DE LA PLANTA 
#         try:
#             b=luz()
#         except:
#             setText("Error lectura de luz")
#             critico+=1
#             time.sleep(1)       
#         #GASES EN EL AMBIENTE DE LA PLANTA
#         try:
#             c=gas()
#         except:
#             setText("Error lectura de gas")
#             critico+=1
#             time.sleep(1)   
#         #HUMEDAD EN LAS RAICES
#         try:
#             d=humedad()
#         except:
#             setText("Error lectura de humedad")
#             critico+=1
#             time.sleep(1)              
        #Habria que ver como funcionan estas condiciones
        if diff%10==0 or diff%10==1 or diff%10==9:            
            if a0>=p:
                True
            else:
                alertaTemp+=1
                setText("Temperatura baja")
                setRGB(254, 185, 58)
                warning+=1
                time.sleep(1)
            if a1>=q:
                True
            else:
                alertaPresion+=1
                setText("Presion atmosferica inestable")
                setRGB(254, 185, 58)
                warning+=1
                time.sleep(1)
            if a2>=r:
                True
            else:
                alertaHumedad+=1
                setText("Poca humedad")
                setRGB(254, 185, 58)
                warning+=1
                time.sleep(1)
        #LUZ DE LA PLANTA
            if b>=s:
                True
            else:
                alertaLuz+=1
                setText("Falta de luz")
                setRGB(254, 185, 58)
                warning+=1
                time.sleep(1)
        #GASES EN EL AMBIENTE DE LA PLANTA
            if c>=t:
                True
            else:
                alertaGas+=1
                setText("Poco oxigeno")
                setRGB(254, 185, 58)
                warning+=1
                time.sleep(1)
        #HUMEDAD EN LAS RAICES
            if d==u:
                True
            else:
                alertaHumedadRaiz+=1
                setText("Raices secas")
                setRGB(254, 185, 58)
                warning+=1
                time.sleep(1)

 

        #
        #Si un sensor no esta conectado (lo de gpio read o eso que tenga valor 0) que ensene un error critico y buzeer para saber si funciona bien todo
        #
        #SISTEMA DE VALORES
        #
        #Bien
        #Warning
        #Critico
        #
        #Cada warning que haya oscurecera el color del lcd, tres warnings indicarian un error critico, por cada warning una alerta sonora corta
        #
        
        if critico >= 0:
            setText("Error critico")
            setRGB(255, 0, 0)
        elif warning>=3:
            setRGB(199, 92, 52)
            setText("Demasiadas alertas")
            time.sleep(8)
            critico+=1
        elif warning == 2:
            setText("Dos alertas, revisa el sistema")
            setRGB(251, 150, 96)# Rosa
        elif warning == 1:
            setText("Una alerta, revisa el sistema")
            setRGB(254, 185, 58)# Amarillo
        elif warning == 0:
            setRGB(0, 255, 0)
            setText("Funcionamiento correctamente")
        
        #         setText("Hello world, this is a test")
        #         setRGB(0,128,64)
        
        

# start = time.time()
# print("hello")
# end = time.time()
# print(end - start)

warning=0
critico=0       
a0=0
a1=0
a2=0
b=0
c=0
d=0
main()    

