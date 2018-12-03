# The MIT License (MIT)
# Copyright (c) 2018 Mike Teachman
# https://opensource.org/licenses/MIT

#
# Street Sense Project:  Air and Noise pollution sensor unit
# https://hackaday.io/project/162059-street-sense
#

# TODO
# list all module dependencies and versions
# -- include bug fix pull request for urtc module

import machine
import uos
import utime
import uasyncio as asyncio
import asyn
import time
import math
import ssd1306
import pms5003
import urtc
import gc

#    SPI Devices
#    - micro SD Card
# 
#    SPI Connections
#    Pin   Function
#    4     Tx
#    18    Rx
#    19    xx
#    23    xx

#    UART Devices
#    - PMS5003 Particulate Sensor
#
#    UART Connections
#    Pin   Function
#    32    Tx
#    33    Rx
#

#    I2C Devices
#    - SSD1306 OLED Display
#    - DS3231 Real Time Clock
#    - TODO external ADC to read Ozone and NO2 sensors
#    
#    I2C Connections
#    Pin   Function
#    21    SDA
#    22    SCL

LOGGING_INTERVAL_IN_SECS = 180.0

# TODO combine ozone, no2 sensors into one class... after I2C ADC integration
class OzoneSensor():
    def __init__(self, barrier_read, barrier_data_ready):
        self.barrier_read = barrier_read
        self.barrier_data_ready = barrier_data_ready
        loop = asyncio.get_event_loop()
        loop.create_task(self.run()) 
        
    async def run(self):
        while True:
            print('ozone...wait on barrier_read')
            await self.barrier_read
            print('ozone...read sensor')
            # TODO implement read of Spec Sensor Ozone sensor
            print('ozone...wait on barrier_data_ready')
            await self.barrier_data_ready
        
class NO2Sensor():
    def __init__(self, barrier_read, barrier_data_ready):
        self.barrier_read = barrier_read
        self.barrier_data_ready = barrier_data_ready
        loop = asyncio.get_event_loop()
        loop.create_task(self.run()) 
        
    async def run(self):
        while True:
            print('NO2...wait on barrier_read')
            await self.barrier_read
            print('NO2...read sensor')
            # TODO implement read of Spec Sensor NO2 sensor
            print('NO2...wait on barrier_data_ready')
            await self.barrier_data_ready

class ParticulateSensor(pms5003.PMS5003):
    def __init__(self, 
                 uart, 
                 lock, 
                 active_mode, 
                 barrier_read, 
                 barrier_data_ready, 
                 event_new_pm25_data):
        self.barrier_read = barrier_read
        self.barrier_data_ready = barrier_data_ready
        self.event_new_pm25_data = event_new_pm25_data
        super().__init__(uart, 
                     lock, 
                     active_mode = active_mode, 
                     event = event_new_pm25_data)
        loop = asyncio.get_event_loop()
        loop.create_task(self.run()) 
        
    async def run(self):
        await self.stop() # place sensor in low power mode 
        while True:
            print('PM2.5...wait on barrier_read')
            await self.barrier_read
            print('PM2.5...STARTING sensor at {}'. format(ds3231.datetime()))
            await self.start()
            print('PM2.5...waiting for event at {}'. format(ds3231.datetime()))
            await self.event_new_pm25_data
            print('PM2.5...read sensor at {}'.format(ds3231.datetime()))
            print('PM2.5 pm25_env = {}'.format(await self.get_value())) 
            self.event_new_pm25_data.clear() 
            print('PM2.5...STOPPING sensor at {}'.format(ds3231.datetime()))
            print('PM2.5...wait on barrier_data_ready')
            await self.barrier_data_ready
            await self.stop()
        
    async def get_value(self):
        return self.pm25_env

class Display():
    def __init__(self, barrier):
        self.pm25 = None
        self.ozone = None
        self.no2 = None
        self.peakdb = None
        print('Display...wait on barrier')
        self.barrier = barrier
        loop = asyncio.get_event_loop()
        loop.create_task(self.run()) 
        
    # TODO 
    #    remove COUPLING Display class should NOT
    #    reach into other sensor classes to get data.
        
    async def run(self):
        while True:
            # show demo data for now
            await self.barrier
            oled.fill(0)
            oled.text("PM2.5:   {}".format(await ps.get_value()), 0, 0)
            oled.text("Ozone:   33", 0, 8) # TODO - show sensor data
            oled.text("NO2:      9", 0, 16) # TODO - show sensor data
            oled.text("Peak dB: 76", 0, 24) # TODO - show sensor data
            oled.show()
            await asyncio.sleep(1)

class IntervalTimer():
    def __init__(self, barrier):
        self.barrier = barrier
        loop = asyncio.get_event_loop()
        loop.create_task(self.run()) 
    
    async def run(self):
        global logging_time  # TODO fix this when interval timer becomes a class
        ds3231.alarm(False, alarm=0)  # TODO fix this coupling
        while True:
            time_now = urtc.tuple2seconds(ds3231.datetime())
            # calculate the next alarm time, aligned to the desired interval
            # e.g.  interval=15mins ==>  align to 00, 15, 30, 45 mins
            time_now += 5  # to eliminate risk of timing hazard close to interval boundary
            wake_time = int(math.ceil(time_now / LOGGING_INTERVAL_IN_SECS) * LOGGING_INTERVAL_IN_SECS)
            
            # set day-of-week (4th element) to None so alarm uses day-of-month (urtc module API requirement)
            # some gymnastics needed ...
            wake_time_tuple = urtc.seconds2tuple(int(wake_time))
            wake_time_list = list(wake_time_tuple)
            wake_time_list[3]=None  
            ds3231.alarm_time(wake_time_list, alarm=0)  # TODO fix coupling   
    
            # loop until the DS3231 alarm is detected 
            # TODO consider use of DS3231 hardware alarm pin, with ESP32 interrupt
            while ds3231.alarm(alarm=0) == False:
                await asyncio.sleep_ms(250)
                
    
            logging_time = urtc.tuple2seconds(ds3231.datetime())
            # clear alarm    
            ds3231.alarm(False, alarm=0)
    
            await self.barrier

class SDCardLogger():
    def __init__(self, barrier):
        sdconfig = uos.sdconfig(
                        uos.SDMODE_SPI,
                        clk=18,
                        mosi=23,
                        miso=19,
                        cs=4,
                        maxspeed=40,
                        )
        mount = uos.mountsd()
        self.barrier = barrier
        loop = asyncio.get_event_loop()
        loop.create_task(self.run()) 
        
    async def run(self):
        while True:
            s = open('/sd/samples.csv', 'a+')
            # wait until data for all sensors is available
            print('SD Card Logger...wait on barrier')
            await self.barrier
                # write sensor data to the SD Card in CSV form
            print('LOG IT !!!')
            numwrite = s.write('{}, {}\n'.format(logging_time, await ps.get_value()))
            s.close()
            
#
#  TODO add I2S Microphone implementation, with audio sample save to WAV file on micro SD Card
#    - stretch goal:  calculate noise db from audio samples
#

#
#  TODO add User Interface, likely using setup screens driven by buttons
#

#
#  TODO add MQTT sensor data push to either Thingspeak or Adafruit IO
#            
        
async def idle():
    while True:
        await asyncio.sleep(2)
        time.sleep_ms(5)
        #print(gc.mem_free())
 
i2c = machine.I2C(scl=machine.Pin(22), sda=machine.Pin(21))
ds3231 = urtc.DS3231(i2c, address=0x68)
oled = ssd1306.SSD1306_I2C(128, 32, i2c)
logging_time = None  #  TODO implement without using a global
pms5003.set_debug(False)
pms5003.WAIT_AFTER_WAKEUP = 30

loop = asyncio.get_event_loop()
uart = machine.UART(1, tx=32, rx=33, baudrate=9600)
lock = asyn.Lock()
event_new_pm25_data = asyn.Event() # TODO 200ms polling interval to reduce CPU overhead?

# TODO investigate making use of Barriers easier for a reader to understand
barrier_read_all_sensors = asyn.Barrier(4)   
barrier_sensor_data_ready = asyn.Barrier(5)

ps = ParticulateSensor(uart, 
                       lock, 
                       True, 
                       barrier_read_all_sensors, 
                       barrier_sensor_data_ready, 
                       event_new_pm25_data)
ozone = OzoneSensor(barrier_read_all_sensors, barrier_sensor_data_ready)
no2 = NO2Sensor(barrier_read_all_sensors, barrier_sensor_data_ready)
iterval_timer = IntervalTimer(barrier_read_all_sensors)
sdcard_logger = SDCardLogger(barrier_sensor_data_ready)
display = Display(barrier_sensor_data_ready)
loop.create_task(idle())  # needed on esp32_LoBo to not trigger watchdog.
loop.run_forever()