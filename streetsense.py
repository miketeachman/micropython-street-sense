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
import math
import ssd1306
import pms5003
import urtc
from machine import I2S
import logging
import ads1x15

logging.basicConfig(level=logging.DEBUG)

#    SPI Device
#    - micro SD Card
# 
#    SPI Connections
#    Pin   Function
#    4     Tx
#    18    Rx
#    19    xx
#    23    xx

#    UART Device
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
#    - ADS1015 12-bit ADC
#    
#    I2C Connections
#    Pin   Function
#    21    SDA
#    22    SCL

#    I2S Device
#    - Adafruit I2S MEMS Microphone Breakout - SPH0645LM4H
#
#    I2S Connections
#    Pin   Function
#    25    SCK
#    26    WS
#    27    SDIN
#    

LOGGING_INTERVAL_IN_SECS = 120.0

NUM_BYTES_IN_SDCARD_SECTOR = 512

# I2S Microphone related config
SAMPLES_PER_SECOND = 20000
RECORD_TIME_IN_SECONDS = 10
NUM_BYTES_RX = 8
NUM_BYTES_USED = 2
BITS_PER_SAMPLE = NUM_BYTES_USED * 8
NUM_BYTES_IN_SAMPLE_BLOCK = NUM_BYTES_IN_SDCARD_SECTOR * (NUM_BYTES_RX // NUM_BYTES_USED)
NUM_SAMPLE_BYTES_IN_WAV = (RECORD_TIME_IN_SECONDS * SAMPLES_PER_SECOND * NUM_BYTES_USED)

PM25_POLLING_DELAY_MS = 500

def gen_wav_header(
    sampleRate,
    bitsPerSample,
    channels,
    samples,
    ):
    datasize = samples * channels * bitsPerSample // 8
    o = bytes('RIFF', 'ascii')  # (4byte) Marks file as RIFF
    o += (datasize + 36).to_bytes(4, 'little')  # (4byte) File size in bytes excluding this and RIFF marker
    o += bytes('WAVE', 'ascii')  # (4byte) File type
    o += bytes('fmt ', 'ascii')  # (4byte) Format Chunk Marker
    o += (16).to_bytes(4, 'little')  # (4byte) Length of above format data
    o += (1).to_bytes(2, 'little')  # (2byte) Format type (1 - PCM)
    o += channels.to_bytes(2, 'little')  # (2byte)
    o += sampleRate.to_bytes(4, 'little')  # (4byte)
    o += (sampleRate * channels * bitsPerSample // 8).to_bytes(4,
            'little')  # (4byte)
    o += (channels * bitsPerSample // 8).to_bytes(2, 'little')  # (2byte)
    o += bitsPerSample.to_bytes(2, 'little')  # (2byte)
    o += bytes('data', 'ascii')  # (4byte) Data Chunk Marker
    o += datasize.to_bytes(4, 'little')  # (4byte) Data size in bytes
    return o

# TODO combine ozone, no2 sensors into one class... after I2C ADC integration
# Pass in ADC object? ... TODO think of how to abstract it... dHylands approach.

class OzoneSensor():
    def __init__(self, barrier_read, barrier_data_ready):
        self.barrier_read = barrier_read
        self.barrier_data_ready = barrier_data_ready
        loop = asyncio.get_event_loop()
        loop.create_task(self.run()) 
        self.v_gas = None
        self.v_ref = None
        
    async def get_ppm(self):
        return self.v_gas
    
    async def run(self):
        while True:
            await self.barrier_read
            self.v_gas = adc.read(1)
            yield
            self.v_ref = adc.read(0)
            await self.barrier_data_ready
        
class NO2Sensor():
    def __init__(self, barrier_read, barrier_data_ready):
        self.barrier_read = barrier_read
        self.barrier_data_ready = barrier_data_ready
        loop = asyncio.get_event_loop()
        loop.create_task(self.run())
        self.v_gas = None
        self.v_ref = None
        
    async def get_value(self):
        return self.v_gas
        
    async def run(self):
        while True:
            await self.barrier_read
            self.v_gas = adc.read(3)
            yield
            self.v_ref = adc.read(2)
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
            await self.barrier_read
            await self.start()
            await self.event_new_pm25_data
            self.event_new_pm25_data.clear() 
            await self.barrier_data_ready
            await self.stop()
        
    async def get_value(self):
        return self.pm25_env

class Display():
    def __init__(self, barrier_sensor_data_ready):
        self.pm25 = None
        self.ozone = None
        self.no2 = None
        self.peakdb = None
        self.barrier_sensor_data_ready = barrier_sensor_data_ready
        loop = asyncio.get_event_loop()
        loop.create_task(self.run()) 
        
    async def run(self):
        while True:
            # show demo data for now
            await self.barrier_sensor_data_ready
            oled.fill(0)
            yield
            oled.text("Ozone v_gas {}".format(ozone.v_gas), 0, 0)
            yield
            oled.text("Ozone v_ref {}".format(ozone.v_ref), 0, 8)
            yield
            oled.text("NO2 v_gas {}".format(no2.v_gas), 0, 16)
            yield
            oled.text("NO2 v_ref  {}".format(no2.v_ref), 0, 24)
            yield
            oled.show()

class IntervalTimer():
    def __init__(self, barrier):
        self.barrier = barrier
        loop = asyncio.get_event_loop()
        loop.create_task(self.run()) 
    
    async def run(self):
        global sample_timestamp  # TODO fix this when interval timer becomes a class
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
            while ds3231.alarm(alarm=0) == False:
                await asyncio.sleep_ms(250)

            sample_timestamp = urtc.tuple2seconds(ds3231.datetime())
            # clear alarm    
            ds3231.alarm(False, alarm=0)
    
            await self.barrier

class SDCardLogger():
    def __init__(self, barrier):
        self.barrier = barrier
        loop = asyncio.get_event_loop()
        loop.create_task(self.run()) 
        
    async def run(self):
        while True:
            s = open('/sd/samples.csv', 'a+')
            # wait until data for all sensors is available
            await self.barrier
            
            # write sensor data to the SD Card in CSV form
            numwrite = s.write('{}, {}, {}, {}, {}, {}\n'.format(
                                                sample_timestamp, 
                                                await ps.get_value(),
                                                ozone.v_gas,
                                                ozone.v_ref,
                                                no2.v_gas,
                                                no2.v_ref))
            yield
            s.close()
            yield
            
            
            
class Microphone():
    def __init__(self):
        loop = asyncio.get_event_loop()
        loop.create_task(self.run_mic()) 
        pass
        
    async def run_mic(self):
        m=open('/sd/upy.wav','wb')
        wav_header = gen_wav_header(SAMPLES_PER_SECOND, BITS_PER_SAMPLE, 1,
                            SAMPLES_PER_SECOND * RECORD_TIME_IN_SECONDS)
        m.write(wav_header)
        numread = 0
        numwrite = 0
        samples = bytearray(NUM_BYTES_IN_SAMPLE_BLOCK)
        sd_sector = bytearray(NUM_BYTES_IN_SDCARD_SECTOR)

        for _ in range(NUM_SAMPLE_BYTES_IN_WAV // NUM_BYTES_IN_SDCARD_SECTOR):
            try:
                # read sample block from microphone
                numread = audio.readinto(samples)
                
                # prune samples
                for i in range(NUM_BYTES_IN_SAMPLE_BLOCK // NUM_BYTES_RX):
                    sd_sector[2*i] = samples[8*i + 2]
                    sd_sector[2*i + 1] = samples[8*i + 3]
                
                # write samples to SD Card
                numwrite = m.write(sd_sector)
                yield
                
            except Exception as e:
                print('unexpected exception {} {}'.format(type(e).__name__, e))
                m.close()
                audio.deinit()
        m.close()
        audio.deinit()

#
#  TODO add User Interface, likely using setup screens driven by buttons
#

#
#  TODO add MQTT sensor data push to either Thingspeak or Adafruit IO
#     

# 
# TODO add temperature sensor
#

#
#  TODO add stretch goal:  calculate noise db from audio samples
#

async def idle():
    while True:
        await asyncio.sleep(2)
        utime.sleep_us(1)
        
i2c = machine.I2C(scl=machine.Pin(22), sda=machine.Pin(21))
ds3231 = urtc.DS3231(i2c, address=0x68)
oled = ssd1306.SSD1306_I2C(128, 32, i2c)
adc = ads1x15.ADS1015(i2c)
adc.gain=1  # +- 4.096 range  TODO extend class - add method or init to set/get gain.
uart = machine.UART(1, tx=32, rx=33, baudrate=9600)

# dmacount range:  2 to 128 incl
# dmalen range:   8 to 1024 incl
audio=I2S(id=I2S.NUM0,
          sck=25,
          ws=26,
          sdin=27,
          mode=I2S.MASTER|I2S.RX,
          samplerate=SAMPLES_PER_SECOND,
          bits=I2S.BPS32,
          channelformat=I2S.RIGHT_LEFT,
          commformat=I2S.I2S|I2S.I2S_MSB,
          dmacount=128,
          dmalen=128)

sample_timestamp = None  #  TODO implement without using a global
pms5003.set_debug(False)
pms5003.WAIT_AFTER_WAKEUP = 30
asyncio.set_debug(0)
asyncio.core.set_debug(0)


sdconfig = uos.sdconfig(
                        uos.SDMODE_SPI,
                        clk=18,
                        mosi=23,
                        miso=19,
                        cs=4,
                        maxspeed=40,
                        )
mount = uos.mountsd()

loop = asyncio.get_event_loop()
lock = asyn.Lock()
event_new_pm25_data = asyn.Event(PM25_POLLING_DELAY_MS) 

# TODO investigate making use of Barriers easier for a reader to understand
barrier_read_all_sensors = asyn.Barrier(4)   
barrier_sensor_data_ready = asyn.Barrier(5)

ozone = OzoneSensor(barrier_read_all_sensors, barrier_sensor_data_ready)
no2 = NO2Sensor(barrier_read_all_sensors, barrier_sensor_data_ready)
ps = ParticulateSensor(uart, 
                       lock, 
                       True, 
                       barrier_read_all_sensors, 
                       barrier_sensor_data_ready, 
                       event_new_pm25_data)
display = Display(barrier_sensor_data_ready)
iterval_timer = IntervalTimer(barrier_read_all_sensors)
sdcard_logger = SDCardLogger(barrier_sensor_data_ready)
mic = Microphone()
loop.create_task(idle())  # workaround for watchdog trigger issue in LoBo port
loop.run_forever()