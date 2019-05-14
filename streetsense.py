# The MIT License (MIT)
# Copyright (c) 2019 Mike Teachman
# https://opensource.org/licenses/MIT

#
# Street Sense Project:  Air and Noise pollution sensor unit
# https://hackaday.io/project/162059-street-sense
#

import gc
import sys
import math
import machine
from machine import I2S
from machine import I2C
from machine import Pin
from machine import UART
from array import array
import uos
import utime
from mqtt_as import MQTTClient
from mqtt_config import mqtt_config
import uasyncio as asyncio
import asyn
import ms_timer
import logging
from ads1219 import ADS1219
import ssd1306
import pms5003
import urtc
import si7021
from bitcrusher import bitcrusher

#    SPI Device
#    - micro SD Card
# 
#    SPI Connections
#    Pin   Function
#    4     CS
#    18    SCK
#    19    MISO
#    23    MOSI

#    UART Device
#    - PMS5003 Particulate Sensor
#
#    UART Connections
#    Pin   Function
#    32    Tx
#    33    Rx
#
#    Sensor Power Control
#    Pin   Function
#    14    Pwr on/off
#

#    I2C Devices
#    - SSD1306 OLED Display
#    - DS3231 Real Time Clock
#    - ADS1219 24-bit ADC
#    
#    I2C Connections
#    Pin   Function
#    21    SDA
#    22    SCL

#    I2S Device
#    - INMP441 omnidirectional MEMS microphone
#
#    I2S Connections
#    Pin   Function
#    27    SCK
#    26    WS
#    25    SD
#  
#    ADS1219 ADC
#    Pin   Function
#    34    DRDY

LOGGING_INTERVAL_IN_SECS = 60.0*2

NUM_BYTES_IN_SDCARD_SECTOR = 512

# I2S Microphone related config
SAMPLES_PER_SECOND = 10000
RECORD_TIME_IN_SECONDS = 60*60*1
NUM_BYTES_RX = 8
NUM_BYTES_USED = 2
BITS_PER_SAMPLE = NUM_BYTES_USED * 8
NUM_BYTES_IN_SAMPLE_BLOCK = NUM_BYTES_IN_SDCARD_SECTOR * (NUM_BYTES_RX // NUM_BYTES_USED)
NUM_SAMPLE_BYTES_IN_WAV = (RECORD_TIME_IN_SECONDS * SAMPLES_PER_SECOND * NUM_BYTES_USED)
NUM_SAMPLE_BYTES_TO_RX = ((RECORD_TIME_IN_SECONDS * SAMPLES_PER_SECOND * NUM_BYTES_RX))

PM25_POLLING_DELAY_MS = 500

# thresholds < 20480 will result in GC only on-demand (when allocation fails)        
GC_THRESHOLD = 10

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

# TODO pass in ADC object
class SpecSensors():
    SAMPLES_TO_CAPTURE = 100
    CALIBRATION_FACTOR_OZONE = (-21.7913*(10**-3))  # mV/ppb
    CALIBRATION_FACTOR_NO2 = (-11.13768*(10**-3))   # mV/ppb
                          
    def __init__(self):
        self.sample_count = 0
        self.sample_sum = 2**32-1   # allocate 4 byte sample to be used in ISR  TODO needed?

        adc.set_channel(ADS1219.CHANNEL_AIN0)
        adc.set_conversion_mode(ADS1219.CM_SINGLE)
        adc.set_gain(ADS1219.GAIN_1X)
        adc.set_data_rate(ADS1219.DR_20_SPS)
        adc.set_vref(ADS1219.VREF_INTERNAL)
        self.drdy_pin = Pin(34, mode=Pin.IN, trigger=Pin.IRQ_DISABLE, handler=self.callback)        
        self.ozone_v_gas = None
        self.ozone_v_ref = None
        self.ozone_ppb = None
        self.no2_v_gas = None        
        self.no2_v_ref = None
        self.no2_ppb = None        
        
    def callback(self, arg):
        if self.sample_count < self.SAMPLES_TO_CAPTURE:
            self.sample_sum += adc.read_data_irq()
            self.sample_count += 1
            # re-enable interrupt
            self.drdy_pin.init(trigger=Pin.IRQ_LOLEVEL)
        
    async def read(self, adc_channel):
        print('adc_channel=', adc_channel)
        self.sample_sum= 0 
        self.sample_count = 0     
        adc.set_channel(adc_channel)
        adc.set_conversion_mode(ADS1219.CM_CONTINUOUS)
        adc.set_gain(ADS1219.GAIN_1X)
        adc.set_data_rate(ADS1219.DR_20_SPS)
        adc.set_vref(ADS1219.VREF_INTERNAL)
        adc.start_sync() # starts continuous sampling
        
        await asyncio.sleep(1)
        print('start', utime.ticks_ms())

        self.drdy_pin.init(trigger=Pin.IRQ_LOLEVEL)
        
        while self.sample_count < self.SAMPLES_TO_CAPTURE:
            #print(self.sample_count)
            await asyncio.sleep_ms(10)

        self.drdy_pin.init(trigger=Pin.IRQ_DISABLE)
        print('done', utime.ticks_ms())
        
        adc.set_conversion_mode(ADS1219.CM_SINGLE)
        
        avg_mv = self.sample_sum*2.048*1000/(2**23)/self.sample_count
        print("avg_mv = ", avg_mv)
        
        return avg_mv
    
    async def read_all(self):
        # read Ozone gas voltage
        self.ozone_v_gas = await self.read(ADS1219.CHANNEL_AIN1)        
        # read Ozone reference voltage
        self.ozone_v_ref = await self.read(ADS1219.CHANNEL_AIN0)        
        # read NO2 gas voltage
        self.no2_v_gas = await self.read(ADS1219.CHANNEL_AIN2)        
        # read NO2 reference voltage
        self.no2_v_ref = await self.read(ADS1219.CHANNEL_AIN3)        
        
        # calculate gas concentration in parts-per-billion (ppb)
        # TODO calibrate Spec Sensors, with offset
        self.ozone_ppb = (self.ozone_v_gas - self.ozone_v_ref) / self.CALIBRATION_FACTOR_OZONE
        self.no2_ppb = (self.no2_v_gas - self.no2_v_ref) / self.CALIBRATION_FACTOR_NO2
        
class THSensor():
    def __init__(self):
        self.temp_degc = None
        self.humid_rh = None        
        
    async def read(self):
        self.temp_degc = temp_humid_sensor.temperature
        self.humid_rh = temp_humid_sensor.relative_humidity

class ParticulateSensor():
    def __init__(self, 
                 lock, 
                 event_new_pm25_data):
        self.lock = lock
        self.event_new_pm25_data = event_new_pm25_data
        self.pm25_reading = None
        
    async def read_pm25(self):
        pm25_pwr_pin = Pin(14, Pin.OUT)

        print('PM2.5:  power-up')
        pm25_pwr_pin.value(1)
        print('PM2.5:  30s warm-up')
        await asyncio.sleep(30) # 30s warm-up period as specified in datasheet

        # reinitialize the UART as means have the GPIO pins
        # working as Tx and Rx 
        # note:  reinitialize PMS5003 sensor so the new UART object is 
        #        associated with the PMS5003 object
        self.uart = UART(1, tx=32, rx=33, baudrate=9600)
        self.pm25 = pms5003.PMS5003(self.uart, self.lock, event = self.event_new_pm25_data)
        await asyncio.sleep(1)
        print('PM2.5:  set Passive mode')
        await self.pm25.setPassiveMode()
        print('PM2.5:  read sensor')
        await asyncio.sleep(1)
        await self.pm25.read()
        print('PM2.5:  waiting for sensor event')
        await self.event_new_pm25_data
        print('PM2.5:  got sensor event')
        self.pm25_reading = await ps.get_value()
        print('PM2.5:  value = {}'.format(self.pm25_reading))
        self.event_new_pm25_data.clear() 
        
        # Next section of code sets UART Tx and Rx pins as inputs with pull downs
        # This is done to make sure Tx and Rx are at zero volts
        # before the particulate sensor is powered off
        # A non-obvious sequence of events is needed to achieve this:
        # 1. deinit_start():  flags the UART task to stop running
        # 2. test UART task in a non-blocking wait loop until it stops
        # 3. deinit_finish():  final steps to terminate the UART task
        # 4. set Tx and Rx pins as pulled-down inputs
        # 5. power down the particulate sensor
        self.uart.deinit_start()
        iter = 50
        while iter and self.uart.is_task_running():
            await asyncio.sleep_ms(100)
            iter -= 1
            
        if not iter:
            raise ValueError('could not stop UART task')
        else:
            self.uart.deinit_finish()

        Pin(32, Pin.IN, Pin.PULL_DOWN)
        Pin(33, Pin.IN, Pin.PULL_DOWN)
        
        print('PM2.5:  power-down')
        pm25_pwr_pin.value(0)
        
    async def get_value(self):
        return self.pm25.pm25_env

class IntervalTimer():
    def __init__(self, event_mqtt_publish):    
        self.event_mqtt_publish = event_mqtt_publish
        loop = asyncio.get_event_loop()
        loop.create_task(self.run_timer()) 
    
    async def run_timer(self):
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
    
            print('next sensor read at {}'.format(wake_time_list))
            print('waiting for DS3231 alarm')
            # loop until the DS3231 alarm is detected 
            # TODO consider use of DS3231 hardware alarm pin, with ESP32 interrupt
            # e.g.  wait on asyncio event, trigger event IRQ routine
            while ds3231.alarm(alarm=0) == False:
                await asyncio.sleep_ms(250)

            sample_timestamp = urtc.tuple2seconds(ds3231.datetime())
            # clear alarm    
            ds3231.alarm(False, alarm=0)
            
            print('DS3231 alarm -> read all sensors')
            # dispatch a whole pile of activity
            await ps.read_pm25()
            await spec_sensors.read_all()
            await temp_hum.read()
            await display.display()
            await sdcard_logger.run_logger()
            self.event_mqtt_publish.set()

class Display():
    def __init__(self):
        oled.fill(0)
        oled.text("Street Sense", 0, 0)
        oled.text("Online !", 0, 8)
        oled.show()
        
    async def display(self):
        # wait for display data to be ready
        oled.fill(0)
        await asyncio.sleep(0)
        oled.text("O3 ppb {}".format(spec_sensors.ozone_ppb), 0, 0)
        await asyncio.sleep(0)
        oled.text("NO2 ppb {}".format(spec_sensors.no2_ppb), 0, 8)
        await asyncio.sleep(0)
        oled.text("PM25 {}".format(ps.pm25_reading), 0, 16)
        await asyncio.sleep(0)
        oled.text("TdegC {}".format(temp_hum.temp_degc), 0, 24)
        await asyncio.sleep(0)
        oled.show()

class SDCardLogger():
    def __init__(self):
        pass
        
    async def run_logger(self):
        s = open('/sd/samples.csv', 'a+')
        await asyncio.sleep(0)
        # wait until data for all sensors is available
        # write sensor data to the SD Card in CSV format
        numwrite = s.write('{}, {}, {}, {}, {}, {}, {:.1f}, {:.1f}\n'.format(
                                                    sample_timestamp, 
                                                    ps.pm25_reading,
                                                    spec_sensors.ozone_v_gas,
                                                    spec_sensors.ozone_v_ref,
                                                    spec_sensors.no2_v_gas,
                                                    spec_sensors.no2_v_ref,
                                                    temp_hum.temp_degc,
                                                    temp_hum.humid_rh))
        print('wrote log')
        await asyncio.sleep(0)
        print('Logger:  s.close')
        s.close()
        await asyncio.sleep(0)

class MQTTPublish():
    def __init__(self, event_mqtt_publish):    
        self.event_mqtt_publish = event_mqtt_publish    
        MQTTClient.DEBUG = False
        self.feedname_pm25 = bytes('{:s}/feeds/{:s}'.format(b'MikeTeachman', b'PM25'), 'utf-8')
        self.feedname_o3 = bytes('{:s}/feeds/{:s}'.format(b'MikeTeachman', b'o3'), 'utf-8')
        self.feedname_no2 = bytes('{:s}/feeds/{:s}'.format(b'MikeTeachman', b'no2'), 'utf-8')
        self.feedname_temp = bytes('{:s}/feeds/{:s}'.format(b'MikeTeachman', b'TdegC'), 'utf-8')
        self.feedname_humidity = bytes('{:s}/feeds/{:s}'.format(b'MikeTeachman', b'humidity'), 'utf-8')
        
        self.client = MQTTClient(server='io.adafruit.com', 
                                 ssid=mqtt_config['ssid'], 
                                 wifi_pw=mqtt_config['wifi_pw'], 
                                 user=mqtt_config['user'], 
                                 password=mqtt_config['password'])
        loop = asyncio.get_event_loop()
        try:
            loop.create_task(self.run_mqtt())
        finally:
            self.client.close()  # Prevent LmacRxBlk:1 errors  
        
    async def run_mqtt(self):
        await self.client.connect()
        self.client.pause()
        while True:
            await self.event_mqtt_publish
            print('turn WiFi on')
            self.client.resume()
            await self.client.publish(self.feedname_pm25, '{}'.format(ps.pm25_reading), qos = 0)
            await self.client.publish(self.feedname_o3, '{}'.format(spec_sensors.ozone_ppb), qos = 0)
            await self.client.publish(self.feedname_no2, '{}'.format(spec_sensors.no2_ppb), qos = 0)
            await self.client.publish(self.feedname_temp, '{:.1f}'.format(temp_hum.temp_degc), qos = 0)
            await self.client.publish(self.feedname_humidity, '{:.1f}'.format(temp_hum.humid_rh), qos = 0)
            # pausing the MQTT client will turn off the WiFi radio
            # which reduces the processor power usage
            print('turn WiFi off')
            self.client.pause()
            self.event_mqtt_publish.clear()
            
class Microphone():
    def __init__(self):
        loop = asyncio.get_event_loop()
        loop.create_task(self.run_mic()) 
                
    async def run_mic(self):
        # dmacount range:  2 to 128 incl
        # dmalen range:   8 to 1024 incl
        
        bck_pin = Pin(27)
        ws_pin = Pin(26)
        sdin_pin = Pin(25)

        audio=I2S(I2S.NUM0,
            bck=bck_pin,
            ws=ws_pin,
            sdin=sdin_pin,
            mode=I2S.MASTER_RX,
            samplerate=SAMPLES_PER_SECOND,
            dataformat=I2S.B32,
            channelformat=I2S.RIGHT_LEFT,
            standard=I2S.PHILIPS,
            dmacount=64,
            dmalen=256)
        timer_ms = ms_timer.MillisecTimer()
        m=open('/sd/upy.wav','wb')
        wav_header = gen_wav_header(SAMPLES_PER_SECOND, BITS_PER_SAMPLE, 1,
                            SAMPLES_PER_SECOND * RECORD_TIME_IN_SECONDS)
        m.write(wav_header)
        numread = 0
        numwrite = 0
        bytes_in_dma_memory = 0
        overrun_count = 0
        sdwrite_over_100ms = 0
        sdwrite_50_to_100ms = 0
        sdwrite_count = 0
        dma_byte_capacity = 64 * 256 * NUM_BYTES_RX  # dmacount*dmalen*8
        samples = bytearray(NUM_BYTES_IN_SAMPLE_BLOCK)
        sd_sector = bytearray(NUM_BYTES_IN_SDCARD_SECTOR)
        bytes_remaining_to_rx = NUM_SAMPLE_BYTES_TO_RX
        print('Mic:  Start')
        while bytes_remaining_to_rx > 0:
            try:
                start_ticks_us = utime.ticks_us()

                # read samples from microphone
                numread = audio.readinto(samples, timeout=0)
                bytes_remaining_to_rx -= numread
                
                if numread == 0:
                    # no samples available in DMA memory
                    # allow lower priority coroutines to run
                    await timer_ms(2)
                    bytes_in_dma_memory = 0
                else:
                    bytes_in_dma_memory -= numread
                    
                    # reduce sample resolution to 16 bits
                    bitcrusher(samples, sd_sector)
                
                    # write samples to SD Card
                    start_sd_write = utime.ticks_us()
                    #utime.sleep_us(3000)
                    numwrite = m.write(sd_sector)
                    end_sd_write = utime.ticks_us()
                    sdwrite_count += 1
                    
                    sd_write_time = end_sd_write - start_sd_write
                    if (sd_write_time) > 100*1000:  # 100ms
                        sdwrite_over_100ms += 1
                    elif (sd_write_time) > 50*1000:  # 50ms
                        sdwrite_50_to_100ms += 1
                    
                end_ticks_us = utime.ticks_us()
                # TODO use ticks diff us routine
                bytes_in_dma_memory += ((end_ticks_us - start_ticks_us) * (SAMPLES_PER_SECOND * NUM_BYTES_RX)) // 1000000
                if bytes_in_dma_memory > dma_byte_capacity:
                    overrun_count += 1
                    #print('Mic:  DMA overrun!, count= ', overrun_count)
                
            except Exception as e:
                print('unexpected exception {} {}'.format(type(e).__name__, e))
                m.close()
                audio.deinit()
        m.close()
        audio.deinit()
        print('================ Mic:  Done ======================')
        print('Mic Stats:  overrun_count: {}, sdwrite_50_to_100ms: {}, sdwrite_over_100ms: {}, sdwrite_count:  {}'.format(overrun_count, sdwrite_50_to_100ms, sdwrite_over_100ms, sdwrite_count))
        #machine.reset()
        #print('done mic. Percentage of zero I2S reads {:.1f}% '.format((1 - (count_sdwrite / count_i2sread)) * 100))
  
async def idle():
    while True:
        await asyncio.sleep(2)
        utime.sleep_us(1)
        #print(gc.mem_free())

#
#  TODO add User Interface, likely using setup screens driven by buttons
#

#
#  TODO add stretch goal:  calculate noise db from audio samples
#

if sys.platform == 'esp32_LoBo':  
    gc.threshold(GC_THRESHOLD, 0)  #  2nd arg=1 turns on GC debug  
else:
    gc.threshold(GC_THRESHOLD)  
              
logging.basicConfig(level=logging.DEBUG)
i2c = I2C(scl=Pin(22), sda=Pin(21))
ds3231 = urtc.DS3231(i2c, address=0x68)
oled = ssd1306.SSD1306_I2C(128, 32, i2c)
adc = ADS1219(i2c, address=0x41)
temp_humid_sensor = si7021.Si7021(i2c)

sample_timestamp = None  #  TODO implement without using a global
pms5003.set_debug(False)
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

loop = asyncio.get_event_loop(ioq_len=2)
lock = asyn.Lock()
event_new_pm25_data = asyn.Event(PM25_POLLING_DELAY_MS)
event_mqtt_publish = asyn.Event() 

spec_sensors = SpecSensors()
temp_hum = THSensor()
ps = ParticulateSensor(lock, event_new_pm25_data)
display = Display()
interval_timer = IntervalTimer(event_mqtt_publish)
sdcard_logger = SDCardLogger()
mic = Microphone()
mqtt = MQTTPublish(event_mqtt_publish)
loop.create_task(idle())  # workaround for watchdog trigger issue in LoBo port
loop.run_forever()