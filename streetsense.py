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

LOGGING_INTERVAL_IN_SECS = 60.0*2

NUM_BYTES_IN_SDCARD_SECTOR = 512

# I2S Microphone related config
SAMPLES_PER_SECOND = 10000
RECORD_TIME_IN_SECONDS = 60*5
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

# TODO combine ozone, no2 sensors into one class... after I2C ADC integration
# Pass in ADC object? ... TODO think of how to abstract it

class OzoneSensor():
    def __init__(self, barrier_read, barrier_data_ready):
        self.barrier_read = barrier_read
        self.barrier_data_ready = barrier_data_ready
        loop = asyncio.get_event_loop()
        loop.create_task(self.run_ozone()) 
        self.v_gas = None
        self.v_ref = None        
        
    async def run_ozone(self):
        while True:
            await self.barrier_read
            adc.set_channel(ADS1219.CHANNEL_AIN0)
            self.v_ref = adc.read_data()
            adc.set_channel(ADS1219.CHANNEL_AIN1)
            self.v_gas = adc.read_data()
            await self.barrier_data_ready
        
class NO2Sensor():
    def __init__(self, barrier_read, barrier_data_ready):
        self.barrier_read = barrier_read
        self.barrier_data_ready = barrier_data_ready
        loop = asyncio.get_event_loop()
        loop.create_task(self.run_no2())
        self.v_gas = None
        self.v_ref = None        
        
    async def run_no2(self):
        while True:
            await self.barrier_read
            adc.set_channel(ADS1219.CHANNEL_AIN3)
            self.v_ref = adc.read_data()
            adc.set_channel(ADS1219.CHANNEL_AIN2)
            self.v_gas = adc.read_data()
            await self.barrier_data_ready
            
class THSensor():
    def __init__(self, barrier_read, barrier_data_ready):
        self.barrier_read = barrier_read
        self.barrier_data_ready = barrier_data_ready
        loop = asyncio.get_event_loop()
        loop.create_task(self.run_th())
        self.temp_degc = None
        self.humid_rh = None        
        
    async def run_th(self):
        while True:
            await self.barrier_read
            self.temp_degc = temp_humid_sensor.temperature
            self.humid_rh = temp_humid_sensor.relative_humidity
            await self.barrier_data_ready            

#  TODO re-think design - what is the value in defining a class versus just a function?
class ParticulateSensor():
    def __init__(self, 
                 lock, 
                 barrier_read, 
                 barrier_data_ready, 
                 event_new_pm25_data):
        self.lock = lock
        self.barrier_read = barrier_read
        self.barrier_data_ready = barrier_data_ready
        self.event_new_pm25_data = event_new_pm25_data
        self.uart = machine.UART(1, tx=32, rx=33, baudrate=9600)
        loop = asyncio.get_event_loop()
        loop.create_task(self.run_pm25()) 
        
    async def run_pm25(self):
        pm25_pwr_pin = machine.Pin(14, machine.Pin.OUT)

        while True:
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

            machine.Pin(32, machine.Pin.IN, machine.Pin.PULL_DOWN)
            machine.Pin(33, machine.Pin.IN, machine.Pin.PULL_DOWN)
            
            print('PM2.5:  power-down')
            pm25_pwr_pin.value(0)

            await self.barrier_read
            print('PM2.5:  power-up')
            pm25_pwr_pin.value(1)
            await asyncio.sleep(2) # allow pwr-up time
            # reinitialize the UART as means have the GPIO pins
            # working as Tx and Rx 
            # note:  reinitialize PMS5003 sensor so the new UART object is 
            #        associated with the PMS5003 object
            self.uart = machine.UART(1, tx=32, rx=33, baudrate=9600)
            self.pm25 = pms5003.PMS5003(self.uart, self.lock, event = self.event_new_pm25_data)
            await asyncio.sleep(1)
            # TODO do some scope investigation to see how long it takes
            # for sensor to autonomously send the first data on Rx
            print('PM2.5:  set Passive mode')
            await self.pm25.setPassiveMode()
            print('PM2.5:  30s warm-up')
            await asyncio.sleep(30) # 30s warm-up period as specified in datasheet
            print('PM2.5:  read sensor')
            await self.pm25.read()
            print('PM2.5:  waiting for sensor event')
            await self.event_new_pm25_data
            print('PM2.5:  got sensor event')
            print('PM2.5:  value = {}'.format(await ps.get_value()))
            self.event_new_pm25_data.clear() 
            await self.barrier_data_ready
        
    async def get_value(self):
        return self.pm25.pm25_env

class IntervalTimer():
    def __init__(self, barrier_read):
        self.barrier_read = barrier_read
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
            while ds3231.alarm(alarm=0) == False:
                await asyncio.sleep_ms(250)

            sample_timestamp = urtc.tuple2seconds(ds3231.datetime())
            # clear alarm    
            ds3231.alarm(False, alarm=0)
            
            print('DS3231 alarm -> read all sensors')
            await self.barrier_read

class Display():
    def __init__(self, barrier_data_ready):
        self.barrier_data_ready = barrier_data_ready
        oled.fill(0)
        oled.text("Street Sense", 0, 0)
        oled.text("Online", 0, 8)
        oled.show()
        loop = asyncio.get_event_loop()
        loop.create_task(self.run_display()) 
        
    async def run_display(self):
        while True:
            # wait for display data to be ready
            await self.barrier_data_ready
            oled.fill(0)
            await asyncio.sleep(0)
            oled.text("O3 v_gas {}".format(ozone.v_gas), 0, 0)
            await asyncio.sleep(0)
            oled.text("NO2 v_gas {}".format(no2.v_gas), 0, 8)
            await asyncio.sleep(0)
            oled.text("Temp {:.1f}".format(temp_hum.temp_degc), 0, 16)
            await asyncio.sleep(0)
            oled.text("RH {:.1f}".format(temp_hum.humid_rh), 0, 24)
            await asyncio.sleep(0)
            oled.show()
            await asyncio.sleep(0)

class SDCardLogger():
    def __init__(self, barrier_data_ready):
        self.barrier_data_ready = barrier_data_ready
        loop = asyncio.get_event_loop()
        loop.create_task(self.run_logger()) 
        
    async def run_logger(self):
        while True:
            s = open('/sd/samples.csv', 'a+')
            await asyncio.sleep(0)
            # wait until data for all sensors is available
            await self.barrier_data_ready
            # write sensor data to the SD Card in CSV format
            numwrite = s.write('{}, {}, {}, {}, {:.1f}, {:.1f}\n'.format(
                                                        sample_timestamp, 
                                                        await ps.get_value(),
                                                        ozone.v_gas,
                                                        no2.v_gas,
                                                        temp_hum.temp_degc,
                                                        temp_hum.humid_rh))
            print('wrote log')
            await asyncio.sleep(0)
            print('Logger:  s.close')
            s.close()
            await asyncio.sleep(0)

class MQTTPublish():
    def __init__(self, barrier_data_ready):    
        self.barrier_data_ready = barrier_data_ready
        MQTTClient.DEBUG = True
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
            await self.barrier_data_ready
            self.client.resume()
            await self.client.publish(self.feedname_pm25, '{}'.format(await ps.get_value()), qos = 0)
            await self.client.publish(self.feedname_o3, '{}'.format(ozone.v_gas), qos = 0)
            await self.client.publish(self.feedname_no2, '{}'.format(no2.v_gas), qos = 0)
            await self.client.publish(self.feedname_temp, '{:.1f}'.format(temp_hum.temp_degc), qos = 0)
            await self.client.publish(self.feedname_humidity, '{:.1f}'.format(temp_hum.humid_rh), qos = 0)
            # pausing the MQTT client will turn off the WiFi radio
            # which reduces the processor power usage
            self.client.pause()            
            
class Microphone():
    def __init__(self):
        loop = asyncio.get_event_loop()
        loop.create_task(self.run_mic()) 
                
    async def run_mic(self):
        # dmacount range:  2 to 128 incl
        # dmalen range:   8 to 1024 incl
        
        bck_pin = machine.Pin(27)
        ws_pin = machine.Pin(26)
        sdin_pin = machine.Pin(25)

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
        
class SpecTest():
    def __init__(self):
        loop = asyncio.get_event_loop()
        loop.create_task(self.run_spec()) 
        
    async def run_spec(self):
        log_duration_in_s = 120
        delay_ms = 500
        # delete old logfile if it exists
        files = uos.listdir("/sd")
        if 'logfile.csv' in files:
            print('removing old logfile')
            uos.remove('/sd/logfile.csv')

        log_file = open('/sd/logfile.csv', 'wt')
        iterations = log_duration_in_s / (delay_ms * 2)
        print('start Spec Sensor test')
        while iterations:
            adc.set_channel(ADS1219.CHANNEL_AIN0)
            vref_ozone = adc.read_data()
            await asyncio.sleep_ms(delay_ms)
            adc.set_channel(ADS1219.CHANNEL_AIN1)
            vgas_ozone = adc.read_data()
            await asyncio.sleep_ms(delay_ms)
            
            adc.set_channel(ADS1219.CHANNEL_AIN3)
            vref_no2 = adc.read_data()
            await asyncio.sleep_ms(delay_ms)
            adc.set_channel(ADS1219.CHANNEL_AIN2)
            vgas_no2 = adc.read_data()
            await asyncio.sleep_ms(delay_ms)
            
            log_string = '{}, {}, {:4.4f}, {:4.4f}, {:4.4f}, {:4.4f}\n'.format(
                vgas_ozone, vgas_no2, vgas_ozone*2.048*1000/(2**23), vref_ozone*2.048*1000/(2**23),
                vgas_no2*2.048*1000/(2**23), vref_no2*2.048*1000/(2**23))
            log_file.write(log_string)
            print(log_string)
            iterations -= 1
            await asyncio.sleep_ms(delay_ms)
        log_file.close()   
        print('end Spec Sensor test')
  
async def idle():
    while True:
        await asyncio.sleep(2)
        utime.sleep_us(1)
        #print(gc.mem_free())

  
#
#  TODO add User Interface, likely using setup screens driven by buttons
#

#
#  TODO add MQTT sensor data push to either Thingspeak or Adafruit IO
#     

# 
# TODO add temperature/humidity sensor
#

#
#  TODO add stretch goal:  calculate noise db from audio samples
#

if sys.platform == 'esp32_LoBo':  
    gc.threshold(GC_THRESHOLD, 0)  #  2nd arg=1 turns on GC debug  
else:
    gc.threshold(GC_THRESHOLD)  
              
logging.basicConfig(level=logging.DEBUG)
i2c = machine.I2C(scl=machine.Pin(22), sda=machine.Pin(21))
ds3231 = urtc.DS3231(i2c, address=0x68)
oled = ssd1306.SSD1306_I2C(128, 32, i2c)
adc = ADS1219(i2c, address=0x41)
adc.set_conversion_mode(ADS1219.CM_SINGLE)
adc.set_gain(ADS1219.GAIN_1X)
adc.set_data_rate(ADS1219.DR_20_SPS)
adc.set_vref(ADS1219.VREF_INTERNAL)
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

# TODO investigate making use of Barriers easier for a reader to understand
barrier_read_all_sensors = asyn.Barrier(5)   
barrier_sensor_data_ready = asyn.Barrier(7)
ozone = OzoneSensor(barrier_read_all_sensors, barrier_sensor_data_ready)
no2 = NO2Sensor(barrier_read_all_sensors, barrier_sensor_data_ready)
temp_hum = THSensor(barrier_read_all_sensors, barrier_sensor_data_ready)
ps = ParticulateSensor(lock, 
                       barrier_read_all_sensors, 
                       barrier_sensor_data_ready, 
                       event_new_pm25_data)
display = Display(barrier_sensor_data_ready)
interval_timer = IntervalTimer(barrier_read_all_sensors)
sdcard_logger = SDCardLogger(barrier_sensor_data_ready)
mic = Microphone()
mqtt = MQTTPublish(barrier_sensor_data_ready)
#spec_sensor_testing = SpecTest()
loop.create_task(idle())  # workaround for watchdog trigger issue in LoBo port
loop.run_forever()