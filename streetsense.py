# The MIT License (MIT)
# Copyright (c) 2019 Mike Teachman
# https://opensource.org/licenses/MIT

#
# Street Sense Project:  Air and Noise pollution sensor unit
# https://hackaday.io/project/162059-street-sense
#

import gc
import sys
import esp
import math
import machine
from machine import I2S
from machine import I2C
from machine import Pin
from machine import UART
from machine import SDCard
from machine import ADC
from array import array
import uos
import utime
from mqtt_as import MQTTClient
from mqtt_config import mqtt_config
import uasyncio as asyncio
import asyn
from aswitch import Pushbutton
import ms_timer
import logging
from ads1219 import ADS1219
import lvgl as lv
import ILI9341 as ili
import lvesp32
import pms5003
import urtc
import si7021
import i2stools
import dba
#import fft
#import ustruct

###################################
#    GPIO Pin Allocation
###################################
#
#    SPI Devices
#    - Micro SD Card
#    - ILI9341 display
# 
#    Shared SPI Connections
#    Pin   Function
#    18    SCK
#    19    MISO
#    23    MOSI

#    Micro SD Card
#    4     CS

#    ILI9341 Display
#    Pin   Function
#    22    CS
#    21    DC
#    2     LED backlight control

#    UART Device
#    - PMS5003 Particulate Sensor
#
#    UART Connections
#    Pin   Function
#    32    Tx
#    33    Rx
#
#    Particulate Sensor Power Control
#    Pin   Function
#    25    Pwr on/off

#    I2C Devices
#    - DS3231 Real Time Clock (address = 0x68)
#    - ADS1219 24-bit ADC (address = 0x41)
#    - si7021 Temp/Humidity sensor (address = 0x40 )
#    
#    I2C Connections
#    Pin   Function
#    27    SDA
#    26    SCL

#    ADS1219 ADC
#    Pin   Function
#    34    DRDY

#    I2S Device
#    - INMP441 omnidirectional MEMS microphone
#
#    I2S Connections
#    Pin   Function
#    13    SCK
#    12    WS
#    14    SD
#  
#    Push Buttons
#    Pin   Function
#    0     Advance
#    15    Select

#    Analog Inputs
#    35    Battery Voltage. Resistive divider on Lolin board: BAT-100k-Pin35-100k-GND
#    39    USB Voltage. Resistive divider:  USB-68k-Pin39-100k-GND

#    UNUSED GPIO PINS
#    5
#    36

LOGGING_INTERVAL_IN_SECS = 60*2

#  TODO clean this up...confusing, complicated
# I2S Microphone related config
SAMPLES_PER_SECOND = 10000
RECORD_TIME_IN_SECONDS = 120
NUM_BYTES_RX = 8
NUM_BYTES_USED = 2  # this one is especially bad  TODO:  refactor
BITS_PER_SAMPLE = NUM_BYTES_USED * 8
NUM_BYTES_IN_SDCARD_SECTOR = 512
NUM_BYTES_IN_SAMPLE_BLOCK = NUM_BYTES_IN_SDCARD_SECTOR * (NUM_BYTES_RX // NUM_BYTES_USED)
NUM_SAMPLE_BYTES_IN_WAV = (RECORD_TIME_IN_SECONDS * SAMPLES_PER_SECOND * NUM_BYTES_USED)
NUM_SAMPLE_BYTES_TO_RX = ((RECORD_TIME_IN_SECONDS * SAMPLES_PER_SECOND * NUM_BYTES_RX))

PM25_POLLING_DELAY_MS = 500

# TODO put this function into a file
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
        self.drdy_pin = Pin(34, mode=Pin.IN)        
        self.ozone_v_gas = 0
        self.ozone_v_ref = 0
        self.ozone_ppb = 0
        self.no2_v_gas = 0        
        self.no2_v_ref = 0
        self.no2_ppb = 0        
        
    def callback(self, arg):
        if self.sample_count < self.SAMPLES_TO_CAPTURE:
            self.sample_sum += adc.read_data_irq()
            self.sample_count += 1
            # re-enable interrupt
            #self.drdy_pin.init(trigger=Pin.IRQ_LOLEVEL)
            self.drdy_pin.irq(trigger=Pin.IRQ_FALLING, handler=self.callback)
        
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

        self.drdy_pin.irq(trigger=Pin.IRQ_FALLING, handler=self.callback)
        
        while self.sample_count < self.SAMPLES_TO_CAPTURE:
            #print(self.sample_count)
            await asyncio.sleep_ms(10)

        self.drdy_pin.init()  # TBD - will this disable interrupt on pin ??
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
        self.temp_degc = 0
        self.humid_rh = 0        
        
    async def read(self):
        self.temp_degc = temp_humid_sensor.temperature
        self.humid_rh = temp_humid_sensor.relative_humidity

class ParticulateSensor():
    def __init__(self, 
                 lock, 
                 event_new_pm25_data):
        self.lock = lock
        self.event_new_pm25_data = event_new_pm25_data
        self.pm25_reading = 0
        self.pm25_pwr_pin = Pin(25, Pin.OUT)
        self.pm25_pwr_pin.value(0)
        
    async def read_pm25(self):
        print('PM2.5:  power-up')
        self.pm25_pwr_pin.value(1)
        print('PM2.5:  30s warm-up')
        await asyncio.sleep(30) # 30s warm-up period as specified in datasheet
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
        Pin(32, Pin.IN, Pin.PULL_DOWN)
        Pin(33, Pin.IN, Pin.PULL_DOWN)
        print('PM2.5:  power-down')
        self.pm25_pwr_pin.value(0)
        
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
            # TODO improve the first-time calc of wake_time ... missed starting about 10% of the time
            time_now += 10  # to eliminate risk of timing hazard close to interval boundary
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
            await sdcard_logger.run_logger()
            self.event_mqtt_publish.set()

class Display():
    def __init__(self):
        self.screens = [self.show_measurement_screen, self.show_voltage_monitor_screen, self.show_display_sleep_screen]
        pin_screen = Pin(0, Pin.IN, Pin.PULL_UP)
        pb_screen = Pushbutton(pin_screen)
        pb_screen.press_func(self.next_screen)
        self.active_screen = 1  # TODO make some sort of datastructure for screens + screen ids
        self.next_screen = 0 # show the measurement screen first TODO this is a clunky way to show this after measurement screen
        self.diag_count = 0
        self.backlight_ctrl = Pin(2, Pin.OUT)
        self.backlight_ctrl.value(1)
        loop = asyncio.get_event_loop()
        loop.create_task(self.run_display()) 
        
    # TODO power save mode:
    # - after a timeout, turn backlight off, and send cmd to put display to sleep
        
    async def run_display(self):
        # Initialize the ILI9341 driver
        # spihost:  1=HSPI 2=VSPI
        disp = ili.display(spihost=1, miso=19, mosi=23, clk=18, cs=22, dc=21, rst=15, backlight=2, mhz=20)
        disp.init()
        
        # Register display driver to LittlevGL
        # ... Start boilerplate magic
        disp_buf1 = lv.disp_buf_t()
        buf1_1 = bytearray(480*10)
        lv.disp_buf_init(disp_buf1,buf1_1, None, len(buf1_1)//4)
        disp_drv = lv.disp_drv_t()
        lv.disp_drv_init(disp_drv)
        disp_drv.buffer = disp_buf1
        disp_drv.flush_cb = disp.flush
        disp_drv.hor_res = 320  
        disp_drv.ver_res = 240
        disp_drv.rotated = 0
        lv.disp_drv_register(disp_drv)  
        # ... End boilerplate magic   
                   
        await self.show_welcome_screens()
        
        # continually refresh the active screen
        # detect screen change coming from a button press
        while True:
            if (self.next_screen != self.active_screen):
                self.active_screen = self.next_screen
            await self.screens[self.active_screen]()
            await asyncio.sleep(2)  # TODO idea:  each screen might have a configurable update time                  
        
    # following function is called when the screen advance button is pressed
    async def next_screen(self):
        self.next_screen = (self.active_screen + 1) % len(self.screens)
        
    async def show_welcome_screens(self):
        #
        # show Street Sense image 
        #
        welcome_screen1 = lv.obj()
        with open('street_sense_b_rgb565.bin','rb') as f:
            img_data = f.read()
            
        img = lv.img(welcome_screen1)
        img_dsc = lv.img_dsc_t({
            'header':{
                'always_zero': 0,
                'w':320,
                'h':240,
                'cf':lv.img.CF.TRUE_COLOR
            },
            'data_size': len(img_data),
            'data': img_data
        })
        
        img.set_src(img_dsc)
        
        lv.scr_load(welcome_screen1)
        await asyncio.sleep(1)
        #
        # show GVCC image 
        #
        welcome_screen2 = lv.obj()
        with open('gvcc_240x240_b_rgb565.bin','rb') as f:
            img_data = f.read()
            
        img = lv.img(welcome_screen2)
        img.set_x(40)  # center image by moving over 40px
        img_dsc = lv.img_dsc_t({
            'header':{
                'always_zero': 0,
                'w':240,
                'h':240,
                'cf':lv.img.CF.TRUE_COLOR
            },
            'data_size': len(img_data),
            'data': img_data
        })
        
        img.set_src(img_dsc)
        lv.scr_load(welcome_screen2)
        
        await asyncio.sleep(1)
        #
        # show GV Placemaking image 
        #
        welcome_screen3 = lv.obj()
        with open('placemaking_320x96_b_rgb565.bin','rb') as f:
            img_data = f.read()
            
        img = lv.img(welcome_screen3)
        img_dsc = lv.img_dsc_t({
            'header':{
                'always_zero': 0,
                'w':320,
                'h':96,
                'cf':lv.img.CF.TRUE_COLOR
            },
            'data_size': len(img_data),
            'data': img_data
        })
        
        img.set_src(img_dsc)
        lv.scr_load(welcome_screen3)   
        await asyncio.sleep(1)
        
    async def show_measurement_screen(self):
        # 
        # Measurement screen using a table
        #
        #
        # lv.table.STYLE.CELL1 = normal cell
        # lv.table.STYLE.CELL2 = header cell
        # lv.table.STYLE.CELL3 = ?
        # lv.table.STYLE.CELL4 = ?
        measurement_screen = lv.obj()
        
        # set background color, with no gradient
        screenstyle = lv.style_t(lv.style_plain)
        #screenstyle.body.main_color = lv.color_make(0xFF, 0xA5, 0x00)
        # 0xFF, 0x00, 0x00  Red
        # 0xC0, 0xC0, 0xC0  Silver
        # 0xFF, 0xA5, 0x00  Orange
        #screenstyle.body.grad_color = lv.color_make(0xFF, 0xA5, 0x00)
        screenstyle.body.border.color = lv.color_hex(0xe32a19)
        screenstyle.body.border.width = 5
        measurement_screen.set_style(screenstyle)
        
        tablestyle = lv.style_t(lv.style_plain)
        tablestyle.body.border.width = 0
        tablestyle.body.opa = 0
        
        cellstyle = lv.style_t(lv.style_plain)
        cellstyle.text.color = lv.color_hex(0xa028d4)
        cellstyle.text.font = lv.font_roboto_28
        cellstyle.body.padding.top = 1
        cellstyle.body.padding.bottom = 1
        cellstyle.body.border.width = 0
        cellstyle.body.opa = 0
        
        mtable = lv.table(measurement_screen)
        mtable.set_row_cnt(6)
        mtable.set_col_cnt(3)
        mtable.set_col_width(0, 110)
        mtable.set_col_width(1, 100)
        mtable.set_col_width(2, 100)
        mtable.set_style(lv.table.STYLE.BG, tablestyle)
        mtable.set_style(lv.table.STYLE.CELL1, cellstyle)
        
        mtable.set_cell_value(0,0, "NO2")
        mtable.set_cell_value(1,0, "O3")
        mtable.set_cell_value(2,0, "PM2.5")
        mtable.set_cell_value(3,0, "Temp")
        mtable.set_cell_value(4,0, "RH")
        mtable.set_cell_value(5,0, "Noise")
        
        mtable.set_cell_value(0,2, "ppb")
        mtable.set_cell_value(1,2, "ppb")
        mtable.set_cell_value(2,2, "ug/m3")
        mtable.set_cell_value(3,2, "degC")
        mtable.set_cell_value(4,2, "%")
        mtable.set_cell_value(5,2, "dB(A)")

        lv.scr_load(measurement_screen)
        self.backlight_ctrl.value(1)

        mtable.set_cell_value(0,1, '{:.1f}'.format(spec_sensors.no2_ppb))
        mtable.set_cell_value(1,1, '{:.1f}'.format(spec_sensors.ozone_ppb))
        mtable.set_cell_value(2,1, '{:.1f}'.format(ps.pm25_reading))
        mtable.set_cell_value(3,1, '{:.1f}'.format(temp_hum.temp_degc))
        mtable.set_cell_value(4,1, '{:.1f}'.format(temp_hum.humid_rh))
        mtable.set_cell_value(5,1, '{:.1f}'.format(mic.dba))
        
    async def show_voltage_monitor_screen(self): 
        # 
        # Measurement screen using a table
        #
        #
        # lv.table.STYLE.CELL1 = normal cell
        # lv.table.STYLE.CELL2 = header cell
        # lv.table.STYLE.CELL3 = ?
        # lv.table.STYLE.CELL4 = ?
        voltage_screen = lv.obj()
        
        # set background color, with no gradient
        screenstyle = lv.style_t(lv.style_plain)
        #screenstyle.body.main_color = lv.color_make(0xFF, 0xA5, 0x00)
        # 0xFF, 0x00, 0x00  Red
        # 0xC0, 0xC0, 0xC0  Silver
        # 0xFF, 0xA5, 0x00  Orange
        #screenstyle.body.grad_color = lv.color_make(0xFF, 0xA5, 0x00)
        screenstyle.body.border.color = lv.color_hex(0xe32a19)
        screenstyle.body.border.width = 5
        voltage_screen.set_style(screenstyle)
        
        tablestyle = lv.style_t(lv.style_plain)
        tablestyle.body.border.width = 0
        tablestyle.body.opa = 0
        
        cellstyle = lv.style_t(lv.style_plain)
        cellstyle.text.color = lv.color_hex(0xa028d4)
        cellstyle.text.font = lv.font_roboto_28
        cellstyle.body.padding.top = 1
        cellstyle.body.padding.bottom = 1
        cellstyle.body.border.width = 0
        cellstyle.body.opa = 0
        
        mtable = lv.table(voltage_screen)
        mtable.set_row_cnt(2)
        mtable.set_col_cnt(3)
        mtable.set_col_width(0, 110)
        mtable.set_col_width(1, 100)
        mtable.set_col_width(2, 100)
        mtable.set_style(lv.table.STYLE.BG, tablestyle)
        mtable.set_style(lv.table.STYLE.CELL1, cellstyle)
        
        mtable.set_cell_value(0,0, "V_BAT")
        mtable.set_cell_value(1,0, "V_USB")
        
        mtable.set_cell_value(0,2, "V")
        mtable.set_cell_value(1,2, "V")
        
        lv.scr_load(voltage_screen)
        self.backlight_ctrl.value(1)
        
        mtable.set_cell_value(0,1, '{:.2f}'.format(voltage_monitor.v_bat))
        mtable.set_cell_value(1,1, '{:.2f}'.format(voltage_monitor.v_usb))

    async def show_display_sleep_screen(self): 
        self.backlight_ctrl.value(0)

    async def show_audio_screen(self):  
        '''
        self.tft.clearwin()
        await asyncio.sleep(0)
        self.tft.text(0, 0, "Audio Screen", color=self.tft.CYAN)
        await asyncio.sleep(0)
        self.tft.text(0, 20, "... coming soon", color=self.tft.CYAN)
        '''
        pass
        
    async def show_diag_screen(self):  
        '''
        self.tft.clearwin()
        await asyncio.sleep(0)
        self.tft.text(0, 0, "Diag Screen ...", color=self.tft.CYAN)
        await asyncio.sleep(0)
        self.tft.text(0, 20, "Count = {}".format(self.diag_count), color=self.tft.CYAN)
        await asyncio.sleep(0)
        self.tft.text(0, 40, "Wifi: {}".format(mqtt.wifi_status), color=self.tft.CYAN) 
        '''
        pass       
        
class SDCardLogger():
    def __init__(self):
        pass
        
    async def run_logger(self):
        print('SDCardLogger:  opening file')
        s = open('/sd/samples.csv', 'a+')
        await asyncio.sleep(0)
        # wait until data for all sensors is available
        # write sensor data to the SD Card in CSV format
        numwrite = s.write('{}, {}, {}, {}, {}, {}, {:.1f}, {:.1f}, {:.1f}\n'.format(
                                                    sample_timestamp, 
                                                    ps.pm25_reading,
                                                    spec_sensors.ozone_v_gas,
                                                    spec_sensors.ozone_v_ref,
                                                    spec_sensors.no2_v_gas,
                                                    spec_sensors.no2_v_ref,
                                                    temp_hum.temp_degc,
                                                    temp_hum.humid_rh,
                                                    mic.dba))
        print('SDCardLogger:  wrote log')
        await asyncio.sleep(0)
        print('SDCardLogger:  s.close')
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
        self.feedname_dba = bytes('{:s}/feeds/{:s}'.format(b'MikeTeachman', b'dba'), 'utf-8')
        self.feedname_vbat = bytes('{:s}/feeds/{:s}'.format(b'MikeTeachman', b'vbat'), 'utf-8')
        
        self.wifi_status = 'unknown'
        
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
            self.wifi_status = 'on'
            self.client.resume()
            await self.client.publish(self.feedname_pm25, '{}'.format(ps.pm25_reading), qos = 0)
            await self.client.publish(self.feedname_o3, '{}'.format(spec_sensors.ozone_ppb), qos = 0)
            await self.client.publish(self.feedname_no2, '{}'.format(spec_sensors.no2_ppb), qos = 0)
            await self.client.publish(self.feedname_temp, '{:.2f}'.format(temp_hum.temp_degc), qos = 0)
            await self.client.publish(self.feedname_humidity, '{:.1f}'.format(temp_hum.humid_rh), qos = 0)
            await self.client.publish(self.feedname_dba, '{:.1f}'.format(mic.dba), qos = 0)
            await self.client.publish(self.feedname_vbat, '{:.2f}'.format(voltage_monitor.v_bat), qos = 0)
            
            # pausing the MQTT client will turn off the WiFi radio
            # which reduces the processor power usage
            print('turn WiFi off')
            self.wifi_status = 'off'
            self.client.pause()
            self.event_mqtt_publish.clear()
            
class Microphone():
    def __init__(self):
        self.dba = 0
        loop = asyncio.get_event_loop()
        loop.create_task(self.run_mic()) 
                
    async def run_mic(self):
        # dmacount range:  2 to 128 incl
        # dmalen range:   8 to 1024 incl
        
        bck_pin = Pin(13)
        ws_pin = Pin(12)
        sdin_pin = Pin(14)

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
        
        noise = dba.DBA(samples=10000, resolution=dba.B16, 
             coeffa=(1.0, -2.3604841 ,  0.83692802,  1.54849677, -0.96903429, -0.25092355,  0.1950274),
             coeffb=(0.61367941, -1.22735882, -0.61367941,  2.45471764, -0.61367941, -1.22735882,  0.61367941))
                
        print('Mic: opening WAV file')
        m=open('/sd/upy.wav','wb')
        wav_header = gen_wav_header(SAMPLES_PER_SECOND, BITS_PER_SAMPLE, 1,
                            SAMPLES_PER_SECOND * RECORD_TIME_IN_SECONDS)
        print('write WAV header')
        m.write(wav_header)
        numread = 0
        numwrite = 0
        bytes_in_dma_memory = 0
        overrun_count = 0
        sdwrite_over_100ms = 0
        sdwrite_50_to_100ms = 0
        sdwrite_count = 0
        dma_capacity = 64 * 256 * NUM_BYTES_RX  # dmacount*dmalen*8
        samples = bytearray(NUM_BYTES_IN_SAMPLE_BLOCK)
        sd_sector = bytearray(NUM_BYTES_IN_SDCARD_SECTOR)
        bytes_remaining_to_rx = NUM_SAMPLE_BYTES_TO_RX
        mic_file_open = True
        print('Mic recording start')
        while True:
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
                    
                    # copy sample from left channel and reduce sample resolution to 16 bits
                    num_copied = i2stools.copy(bufin=samples, bufout=sd_sector, channel=i2stools.LEFT, format=i2stools.B16)
                    
                    # feed samples to dBA calculation
                    res = noise.calc(sd_sector)
                    if (res != None):
                        # dba result ready
                        self.dba = res
                        #print("noise = {:.1f} dB(A)".format(self.dba))
                
                    # write samples to SD Card
                    if bytes_remaining_to_rx > 0:
                        start_sd_write = utime.ticks_us()
                        numwrite = m.write(sd_sector)
                        sd_write_time = utime.ticks_diff(utime.ticks_us(), start_sd_write)
                        sdwrite_count += 1
                        
                        #print('sd_write_time = ', sd_write_time)
                        if (sd_write_time) > 100*1000:  # 100ms
                            sdwrite_over_100ms += 1
                        elif (sd_write_time) > 50*1000:  # 50ms
                            sdwrite_50_to_100ms += 1
                            
                            end_ticks_us = utime.ticks_us()  
                            bytes_in_dma_memory += (utime.ticks_diff(end_ticks_us, start_ticks_us) * (SAMPLES_PER_SECOND * NUM_BYTES_RX)) // 1000000
                            if bytes_in_dma_memory > dma_capacity:
                                overrun_count += 1
                                #print('Mic:  DMA overrun!, count= ', overrun_count)
                    elif mic_file_open == True:
                        m.close()
                        print('================ Mic recording done ======================')
                        print('Mic Stats:  overrun_count: {}, sdwrite_50_to_100ms: {}, sdwrite_over_100ms: {}, sdwrite_count:  {}'.format(overrun_count, sdwrite_50_to_100ms, sdwrite_over_100ms, sdwrite_count))
                        #print('done mic. Percentage of zero I2S reads {:.1f}% '.format((1 - (count_sdwrite / count_i2sread)) * 100))
                        mic_file_open = False
                        
                    # fft TODO  coming soon .... 
                    '''
                    num_samples = 256
                    y = bytearray(num_samples * 4) # (sample / 2) freq bins, 2 floats per bin (real/imag)
                    fft.fft(sd_sector, y)
                    
                    print('fft results')
                    print(ustruct.unpack('>fffff', y)[0]) 
                    print(ustruct.unpack('>fffff', y)[1]) 
                    print(ustruct.unpack('>fffff', y)[2]) 
                    print(ustruct.unpack('>fffff', y)[3]) 
                    print(ustruct.unpack('>fffff', y)[4]) 
                    '''
            except Exception as e:
                m.close()
                audio.deinit()
  
class VoltageMonitor():
    NUM_READINGS = 10
    READING_PERIOD_MS = 100
    V_BAT_CALIBRATION = 0.001757
    V_USB_CALIBRATION = 0.001419    
    def __init__(self):
        self.v_bat = 0
        self.v_usb = 0
        self.vbat_pin = ADC(Pin(35))
        self.vbat_pin.atten(ADC.ATTN_11DB)
        self.vbat_pin.width(ADC.WIDTH_12BIT)
        self.vusb_pin = ADC(Pin(39))
        self.vusb_pin.atten(ADC.ATTN_11DB)
        self.vusb_pin.width(ADC.WIDTH_12BIT)
        loop = asyncio.get_event_loop()
        loop.create_task(self.run_v_monitor()) 
                
    async def run_v_monitor(self):
        v_bat_sample_sum = 0
        v_usb_sample_sum = 0
        
        while True:
            # take average of readings
            for _ in range(VoltageMonitor.NUM_READINGS):
                v_bat_sample_sum += self.vbat_pin.read()
                v_usb_sample_sum += self.vusb_pin.read()
                await asyncio.sleep_ms(VoltageMonitor.READING_PERIOD_MS)
                
            self.v_bat = v_bat_sample_sum * VoltageMonitor.V_BAT_CALIBRATION / VoltageMonitor.NUM_READINGS 
            self.v_usb = v_usb_sample_sum * VoltageMonitor.V_USB_CALIBRATION / VoltageMonitor.NUM_READINGS
            v_bat_sample_sum = 0
            v_usb_sample_sum = 0
#
#  TODO add User Interface, likely using setup screens driven by buttons
#

logging.basicConfig(level=logging.DEBUG)
i2c = I2C(scl=Pin(26), sda=Pin(27))
ds3231 = urtc.DS3231(i2c, address=0x68)
adc = ADS1219(i2c, address=0x41)
temp_humid_sensor = si7021.Si7021(i2c)

sample_timestamp = None  #  TODO implement without using a global
esp.osdebug(esp.LOG_ERROR)
pms5003.set_debug(False)
asyncio.set_debug(False)
asyncio.core.set_debug(False)

# slot=2 configures SD Card to use the SPI3 controller (VSPI), DMA channel = 2
# slot=3 configures SD Card to use the SPI2 controller (HSPI), DMA channel = 1
sd = SDCard(slot=3, sck=Pin(18), mosi=Pin(23), miso=Pin(19), cs=Pin(4))
# TODO figure out the root cause of the intermittent problem 0x109 error that happens on mounting the SD Card
utime.sleep(2) # workaround for 0x109 error (CRC error) that sometimes happens in the following mount call 
uos.mount(sd, "/sd")

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
voltage_monitor = VoltageMonitor()
loop.run_forever()