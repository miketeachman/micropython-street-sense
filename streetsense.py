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
import wavheader
from collections import namedtuple
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

LOGGING_INTERVAL_IN_SECS = 60*15

#  TODO clean this up...confusing, complicated
# I2S Microphone related config
SAMPLES_PER_SECOND = 10000
RECORD_TIME_IN_SECONDS = 60*60*4
NUM_BYTES_RX = 8
NUM_BYTES_USED = 2  # this one is especially bad  TODO:  refactor
BITS_PER_SAMPLE = NUM_BYTES_USED * 8
NUM_BYTES_IN_SDCARD_SECTOR = 512
NUM_BYTES_IN_SAMPLE_BLOCK = NUM_BYTES_IN_SDCARD_SECTOR * (NUM_BYTES_RX // NUM_BYTES_USED)
NUM_SAMPLE_BYTES_IN_WAV = (RECORD_TIME_IN_SECONDS * SAMPLES_PER_SECOND * NUM_BYTES_USED)
NUM_SAMPLE_BYTES_TO_RX = ((RECORD_TIME_IN_SECONDS * SAMPLES_PER_SECOND * NUM_BYTES_RX))

PM25_POLLING_DELAY_MS = 500

#
# measurement repo
#
# - stores measurements
# - latest
# - stats: min, max
# - timestamps
#
class MeasurementRepo():
    Measurement = namedtuple('Measurement', 'current min max')
    
    def __init__(self):
        self.measurement_repo = {}

    def add(self, measurement, value):
        if not measurement in self.measurement_repo:
            self.measurement_repo[measurement] = MeasurementRepo.Measurement(current=None, min=None, max=None)
    
        min =  self.measurement_repo[measurement].min
        if min is None or value < min:
            min = value
            
        max =  self.measurement_repo[measurement].max  
        if max is None or value > max:
            max = value
            
        self.measurement_repo[measurement] = MeasurementRepo.Measurement(current=value, min=min, max=max)
            
    def get(self, measurement):
        if measurement in self.measurement_repo:
            return  self.measurement_repo[measurement]
        else:
            return MeasurementRepo.Measurement(current=0, min=0, max=0)  
        
    def clear_stats(self, measurement):    
        if measurement in self.measurement_repo:
            value =  self.measurement_repo[measurement].current
        self.measurement_repo[measurement] = MeasurementRepo.Measurement(current=value, min=None, max=None)

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
        
    def callback(self, arg):
        if self.sample_count < self.SAMPLES_TO_CAPTURE:
            self.sample_sum += adc.read_data_irq()
            self.sample_count += 1
            # re-enable interrupt
            #self.drdy_pin.init(trigger=Pin.IRQ_LOLEVEL)
            self.drdy_pin.irq(trigger=Pin.IRQ_FALLING, handler=self.callback)
        
    async def read(self, adc_channel):
        log.info('read adc_channel= %d', adc_channel)
        self.sample_sum= 0 
        self.sample_count = 0     
        adc.set_channel(adc_channel)
        adc.set_conversion_mode(ADS1219.CM_CONTINUOUS)
        adc.set_gain(ADS1219.GAIN_1X)
        adc.set_data_rate(ADS1219.DR_20_SPS)
        adc.set_vref(ADS1219.VREF_INTERNAL)
        adc.start_sync() # starts continuous sampling
        
        await asyncio.sleep(1)
        start_capture = utime.ticks_ms()
        self.drdy_pin.irq(trigger=Pin.IRQ_FALLING, handler=self.callback)
        
        while self.sample_count < self.SAMPLES_TO_CAPTURE:
            await asyncio.sleep_ms(10)

        self.drdy_pin.init()  # TODO - will this disable interrupt on pin ??
        log.debug('  done.  conversion time = %d', utime.ticks_diff(utime.ticks_ms(), start_capture))
        adc.set_conversion_mode(ADS1219.CM_SINGLE)
        
        avg_mv = self.sample_sum*2.048*1000/(2**23)/self.sample_count
        log.debug('  avg_mv = %d', avg_mv)
        
        return avg_mv
    
    async def read_all(self):
        # read Ozone gas voltage
        repo.add('o3_vgas', await self.read(ADS1219.CHANNEL_AIN1))        
        # read Ozone reference voltage
        repo.add('o3_vref', await self.read(ADS1219.CHANNEL_AIN0))       
        # read NO2 gas voltage
        repo.add('no2_vgas', await self.read(ADS1219.CHANNEL_AIN2))        
        # read NO2 reference voltage
        repo.add('no2_vref', await self.read(ADS1219.CHANNEL_AIN3))      
        
        # calculate gas concentration in parts-per-billion (ppb)
        # TODO calibrate Spec Sensors, with offset
        ozone_ppb = (repo.get('o3_vgas').current - repo.get('o3_vref').current) / self.CALIBRATION_FACTOR_OZONE
        repo.add('o3', ozone_ppb)
        no2_ppb = (repo.get('no2_vgas').current - repo.get('no2_vref').current) / self.CALIBRATION_FACTOR_NO2
        repo.add('no2', no2_ppb)

# TODO does this class make sense anymore ?        
class THSensor():
    def __init__(self):
        pass
        
    async def read(self):
        repo.add('tdegc', temp_humid_sensor.temperature)
        repo.add('rh', temp_humid_sensor.relative_humidity)

class ParticulateSensor():
    def __init__(self, 
                 lock, 
                 event_new_pm25_data):
        self.lock = lock
        self.event_new_pm25_data = event_new_pm25_data
        self.pm25_pwr_pin = Pin(25, Pin.OUT)
        self.pm25_pwr_pin.value(0)
        
    async def read_pm25(self):
        log.info('PM:  30s warm-up')
        self.pm25_pwr_pin.value(1)
        await asyncio.sleep(30) # 30s warm-up period as specified in datasheet
        self.uart = UART(1, tx=32, rx=33, baudrate=9600)
        self.pm25 = pms5003.PMS5003(self.uart, self.lock, event = self.event_new_pm25_data)
        await asyncio.sleep(1)
        log.debug('PM:  set Passive mode')
        await self.pm25.setPassiveMode()
        log.debug('PM:  trigger read sensor')
        await asyncio.sleep(1)
        await self.pm25.read()
        log.debug('PM:  waiting for event')
        await self.event_new_pm25_data
        log.debug('PM:  got event')
        repo.add('pm25', await ps.get_value())
        log.info('PM:  PM2.5 = %d', repo.get('pm25').current)
        self.event_new_pm25_data.clear() 
        Pin(32, Pin.IN, Pin.PULL_DOWN)
        Pin(33, Pin.IN, Pin.PULL_DOWN)
        log.info('PM:  power-down')
        self.pm25_pwr_pin.value(0)
        
    # TODO method needed anymore ?    
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
            log.info('next sensor read at %s', wake_time_list)
            log.info('waiting for DS3231 alarm')
            # loop until the DS3231 alarm is detected 
            while ds3231.alarm(alarm=0) == False:
                await asyncio.sleep_ms(250)

            sample_timestamp = urtc.tuple2seconds(ds3231.datetime())
            # clear alarm    
            ds3231.alarm(False, alarm=0)
            log.info('DS3231 alarm -> read all sensors')
            # dispatch a whole pile of activity
            # following sequence is deliberately sequential so PM2.5 sensor is powered off when
            # the Spec Sensor devices are being read
            # first clear all the measurement statistics
            await ps.read_pm25()
            await spec_sensors.read_all()
            await temp_hum.read()
            await sdcard_logger.run_logger()
            self.event_mqtt_publish.set()

class Display():
    def __init__(self):
        self.screens = [self.show_measurement_screen, 
                        self.show_voltage_monitor_screen,
                        self.show_decibel_screen, 
                        self.show_display_sleep_screen]
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
        # Workaround note:  need to initialize the SPI bus before initializing LittlevGL
        # SPI bus init was taken out of LittlevGL for streetsense design...should fix this.  TODO
        # SPI bus initialization is done when the SD Card is initialized 
        # (must be called before display initialization) 
        disp = ili.display(spihost=1, miso=19, mosi=23, clk=18, cs=22, dc=21, rst=15, backlight=2, mhz=25)
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
            await asyncio.sleep(1)  # TODO idea:  each screen might have a configurable update time                  
        
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
        
        mtable.set_cell_value(0,1, '{:.1f}'.format(repo.get('no2').current))
        mtable.set_cell_value(1,1, '{:.1f}'.format(repo.get('o3').current))
        mtable.set_cell_value(2,1, '{:.1f}'.format(repo.get('pm25').current))
        mtable.set_cell_value(3,1, '{:.1f}'.format(repo.get('tdegc').current))
        mtable.set_cell_value(4,1, '{:.1f}'.format(repo.get('rh').current))
        mtable.set_cell_value(5,1, '{:.1f}'.format(repo.get('dba').current))

        lv.scr_load(measurement_screen)
        self.backlight_ctrl.value(1)

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
        
        mtable.set_cell_value(0,1, '{:.2f}'.format(repo.get('vbat').current))
        mtable.set_cell_value(1,1, '{:.2f}'.format(repo.get('vusb').current))
        
        lv.scr_load(voltage_screen)
        self.backlight_ctrl.value(1)
        
    async def show_display_sleep_screen(self): 
        self.backlight_ctrl.value(0)

    async def show_decibel_screen(self):  
        # 
        # Decibel screen
        #
        decibel_screen = lv.obj()
        
        # set background color, with no gradient
        screenstyle = lv.style_t(lv.style_plain)
        screenstyle.body.main_color = lv.color_hex(0xffffff)
        # 0xFF, 0x00, 0x00  Red
        # 0xC0, 0xC0, 0xC0  Silver
        # 0xFF, 0xA5, 0x00  Orange
        screenstyle.body.grad_color = lv.color_hex(0xffffff)
        screenstyle.body.border.color = lv.color_hex(0xe32a19)
        screenstyle.body.border.width = 5
        screenstyle.text.color = lv.color_hex(0xa028d4)
        screenstyle.text.font = lv.font_roboto_120
        decibel_screen.set_style(screenstyle)
        
        reading = lv.label(decibel_screen)
        reading.set_x(40)
        reading.set_y(70)
        reading.set_text('{:.1f}'.format(repo.get('dba').current))
        
        unit = lv.label(decibel_screen)
        unitstyle = lv.style_t(lv.style_plain)
        unitstyle.text.color = lv.color_hex(0xa028d4)
        unitstyle.text.font = lv.font_roboto_28
        unit.set_style(lv.label.STYLE.MAIN, unitstyle)
        unit.set_x(215)
        unit.set_y(170)
        unit.set_text("dBA")
        
        lv.scr_load(decibel_screen)        
        self.backlight_ctrl.value(1)
        
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
        log.info('SDCardLogger:  opening file')
        s = open('/sd/samples.csv', 'a+')
        await asyncio.sleep(0)
        # wait until data for all sensors is available
        # write sensor data to the SD Card in CSV format
        numwrite = s.write('{}, {}, {}, {}, {}, {}, {:.1f}, {:.1f}, {:.1f}\n'.format(
                                                    sample_timestamp, 
                                                    repo.get('pm25').current,
                                                    repo.get('o3_vgas').current,
                                                    repo.get('o3_vref').current,
                                                    repo.get('no2_vgas').current,
                                                    repo.get('no2_vref').current,
                                                    repo.get('tdegc').current,
                                                    repo.get('rh').current,
                                                    repo.get('dba').current))
        log.info('SDCardLogger:  wrote log')
        await asyncio.sleep(0)
        log.info('SDCardLogger:  s.close')
        s.close()
        await asyncio.sleep(0)

class MQTTPublish():
    def __init__(self, event_mqtt_publish):    
        self.event_mqtt_publish = event_mqtt_publish    
        self.feedname_pm25 = bytes('{:s}/feeds/{:s}'.format(b'MikeTeachman', b'PM25'), 'utf-8')
        self.feedname_o3 = bytes('{:s}/feeds/{:s}'.format(b'MikeTeachman', b'o3'), 'utf-8')
        self.feedname_no2 = bytes('{:s}/feeds/{:s}'.format(b'MikeTeachman', b'no2'), 'utf-8')
        self.feedname_temp = bytes('{:s}/feeds/{:s}'.format(b'MikeTeachman', b'TdegC'), 'utf-8')
        self.feedname_humidity = bytes('{:s}/feeds/{:s}'.format(b'MikeTeachman', b'humidity'), 'utf-8')
        self.feedname_dba = bytes('{:s}/feeds/{:s}'.format(b'MikeTeachman', b'dba'), 'utf-8')
        self.feedname_dba_max = bytes('{:s}/feeds/{:s}'.format(b'MikeTeachman', b'dba_max'), 'utf-8')
        self.feedname_vbat = bytes('{:s}/feeds/{:s}'.format(b'MikeTeachman', b'vbat'), 'utf-8')
        self.feedname_vbat_min = bytes('{:s}/feeds/{:s}'.format(b'MikeTeachman', b'vbat_min'), 'utf-8')
        
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
            log.info('turn WiFi on')
            self.wifi_status = 'on'
            self.client.resume()
            await self.client.publish(self.feedname_pm25, '{}'.format(repo.get('pm25').current), qos = 0)
            await self.client.publish(self.feedname_o3, '{}'.format(repo.get('o3').current), qos = 0)
            await self.client.publish(self.feedname_no2, '{}'.format(repo.get('no2').current), qos = 0)
            await self.client.publish(self.feedname_temp, '{:.2f}'.format(repo.get('tdegc').current), qos = 0)
            await self.client.publish(self.feedname_humidity, '{:.1f}'.format(repo.get('rh').current), qos = 0)
            await self.client.publish(self.feedname_dba, '{:.1f}'.format(repo.get('dba').current), qos = 0)
            await self.client.publish(self.feedname_dba_max, '{:.1f}'.format(repo.get('dba').max), qos = 0)
            await self.client.publish(self.feedname_vbat, '{:.2f}'.format(repo.get('vbat').current), qos = 0)
            await self.client.publish(self.feedname_vbat_min, '{:.2f}'.format(repo.get('vbat').min), qos = 0)
            
            # pausing the MQTT client will turn off the WiFi radio
            # which reduces the processor power usage
            log.info('turn WiFi off')
            self.wifi_status = 'off'
            self.client.pause()
            self.event_mqtt_publish.clear()
            
            # TODO need a better place to perform measurement stat clearing (another event sync object?)
            repo.clear_stats('o3')
            repo.clear_stats('no2')
            repo.clear_stats('pm25')
            repo.clear_stats('tdegc')
            repo.clear_stats('rh')
            repo.clear_stats('dba')
            repo.clear_stats('vbat')
            
class Microphone():
    def __init__(self):
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
                
        logmic.info('opening WAV file')
        m=open('/sd/upy.wav','wb')
        wav_header = wavheader.gen_wav_header(SAMPLES_PER_SECOND, BITS_PER_SAMPLE, 1,
                            SAMPLES_PER_SECOND * RECORD_TIME_IN_SECONDS)
        logmic.debug('write WAV header')
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
        logmic.info('recording start')
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
                        repo.add('dba', res)
                        logmic.debug("noise = {:.1f} dB(A)".format(repo.get('dba').current))
                
                    # write samples to SD Card
                    if bytes_remaining_to_rx > 0:
                        start_sd_write = utime.ticks_us()
                        numwrite = m.write(sd_sector)
                        sd_write_time = utime.ticks_diff(utime.ticks_us(), start_sd_write)
                        sdwrite_count += 1
                        
                        logmic.debug('sd_write_time = %d', sd_write_time)
                        if (sd_write_time) > 100*1000:  # 100ms
                            sdwrite_over_100ms += 1
                        elif (sd_write_time) > 50*1000:  # 50ms
                            sdwrite_50_to_100ms += 1
                            
                            end_ticks_us = utime.ticks_us()  
                            bytes_in_dma_memory += (utime.ticks_diff(end_ticks_us, start_ticks_us) * (SAMPLES_PER_SECOND * NUM_BYTES_RX)) // 1000000
                            if bytes_in_dma_memory > dma_capacity:
                                overrun_count += 1
                                logmic.debug('Mic:  DMA overrun!, count= ', overrun_count)
                    elif mic_file_open == True:
                        m.close()
                        logmic.info('recording done')
                        logmic.info('Stats:\n  overrun_count: {}\n'
                                    '  sdwrite_50_to_100ms: {}\n' 
                                    '  sdwrite_over_100ms: {}\n' 
                                    '  sdwrite_count:  {}\n'.format(overrun_count, 
                                                                     sdwrite_50_to_100ms, 
                                                                     sdwrite_over_100ms, 
                                                                     sdwrite_count))
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
                
            repo.add('vbat', v_bat_sample_sum * VoltageMonitor.V_BAT_CALIBRATION / VoltageMonitor.NUM_READINGS)
            repo.add('vusb', v_usb_sample_sum * VoltageMonitor.V_USB_CALIBRATION / VoltageMonitor.NUM_READINGS)
            
            v_bat_sample_sum = 0
            v_usb_sample_sum = 0
#
#  TODO add User Interface, likely using setup screens driven by buttons
#

# streetsense debugging uses logging module in micropython-lib
# levels:  debug, info, warning, error, critical
logging.basicConfig(level=logging.INFO)
log = logging.getLogger('streetsense')  # general purpose debugging
log.setLevel(logging.INFO)
logmic = logging.getLogger('streetsense:mic') # microphone specific debug subsystem
logmic.setLevel(logging.INFO)

# debugging for other modules
esp.osdebug(esp.LOG_ERROR)
pms5003.set_debug(False)
asyncio.set_debug(False)
asyncio.core.set_debug(False)
MQTTClient.DEBUG = False

i2c = I2C(scl=Pin(26), sda=Pin(27))
ds3231 = urtc.DS3231(i2c, address=0x68)
adc = ADS1219(i2c, address=0x41)
temp_humid_sensor = si7021.Si7021(i2c)

sample_timestamp = None  #  TODO implement without using a global

# all measurements are stored and retreived to/from 
# a centralized repo
repo = MeasurementRepo()

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