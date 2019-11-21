# Street Sense

Street Sense is a MicroPython based project to build a portable, battery-powered sensor unit to measure: 

- **Air Quality**
    - Ozone
    - NO2 
    - Particulates 
- **Noise Pollution**
- **Environment**
    - Temperature
    - Humidity

## Detailed project documentation:
[Street Sense Project at Hackaday IO](https://hackaday.io/project/162059-street-sense)

## Dependencies
### Hardware 
Lolin D32 Pro  
<https://wiki.wemos.cc/products:d32:d32_pro>
### Micropython
fork of micropython
https://github.com/miketeachman/micropython/tree/esp32-i2s-streetsense-littlevgl
which includes these customizations:
- I2S module to read audio samples from an I2S microphone. [link](https://github.com/miketeachman/micropython/blob/esp32-i2s-streetsense-littlevgl/ports/esp32/machine_i2s.c)
- dba module for calculating dB(A) from a stream of audio samples. [link](https://github.com/miketeachman/micropython/blob/esp32-i2s-streetsense-littlevgl/ports/esp32/moddba.c)  
- i2stools module for efficient copy/pruning of audio samples. [link](https://github.com/miketeachman/micropython/blob/esp32-i2s-streetsense-littlevgl/ports/esp32/modi2stools.c)
- integration of littlevGL graphics library. [link](https://github.com/miketeachman/lv_binding_micropython/tree/lvgl-streetsense)

### MicroPython Modules

| name|link|
|----    | ----- |
|asyn    | https://github.com/peterhinch/micropython-async/blob/master/asyn.py  |
|aswitch | https://github.com/peterhinch/micropython-async/blob/master/aswitch.py |
|uasyncio (fast\_io) | https://github.com/peterhinch/micropython-async/tree/master/fast_io  |
|ms\_timer| https://github.com/peterhinch/micropython-async/blob/master/fast_io/ms_timer.py      |
|pms5003: | https://github.com/miketeachman/micropython-pms5003-minimal   |   
|urtc:    | https://github.com/adafruit/Adafruit-uRTC      |
|ads1219: | https://github.com/miketeachman/micropython-ads1219|
|si7021:  | https://github.com/chrisbalmer/micropython-si7021|
|mqtt\_as:|  https://github.com/peterhinch/micropython-async/blob/master/fast_io/ms_timer.py|
|logging: | https://github.com/micropython/micropython-lib/tree/master/logging|


### MicroPython Libs
fork of littlevgl with customizations
https://github.com/miketeachman/lv_binding_micropython/commits/lvgl-streetsense
 
