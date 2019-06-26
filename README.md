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

## Dependencies
### Hardware 
Lolin D32 Pro  
<https://wiki.wemos.cc/products:d32:d32_pro>
### Micropython
fork of Loboris ESP32  
<https://github.com/MikeTeachman/MicroPython_ESP32_psRAM_LoBo_I2S>
with customizations:
- I2S module:  https://github.com/miketeachman/micropython/commits/esp32-i2s
- support IOBase method from mainline MicroPython
- bitcrusher module: https://github.com/miketeachman/MicroPython_ESP32_psRAM_LoBo_I2S/commit/a40d3e033c1f7797e4ed1cad012ebeb0fb32cb42

### MicroPython Modules
asyn:    https://github.com/peterhinch/micropython-async/blob/master/asyn.py  
aswitch: https://github.com/peterhinch/micropython-async/blob/master/aswitch.py
uasyncio: https://github.com/peterhinch/micropython-async/tree/master/fast_io  
ms\_timer: https://github.com/peterhinch/micropython-async/blob/master/fast_io/ms_timer.py      
pms5003:  https://github.com/miketeachman/micropython-pms5003-minimal      
urtc:     https://github.com/adafruit/Adafruit-uRTC      
ads1219:  https://github.com/miketeachman/micropython-ads1219      
si7021:   https://github.com/chrisbalmer/micropython-si7021  
sh1106:   https://github.com/miketeachman/micropython-sh1106-lobo
 
## Detailed project documentation:
[Street Sense Project at Hackaday IO](https://hackaday.io/project/162059-street-sense)