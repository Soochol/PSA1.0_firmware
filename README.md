# PSA1.0 ESP32-S3-WROOM-N32R16 (model pending)

ESP32 is connected to STM32 and a communications extension board (w/ TYPE1SC LTE modem + SAM-M10Q GNSS module)
- ESP32 is used for communications (BLE / LTE)
- ESP32 is used for AI inference for LMA activation (uses eloquentML)
- STM32 is used for collecting sensor data and main controls of all systems
- ESP32 - STM32 communicates via USART: all sensor data is sent to ESP32 for transmitting to AWS cloudDB and for eloquentML inference

## ASSUMPTIONS
It is assumed that the ESP32 has all the certificates for AWS access
- the device does not check if the modem is connected or not

## TODO
- update board specs on .ini
- functions: create ThingName & endpoints, data parsing (STM USART), data transmission (STM USART), LTE data parsing, BLE for calibrations, breathig rate detection
- to add: AI inference

## TO CHECK
- STM32 data - need to check format
- fall detection using IMUs (do we use the biosig board, or should we add one on the LTE board?)

## DATA STRUCTURES
STM USART DATA RECEIVE: every 0.1s
- raw: adsL,adsR, gyroLXYZ, gyroRXYZ
- stepCount
- battery

SAM-M10Q I2C: every 30s
- gpsLat, gpsLong