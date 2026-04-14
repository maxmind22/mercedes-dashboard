This is a project for controling alternator in order to use LiFePO4 battery, read engine temp, fuel level, vehicle speed ... to LCD display using composite video on ESP32 and arduino working together to accompish different tasks 

the arduino controls engine + A/C cooling fan and DFCO (deceleration fuel cut off) at 1500- 1100 rpm
the ESP32 get data from arduino (via CAN) and display them on LCD using composite video.
ESP32 reads fuel level, low coolant level, low engine oil switch, and provides warnings including engine overheat warning.
ESP32 also controls alternator field to regulate charging current and voltage going into the battery as I'm using LiFePO4 battery (about 154 AH)
