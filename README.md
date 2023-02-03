# sourceRTK

This repo contains all the programs and tests made regarding the use of the F9P and the ESP32 cards. 

The final goal of these tests is to broadcast a precise RTK GNSS position via bluetooth so that our controler (RSPI or smartphone) can use it.

## I2C_scanner

This programm scans every I2C port of the ESP32 and displays if it detects a device the port that device is connected on.

## Example1_GetPositionAccuracy

Displays the position and the accuracy of the position of the F9P. 

*This program doesn't use RTK*

**The RTK's antena must be connected**

## Example15_NTRIPClient_custom

Connects to the Centipede network and queries the **CRO2** base for correction data.

*`secrets.h` must be modified if user wishes to connect to a new WIFI network*