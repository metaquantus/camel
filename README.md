# Camel Dual HX711 Weigh Scales Board

Camel is a board with Dual HX711 Weigh Scales ADCs controlled by a low power STM32L010F4 MCU providing an I2C slave interface.
Power is provided by an I2C Adafruit STEMMA compatible 4-pin JST PH (2mm pitch) connector. For this board we require
5V power (STEMMA requires devices support 3-5V DC):
Pinout
`
1 Green for SCL
2 White for SDA
3 Red for V+
4 Black for GND
`

The MCU drives the 2 HX711 ADCs, reads the conversions and provides the data to the host via I2C. 
It provides functions to calibrate, tare and read the load cells.

