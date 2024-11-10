# Camel I2C Dual HX712 Weigh Scales Board

Camel is a board with Dual HX712 Weigh Scales ADCs controlled by a low power STM32L010F4 MCU providing an I2C slave interface.
Power is provided by an I2C Adafruit STEMMA compatible 4-pin JST PH (2mm pitch) connector. For this board we require
5V power, so it's not exactly STEMMA compatible, as STEMMA requires supports for the range 3-5V.

Pinout
```
1 Green for SCL
2 White for SDA
3 Red for V+
4 Black for GND
```

The MCU drives the 2 HX712 ADCs, reads the conversions and provides the raw data to the host via I2C. 
The host is in charge of providing functions like scaling to weight units, tare and calibration.

According to STM32CubeIDE, the MCU should consume about 5mA and the HX712s about 1mA each. 

It's possible to powerdown one of the HX712s if not needed, saving 1mA. This can be configured from
the host.


