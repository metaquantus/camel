# Camel I2C Dual HX712 Weigh Scales Board

Camel is a board with Dual HX712 Weigh Scales ADCs controlled by a low power STM32L010F4 MCU providing an I2C slave interface.
Power is provided by an I2C Adafruit STEMMA compatible 4-pin JST PH (2mm pitch) connector. This board will work 
with power from 3.3V to 5V, but it's better if VCC > 4V. 

It was intended to support the integrated scales mod for the Gaggia Classic Pro espresso machine provided by the Gaggiuino project. But instead of the main MCU driving the HX712s directly, sensor reading and decoding is offloaded to this board's MCU and a simple I2C interface is used instead to connect to the host MCU. This reduces the number of wires required, frees some pins in the host MCU and saves some clock cycles.

STEMMA Pinouts:
```
1 Green for SCL
2 White for SDA
3 Red for VCC+
4 Black for GND
```

The MCU drives the 2 HX712 ADCs, reads and decodes the conversions and provides the raw data to the host via I2C. 
The HX712s allow programmatic configuration of the sampling rate (10Hz or 40Hz), the amplifier's gain
factor (128 or 256) and whether to put the HX712s into power down mode consuming less than 1uA.
The host is in charge of providing additional functions like scaling to weight units, tare and calibration.
I2C doesn't support slave initiated transfers so the host MCU must poll the device periodically to read the data.
There will be roughly one new sensor reading available of both load cells every 100ms if 10Hz sampling rate is used. 
The internal oscillator is used in both the HX712 ADCs and the MCU so sampling rates are not very precise.
Using the lower sampling rate has lower conversion noise. The HX712 is a 24-bit ADC and data is provided in 2's complement format.
Readings could be negative depending on how the load cell is mechanically setup, bending one way of the other.
For example, in some of the integrated scales mod assembles, one cell gives positive values and 
the other negative. To get the total weight reading, the absolute values of both cells should be added.

According to STM32CubeIDE, the MCU should consume about 5mA and the HX712s about 1mA each. 

If using only one of the HX712s, it's possible power down the other, saving 1mA. This can be configured from
the host.

Programming is done via a Serial Wire Debug (SWD) connector. Both a Tag-Connect TC2030-NL and a 4-pin 2.54mm pitch header are provided for SWD.

ESD protection diodes were added to the interface connectors.

2.54mm pitch header THT pads are used for the load cell connectors, but normally the load cells wires should be soldered directly to the pads instead of using headers or other connector.
