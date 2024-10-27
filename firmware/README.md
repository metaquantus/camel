# Camel Weigh Scales Board Firmware

This MCU only has 16KB flash storage and 2KB RAM, so we're pretty tight.
In fact it doesn't fit in Debug mode.
In Release mode it takes about 10KB.
No space for USART code, maybe if we don't use the HAL libraries,
we can squeze some basic code for the USART, though not needed if the main
functions of reading the HX711 and providing the I2C driver are supported.


