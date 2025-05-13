# Car Diagnostic Deamon
The Python file is installed on the Raspberry Pi, which collects information on a connected car and sends it to an MQTT server.
The collection points are divided into modules, which are the follows:
## GPS module:
 A Sim7600e Module is connected to a Raspberry Pi to connect the Pi to the internet via cellular and additionally, the module collects the GPS data.
 The modem works in RDNIS mode; therefore, GPS data is collected from the NMEA port of the modem.
## GPIO module:
  The module initializes the internal GPIO module of the Raspberry Pi. The initial code is written but this module is not needed currently.
  Thus, the code is an immature level.
## System module:
  The system module collects the data from the Raspberry Pi rather than the external sensors. It is purely for debugging purposes.
## I2C module:
  An OLED screen is connected to the Raspberry Pi to use for diagnostics. The logging library is directed to the OLED screen, therefore, it is possible to get information in a remote location without the need for a screen and keyboard.
  Additionally, INA219 current meters. The INA219 is used to do Voltage and Current measurements around the car. It is intended to be used in areas where OBD-II connection does not provide the information.
## OBD module:
  An USB OBD-II module is connected to the Raspberry.
  OBD library is used to provide general information about a car.
  This module is still under development.
