# emonTH - Wireless Temperature and Humidity monitoring node 
//##################################################################################################################
# RTC wireless sensor nodes for Shyara
Shyara wet mill station has six wireless sensor nodes:
•	3 WSN for Temperature and Humidity
•	3 WSN for Fermentation tank (PH and Alcohol)
All of these WSN are based on TH boards,
For Temp and Humidity WSN, sensors are wired in such way it share the same Analog input pin which means two sensing value are taken from one Analog pin. The TH has surface mounted 10k resitor at A4 and we are using it for temperature measurement as the temperature sensor we are using ahs normal 10K thermistor.
In Summary, The two sensor has four wires which are distributed as follow:
•	Ambient Temperature and Coffee beans Humidity are taken from Analog pin 5, DIL 10K resistor is added here.
•	Ambient Humidity and Coffee beans Temperature are taken from Analog pin 4, Surface mounted resistor used here.

The fermentation tank WSN is not self-powered, it gets powered from solar system installed on the site.
It has two sensor: PH (https://cdn.sparkfun.com/datasheets/Sensors/Biometric/pH_EZO_datasheet_v13.pdf)  and Ethanol (https://www.vernier.com/files/manuals/eth-bta.pdf) sensors.
The ethanol sensor does not need calibration, it need only to replace the plumber ‘tape on the ethanol cap before starting the new measurement for accuracy readings.
The PH sensor need to be calibrated atleast once year, the steps to follow is straight forward but you will need approximately 5 min for this per each fermentation wireless sensor node; The PH sensor used has two type of data protocol, we used the UART one due to availability of the free pins on our TH.
Calibration process is in calibration sketch.

//##################################################################################################################


Part of the openenergymonitor.org project

Main emonTH page: http://openenergymonitor.org/emon/modules/emonTH

Technical Hardware Wiki: http://wiki.openenergymonitor.org/index.php?title=EmonTH 

Schematic + CAD: http://solderpad.com/git/openenergymon/emonth

Design & related Blog posts: 

http://openenergymonitor.blogspot.com/2013/06/emonth-prototype.html
http://openenergymonitor.blogspot.com/2013/10/emonth-update-hardware.html
http://openenergymonitor.blogspot.com/2013/10/emonth-update-software-power.html
http://openenergymonitor.blogspot.com/2013/10/aa-battery-considerations.html
http://openenergymonitor.blogspot.com/2013/11/hardware-manufacture-begins-part-1.html
http://openenergymonitor.blogspot.com/2014/01/emonth-multiple-ds18b20-sensors.html

Builds on JeeLabs, Adafruit and Miles Burton 

## Libraries Needed
* JeeLib: https://github.com/jcw/jeelib (CURRENT emonTH V1.5)
* RFu_JeeLib: https://github.com/openenergymonitor/RFu_jeelib (OLD emonTH V1.4)
* Temperature control library: http://download.milesburton.com/Arduino/MaximTemperature/ (version 372 works with Arduino 1.0) and OneWire library: http://www.pjrc.com/teensy/td_libs_OneWire.html
* DHT22 Sensor Library  https://github.com/adafruit/DHT-sensor-library - be sure to rename the sketch folder to remove the '-'


## emonTH Firmwarwe

**emonTH_DHT22_DS18B20_RFM69CW_Pulse**  Current main emonTH temperature and humidity sensing firmware (Nov2015). Searches for either DHT22 or DS18B20 and reads temperature and humidity once per min (by default) and tx's data back to the emonBase via RFM69CW. If both sensors are detected temperature will be sensed from DS18B20 and humidity from DHT22. Supports on-board RF nodeID setting via DIP switch selectors. Now supports optical counting sensor. See Wiki for more details http://wiki.openenergymonitor.org/index.php/EmonTH_V1.5

* **emonTH_DHT22_DS18B20_RFM69CW** - FOR emonTH V1.5+: Searches for either DHT22 or DS18B20 and reads temperature and humidity once per min (by default) and tx's data back to the emonBase via RFM69CW. If both sensors are detected temperature will be sensed from DS18B20 and humidity from DHT22. Supports on-board node ID DIP switch selectors 

* **emonTH_DHT22_DS18B20** - FOR emonTH V1.4 - emonTH temperature and humidity sensing firmware. Searches for either DHT22 or DS18B20 and reads temperature and humidity once per min (by default) and tx's data back to the emonBase via RFM12B. If both sensors are detected temperature will be sensed from DS18B20 and humidity from DHT22 

* **emonTH_DHT22_dual_DS18B20** - Derived from the main emonTH firmware, but capable of monitoring two (or more) DS18B20 external sensors. You'll need to discover your sensors' addresses to make use of this script - discover them with 'emonTH temperature search' utility sketch in 'Simple emonTH Sensor Test' folder

* **emonTH_DHT22_multiple_DS18B20** - Derived from the dual sensor emonTH firmware by Marshall Scholz. Capable of automatically discovering and monitoring up to 60 connected DS18B20 sensors, one DHT22/DHT11, and one analog pin. The downfalls of this version are that it uses slightly more power than the one sensor sketch, and that the sensor order will probably change if an extra sensor is added once the node has been set up. (This can be easily rectified by changing the input logging feed in emonCMS)

* **Simple emonTH Sensor Test** - 
	* emonTH DHT22 Test 
	* emonTH DS18B20 Test
	* emonTH temperature search - utility sketch for finding hardware addresses of one or more DS18B20 sensors connected to emonTH one-wire bus - The DallasTemperature library's "tester" sketch may do a better job of this

* **emonTH_DHT22_DS18B20_RFM69CW_REEDSWITCH**

Low-power sketch for EmonTH V1.5 that counts pulses from a reed switch with debouncing. It aso sends the temperature/humidity every minute. By [Eirc_AMANN](https://openenergymonitor.org/emon/user/5027) March 2016
[Forum thread development](https://openenergymonitor.org/emon/node/12165)

**Note:**
* Default RFM12B/RFM69CW settings: 433Mhz, network: 210, Node: 19 
* Readings are converted to integer when sent over RF multiple by 0.1 in emoncms to restore reading
* As the JeeLib library sends out packets in individual bytes, 16 bit integers are split into two received values according to Arduino's "little endian" topology

# License
The emonTH hardware designs (schematics and CAD files hosted on http://solderpad.com/openenergymon) are licensed under a Creative Commons Attribution-ShareAlike 3.0 Unported License.

The emonTH firmware is released under the GNU GPL V3 license

The documentation is subject to GNU Free Documentation License 

The emonTH hardware designs follow the terms of the OSHW (Open-source hardware) Statement of Principles 1.0.






 
