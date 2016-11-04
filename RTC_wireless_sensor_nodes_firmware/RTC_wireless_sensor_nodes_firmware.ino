/*
  emonTH Low Power DHT22 Humidity & Temperature, DS18B20 Temperature & Pulse counting Node Example 

  Checkes at startup for presence of a DS18B20 temp sensor , DHT22 (temp + humidity) or both
  If it finds both sensors the temperature value will be taken from the DS18B20 (external) and DHT22 (internal) and humidity from DHT22
  If it finds only DS18B20 then no humidity value will be reported
  If it finds only a DHT22 then both temperature and humidity values will be obtained from this sesor
  
  Technical hardware documentation wiki: http://wiki.openenergymonitor.org/index.php?title=EmonTH
 
  Part of the openenergymonitor.org project
  Licence: GNU GPL V3
 
  Authors: Glyn Hudson
  Builds upon JCW JeeLabs RF12 library, Arduino and Martin Harizanov's work

  THIS SKETCH REQUIRES:

  Libraries in the standard arduino libraries folder:
  - JeeLib            https://github.com/jcw/jeelib
  - DHT22 Sensor Library  https://github.com/adafruit/DHT-sensor-library - be sure to rename the sketch folder to remove the '-'
  - OneWire library     http://www.pjrc.com/teensy/td_libs_OneWire.html
  - DallasTemperature     http://download.milesburton.com/Arduino/MaximTemperature/DallasTemperature_LATEST.zip

  Recommended node ID allocation
  -----------------------------------------------------------------------------------------------------------
  -ID-  -Node Type- 
  0 - Special allocation in JeeLib RFM12 driver - reserved for OOK use
  1-4     - Control nodes 
  5-10  - Energy monitoring nodes
  11-14 --Un-assigned --
  15-16 - Base Station & logging nodes
  17-30 - Environmental sensing nodes (temperature humidity etc.)
  31  - Special allocation in JeeLib RFM12 driver - Node31 can communicate with nodes on any network group
  -------------------------------------------------------------------------------------------------------------
  Change log:
  v2.1 - Branched from emonTH_DHT22_DS18B20 example, first version of pulse counting version
  v2.2 - 60s RF transmit period now uses timer1, pulse events are decoupled from RF transmit
  v2.3 - rebuilt based on low power pulse counting code by Eric Amann: http://openenergymonitor.org/emon/node/10834
  v2.4 - 5 min default transmisson time = 300 ms
  v2.3 - (12/10/14) don't flash LED on RF transmission to save power
  V2.4 - (15/10/15) activate pulse count pin input pullup to stop spurious pulses when no sensor connected
  V2.5 - (23/10/15) default nodeID 23 to enable new emonHub.conf decoder for pulseCount packet structure
  V2.6 - (24/10/15) Tweek RF transmission timmng to help reduce RF packet loss
  
emonhub.conf node decoder:
See: https://github.com/openenergymonitor/emonhub/blob/emon-pi/configuration.md

  [[23]]
    nodename = emonTH_5
    firmware = V2.x_RTC_wireless_sensor_nodes_firmware
    hardware = emonTH_(Node_ID_Switch_DIP1:OFF_DIP2:OFF)
    [[[rx]]]
       names = ambient_temp,ambient_humidity, coffee_beans_temperature, coffee_beans_humidity, battery, pulseCount
       datacodes = h,h,h,h,h,L
       scales = 0.1,0.1,0.1,0.1,0.1,1
       units = C,%,C,%,V,p
*/
const byte version = 26;         // firmware version divided by 10 e,g 16 = V1.6
                                                                      // These variables control the transmit timing of the emonTH
const unsigned long WDT_PERIOD = 80;                                  // mseconds.
const unsigned long WDT_MAX_NUMBER = 690;                             // Data sent after WDT_MAX_NUMBER periods of WDT_PERIOD ms without pulses:
                                                                      // 690x 80 = 55.2 seconds (it needs to be about 5s less than the record interval in emoncms)

const  unsigned long PULSE_MAX_NUMBER = 100;                          // Data sent after PULSE_MAX_NUMBER pulses
const  unsigned long PULSE_MAX_DURATION = 50;              
#define RF69_COMPAT 1                                                 // Set to 1 if using RFM69CW or 0 is using RFM12B
#include <JeeLib.h>                                                   // https://github.com/jcw/jeelib - Tested with JeeLib 3/11/14

boolean debug=1;                                                      // Set to 1 to few debug serial output, turning debug off increases battery life

#define RF_freq RF12_433MHZ                                           // Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
int nodeID = 23;                                                      // EmonTH temperature RFM12B node ID - should be unique on network
const int networkGroup = 210;                                         // EmonTH RFM12B wireless network group - needs to be same as emonBase and emonGLCD
//uint32_t networkGroup = 3421749817;                                                                    // oneWireSensor resolution 9,10,11 or 12bit corresponding to (0.5, 0.25, 0.125, 0.0625 degrees C LSB), 
                                                                      // lower resolution means lower power

const int TEMPERATURE_PRECISION=11;                                   // 9 (93.8ms),10 (187.5ms) ,11 (375ms) or 12 (750ms) bits equal to resplution of 0.5C, 0.25C, 0.125C and 0.0625C
#define ASYNC_DELAY 375                                               // 9bit requres 95ms, 10bit 187ms, 11bit 375ms and 12bit resolution takes 750ms
// See block comment above for library info
#include <avr/power.h>
#include <avr/sleep.h>                                           
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"
ISR(WDT_vect) { Sleepy::watchdogEvent(); }                            // Attached JeeLib sleep function to Atmega328 watchdog -enables MCU to be put into sleep mode inbetween readings to reduce power consumption 

// Hardwired emonTH pin allocations 
const byte activate_coffee_beans_sensor=    5;
const byte activate_ambient_sensor=      6;
const byte LED=            9;
const byte BATT_ADC=       1;
const byte DIP_switch1=    7;
const byte DIP_switch2=    8;
const byte pulse_countINT= 1;                                        // INT 1 / Dig 3 Screw Terminal Block Number 4 on emonTH V1.5 - Change to INT0 DIG2 on emonTH V1.4
const byte pulse_count_pin=3;                                        // INT 1 / Dig 3 Screw Terminal Block Number 4 on emonTH V1.5 - Change to INT0 DIG2 on emonTH V1.4
#define sensors_humidity_pin      19 //A5
#define sensors_temperature_pin   18   //A4
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(sensors_temperature_pin, DHTTYPE);
boolean DHT22_status;                                                 // create flag variable to store presence of oneWireSensor

OneWire oneWire(sensors_humidity_pin);
DallasTemperature sensors(&oneWire);
boolean oneWireSensor;                                                      // create flag variable to store presence of oneWireSensor 

// Note: Please update emonhub configuration guide on OEM wide packet structure change:
// https://github.com/openenergymonitor/emonhub/blob/emon-pi/configuration.md
typedef struct {                                                      // RFM12B RF payload datastructure
  int ambient_temp;
  int ambient_humidity;
  int coffee_beans_temperature;
  int coffee_beans_humidity;    
  int battery;
  unsigned long pulsecount;                                             
} Payload;
Payload emonth;

int numSensors; 
//addresses of sensors, MAX 4!!  
byte allAddress [4][8];                                              // 8 bytes per address

volatile unsigned long pulseCount;
unsigned long WDT_number;
boolean  p;

unsigned long now = 0;

//################################################################################################################################
//################################################################################################################################
void setup() {
//################################################################################################################################
  
  pinMode(LED,OUTPUT); digitalWrite(LED,HIGH);                       // Status LED on
     
  //READ DIP SWITCH POSITIONS - LOW when switched on (default off - pulled up high)
  pinMode(DIP_switch1, INPUT_PULLUP);
  pinMode(DIP_switch2, INPUT_PULLUP);
  boolean DIP1 = digitalRead(DIP_switch1);
  boolean DIP2 = digitalRead(DIP_switch2);
  
  if ((DIP1 == HIGH) && (DIP2 == HIGH)) nodeID=nodeID;
  if ((DIP1 == LOW) && (DIP2 == HIGH)) nodeID=nodeID+1;
  if ((DIP1 == HIGH) && (DIP2 == LOW)) nodeID=nodeID+2;
  if ((DIP1 == LOW) && (DIP2 == LOW)) nodeID=nodeID+3;
  
   rf12_initialize(nodeID, RF_freq, networkGroup);                       // Initialize RFM12B
  
  // Send RFM69CW test sequence (for factory testing)
  for (int i=10; i>-1; i--)                                         
  {
    emonth.coffee_beans_temperature=i; 
    rf12_sendNow(0, &emonth, sizeof emonth);
    delay(100);
  }
  rf12_sendWait(2);
  emonth.coffee_beans_temperature=0;
  // end of factory test sequence
  
  rf12_sleep(RF12_SLEEP);
  if (debug==1)
  {
    Serial.begin(9600);
    Serial.print(DIP1); Serial.println(DIP2);
    Serial.println("OpenEnergyMonitor.org");
    Serial.print("emonTH - Firmware V"); Serial.println(version*0.1); 
    #if (RF69_COMPAT)
      Serial.println("RFM69CW Init> ");
    #else
      Serial.println("RFM12B Init> ");
    #endif
    Serial.print("Node: "); 
    Serial.print(nodeID); 
    Serial.print(" Freq: "); 
    if (RF_freq == RF12_433MHZ) Serial.print("433Mhz");
    if (RF_freq == RF12_868MHZ) Serial.print("868Mhz");
    if (RF_freq == RF12_915MHZ) Serial.print("915Mhz"); 
    Serial.print(" Network: "); 
    Serial.println(networkGroup);
    delay(100);
  }
  
  pinMode(activate_ambient_sensor,OUTPUT);
  pinMode(activate_coffee_beans_sensor,OUTPUT);
  pinMode(BATT_ADC, INPUT);
  digitalWrite(activate_ambient_sensor,LOW);
  pinMode(pulse_count_pin, INPUT_PULLUP);

  //################################################################################################################################
  // Power Save  - turn off what we don't need - http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
  //################################################################################################################################
  ACSR |= (1 << ACD);                     // disable Analog comparator    
  if (debug==0) power_usart0_disable();   //disable serial UART
  power_twi_disable();                    //Disable the Two Wire Interface module.
  // power_timer0_disable();              //don't disable necessary for the oneWireSensor library
  power_timer1_disable();
  power_spi_disable();
 
  //################################################################################################################################
  // Test for presence of DHT22
  //################################################################################################################################
  digitalWrite(activate_ambient_sensor,HIGH);
  dodelay(2000);                                                        // wait 2s for DH22 to warm up
  dht.begin();
  float h = dht.readHumidity();                                         // Read Humidity
  float t = dht.readTemperature();                                      // Read Temperature
  digitalWrite(activate_ambient_sensor,LOW);                              // Power down
  
  if (isnan(t) || isnan(h))                                             // check if returns are valid, if they are NaN (not a number) then something went wrong!
  {
    Sleepy::loseSomeTime(1500); 
    float h = dht.readHumidity();  float t = dht.readTemperature();
    if (isnan(t) || isnan(h))   
    {
      if (debug==1) Serial.println("No DHT22 detected, but A4 is now going to be used by another analog sensor"); 
      DHT22_status=1;
    } 
  } 
  else 
  {
    DHT22_status=1;
    if (debug==1) Serial.println("Detected DHT22");  
  }   
 
  //################################################################################################################################
  // Setup and for presence of oneWireSensor
  //################################################################################################################################
  digitalWrite(activate_coffee_beans_sensor, HIGH); delay(50); 
  sensors.begin();
  sensors.setWaitForConversion(false);                             //disable automatic temperature conversion to reduce time spent awake, conversion will be implemented manually in sleeping http://harizanov.com/2013/07/optimizing-oneWireSensor-code-for-low-power-applications/ 
  numSensors=(sensors.getDeviceCount()); 
  
  byte j=0;                                        // search for one wire devices and
                                                   // copy to device address arrays.
  while ((j < numSensors) && (oneWire.search(allAddress[j])))  j++;
  digitalWrite(activate_coffee_beans_sensor, LOW);
  
  if (numSensors==0)
  {
    if (debug==1) Serial.println("No one wire sensor detected, No oneWireSensor");
    oneWireSensor=1; 
  } 
  else 
  {
    oneWireSensor=1; 
    if (debug==1) {
      Serial.print("Detected "); Serial.print(numSensors); Serial.println(" oneWireSensor");
       if (DHT22_status==1) Serial.println("oneWireSensor & DHT22 found, assume oneWireSensor is coffeeBeansSensor");
    }
  }
  if (debug==1) delay(200);
  
  //################################################################################################################################
  // Serial.print(oneWireSensor); Serial.print(DHT22_status);
  // if (debug==1) delay(200);
   
  digitalWrite(LED,LOW);
  
  emonth.pulsecount = 0;
  pulseCount = 0;
  WDT_number=720;
  p = 0;
  
  attachInterrupt(pulse_countINT, onPulse, RISING);
} // end of setup


//################################################################################################################################
//################################################################################################################################
void loop()
//################################################################################################################################
{ 
  
  if (p) {
    Sleepy::loseSomeTime(PULSE_MAX_DURATION);
    p=0;
  }
  
  if (Sleepy::loseSomeTime(WDT_PERIOD)==1) {
    WDT_number++;
  }
  
  if (WDT_number>=WDT_MAX_NUMBER || pulseCount>=PULSE_MAX_NUMBER) 
  {
    cli();
    emonth.pulsecount += (unsigned int) pulseCount;
    pulseCount = 0;
    sei();
   
    if (oneWireSensor==1)
    {
      digitalWrite(activate_coffee_beans_sensor, HIGH); dodelay(2000); 
      //for(int j=0;j<numSensors;j++) sensors.setResolution(allAddress[j], TEMPERATURE_PRECISION);      // and set the a to d conversion resolution of each.
      //sensors.requestTemperatures();                                        // Send the command to get temperatures
      dodelay(ASYNC_DELAY); //Must wait for conversion, since we use ASYNC mode
      //float temp=(sensors.getTempC(allAddress[0]));
      //##################calibrate this based on the Temp and Humidty Probe#############################
      //Humidity sensor; Linear Equation from sensor datasheet
      //Vout=26.65*RH+1006
      //RH=0.0375*Vout-37.7
      float coffee_beans_temperature=analogRead(sensors_temperature_pin);  //readings from A4
      float coffee_beans_humidity=analogRead(sensors_humidity_pin);  //readings from A5

      
      //##################################################################################################
      digitalWrite(activate_coffee_beans_sensor, LOW);
      if ((coffee_beans_temperature<1250.0) && (coffee_beans_temperature>-40.0))
      {
        if (DHT22_status==0) emonth.coffee_beans_temperature=(coffee_beans_temperature*10);            // if DHT22 is not present assume oneWireSensor is primary sensor (internal)
        if (DHT22_status==1) emonth.coffee_beans_temperature=(coffee_beans_temperature*10);   // if DHT22 is present assume oneWireSensor is coffeeBeansSensor sensor wired into terminal block
      }
      emonth.coffee_beans_humidity=coffee_beans_humidity*10;
    }
    
    if (DHT22_status==1)
    { 
      dodelay(2000);    
      digitalWrite(activate_ambient_sensor,HIGH);      //powered from D3                                                                                                 // Send the command to get temperatures
      dodelay(2000);                                             //sleep for 1.5 - 2's to allow sensor to warm up
      // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
      
      //##################################3calibrate this depending on the temp and Humidity we are going to use********************
      //Humidity sensor; Linear Equation from sensor datasheet
      //Vout=26.65*RH+1006
      //RH=0.0375*Vout-37.7
      emonth.ambient_humidity = analogRead(sensors_humidity_pin);  //data comming from A5
      float ambient_temp=(analogRead(sensors_temperature_pin)); //data comming from A4
      //####################################################################################################
      if ((ambient_temp<1250.0) && (ambient_temp>-40.0)) emonth.ambient_temp = (ambient_temp*10);

      digitalWrite(activate_ambient_sensor,LOW); 
    }
    
    emonth.battery=int(analogRead(BATT_ADC)*0.03225806);                    //read battery voltage, convert ADC to volts x10
       
    if (debug==1) 
    {
      if (oneWireSensor)
      {
        
        if (DHT22_status) 
        {
          Serial.print("Coffee Beans Temp: ");
          Serial.print(emonth.coffee_beans_temperature/10.0); 
          Serial.print("C, ");
          Serial.print(" Coffee Beans Humidity: ");
          Serial.print(emonth.coffee_beans_humidity/10.0); 
          Serial.print("%, ");
        }
        
        if (!DHT22_status) Serial.print(emonth.ambient_temp/10.0);  // use ambient temperature in case coffe beans sensor fails
        
      }
      
      if (DHT22_status)
      {
        Serial.print("Ambient Temperature: ");
        Serial.print(emonth.ambient_temp/10.0); 
        Serial.print("C, Ambient Humidity: ");
        Serial.print(emonth.ambient_humidity/10.0);
        Serial.print("%, ");
      }
      
      Serial.print("Battery voltage: ");  
      Serial.print(emonth.battery/10.0);
      Serial.print("V, Pulse count: ");
      Serial.print(emonth.pulsecount);
      Serial.println("n");
   
      unsigned long last = now;
      now = millis();   
      
      delay(100);
    }

    
    power_spi_enable();
   
    rf12_sleep(RF12_WAKEUP);
    dodelay(100);
    rf12_sendNow(0, &emonth, sizeof emonth);
    // set the sync mode to 2 if the fuses are still the Arduino default
    // mode 3 (full powerdown) can only be used with 258 CK startup fuses
    rf12_sendWait(2);
    rf12_sleep(RF12_SLEEP);
    dodelay(100);
    power_spi_disable();  
    //digitalWrite(LED,HIGH);
    //dodelay(100);
    //digitalWrite(LED,LOW);  
    
    WDT_number=0;
  }


} // end loop 

void dodelay(unsigned int ms)
{
  byte oldADCSRA=ADCSRA;
  byte oldADCSRB=ADCSRB;
  byte oldADMUX=ADMUX;
      
  Sleepy::loseSomeTime(ms); // JeeLabs power save function: enter low power mode for x seconds (valid range 16-65000 ms)
      
  ADCSRA=oldADCSRA;         // restore ADC state
  ADCSRB=oldADCSRB;
  ADMUX=oldADMUX;
}

// The interrupt routine - runs each time a rising edge of a pulse is detected
void onPulse()
{
  p=1;                                       // flag for new pulse set to true
  pulseCount++;                              // number of pulses since the last RF sent
}
