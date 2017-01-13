//from http://playground.arduino.cc/ComponentLib/Thermistor2
#include <math.h>                                               
#define RF69_COMPAT 1                                                 // Set to 1 if using RFM69CW or 0 is using RFM12B
#include <JeeLib.h>                                                   // https://github.com/jcw/jeelib - Tested with JeeLib 3/11/14

boolean debug=1;                                       //Set to 1 to few debug serial output, turning debug off increases battery life

#define RF_freq RF12_433MHZ                 // Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
int nodeID = 30;                               // EmonTH temperature RFM12B node ID - should be unique on network
const int networkGroup = 210;                // EmonTH RFM12B wireless network group - needs to be same as emonBase and emonGLCD
//uint32_t networkGroup = 3421749817;    //numeric value from ansible rmc_key
                                                                      

const int time_between_readings= 1;                                   // in minutes
#include <avr/power.h>
#include <avr/sleep.h>                            

float DEFAULT_VCC=5.0;
float INPUT_VCC=3.3;
float AMBIENT_RESISTOR=9820.0;
float MAX_ADC=1024.0;

ISR(WDT_vect) { Sleepy::watchdogEvent(); } 

double temp;
double temp_ambient;
float VOLTAGE;
float AMBIENT_HUMID_V,AMBIENT_HUMID_ADC,COFFEE_HUMID_V,COFFEE_HUMID_ADC,VOLTAGE_RATIO;
float AMBIENT_HUMID, COFFEE_HUMID;
  
const int COFFEE_PWR = 6;
const int AMBIENT_PWR = 3;
const int LED=            9;
const int BATT_ADC=       1;
const int DIP_switch1=    7;
const int DIP_switch2=    8;
const int analog_4 = 4;   
const int analog_5 = 5;   
boolean coffee_sensor_status;
boolean ambient_sensor_status;


double Thermistor(int RawADC) {
 long Resistance;  double Temp;  // Dual-Purpose variable to save space.
 Resistance=10000.0/(1024.0/ADC-1);  //for pull-up conf
 //Resistance=10000.0*((1024.0/RawADC) - 1);
 Temp = log(Resistance); // Saving the Log(resistance) so not to calculate it 4 times later. // "Temp" means "Temporary" on this line.
 //Temp = 1 / (8.54942e-04 + (2.57305e-04 * Temp) + (1.65368e-07 * Temp * Temp * Temp));   // Now it means both "Temporary" and "Temperature"
 Temp = 1 / (8.99942e-04 + (2.57305e-04 * Temp) + (1.65368e-07 * Temp * Temp * Temp)); 
 Temp = Temp - 273.15;  // Convert Kelvin to Celsius                                         // Now it only means "Temperature"
 return Temp;  // Return the Temperature
}

 double Thermistor_ambient(int RawADC_ambient) {
 long Resistance_ambient;  double Temp_ambient;  // Dual-Purpose variable to save space.
 Resistance_ambient=AMBIENT_RESISTOR/(MAX_ADC/ADC-1);  //for pull-up conf
 Temp_ambient = log(Resistance_ambient); // Saving the Log(resistance) so not to calculate it 4 times later. // "Temp" means "Temporary" on this line.
 //Temp = 1 / (8.99942e-04 + (2.57305e-04 * Temp) + (1.65368e-07 * Temp * Temp * Temp));   // Now it means both "Temporary" and "Temperature"
 Temp_ambient = 1 / (8.99942e-04 + (2.57305e-04 * Temp_ambient) + (1.65368e-07 * Temp_ambient * Temp_ambient * Temp_ambient)); 
 Temp_ambient = Temp_ambient - 273.15;  // Convert Kelvin to Celsius                                         // Now it only means "Temperature"
 return Temp_ambient;  // Return the Temperature
}


// Note: Please update emonhub configuration guide on OEM wide packet structure change:
// https://github.com/openenergymonitor/emonhub/blob/emon-pi/configuration.md
typedef struct {                                                      // RFM12B RF payload datastructure      
      int coffee_temp;
      int coffee_humid;
      int ambient_temp;
      int ambient_humid;
      int temp_diff;
      int humid_diff;
      int battery;    
                                                            
} Payload;
Payload wsn_drying_table;

void setup() {
  
  pinMode(COFFEE_PWR,OUTPUT);
  pinMode(AMBIENT_PWR,OUTPUT);
  pinMode(LED,OUTPUT);

  digitalWrite(LED,HIGH);
  pinMode(DIP_switch1, INPUT_PULLUP);
  pinMode(DIP_switch2, INPUT_PULLUP);
  pinMode(BATT_ADC, INPUT);
  boolean DIP1 = digitalRead(DIP_switch1);
  boolean DIP2 = digitalRead(DIP_switch2);
  
  if ((DIP1 == HIGH) && (DIP2 == HIGH)) nodeID=nodeID;
  if ((DIP1 == LOW) && (DIP2 == HIGH)) nodeID=nodeID+1;
  if ((DIP1 == HIGH) && (DIP2 == LOW)) nodeID=nodeID+2;
  if ((DIP1 == LOW) && (DIP2 == LOW)) nodeID=nodeID+3;
  
  rf12_initialize(nodeID, RF_freq, networkGroup);                       // Initialize RFM12B
  
  // Send RFM12B test sequence (for factory testing)
  for (int i=10; i>-1; i--)                                         
  {
    wsn_drying_table.coffee_temp=i; 
    rf12_sendNow(0, &wsn_drying_table, sizeof wsn_drying_table);
    delay(100);
  }
  rf12_sendWait(2);
  wsn_drying_table.coffee_temp=0;
  // end of factory test sequence
  
  rf12_sleep(RF12_SLEEP);
  if (debug==1)
  {
    Serial.begin(9600);
    Serial.print(DIP1); Serial.println(DIP2);
    Serial.println(" Drying table WSN ");
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
  

  //################################################################################################################################
  // Power Save  - turn off what we don't need - http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
  //################################################################################################################################
  ACSR |= (1 << ACD);                     // disable Analog comparator    
  //if (debug==0) power_usart0_disable();   //disable serial UART
  power_twi_disable();                    //Disable the Two Wire Interface module.
  // power_timer0_disable();              //don't disable necessary for RF
  power_timer1_disable();
  power_spi_disable();
  //################################################################################################################################  

  //********************************checking the presence of coffee beans temp and humid *****************************************
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  digitalWrite(AMBIENT_PWR,LOW);
  digitalWrite(COFFEE_PWR,HIGH);
  dodelay(2000);
  temp=Thermistor(analogRead(analog_4));           // read ADC and convert it to Celsius
  COFFEE_HUMID_ADC = analogRead(analog_5);
  COFFEE_HUMID_V = COFFEE_HUMID_ADC * 0.003225806;//V[mv] convert from count to raw voltage, Constant used in oem, 
  VOLTAGE_RATIO=DEFAULT_VCC/INPUT_VCC;
  COFFEE_HUMID= (0.0375 * COFFEE_HUMID_V*1000 *VOLTAGE_RATIO) -37.7; //1000mv, formula uses mv
     
  if ( temp < -20 || COFFEE_HUMID< 1 || COFFEE_HUMID >99)
  {
    delay(100);
    //Serial.print(AMBIENT_HUMID);
    if (debug==1) Serial.println("unable to find coffee beans sensor "); delay(100);
      Sleepy::loseSomeTime(3000);   //wait 3 sec and try again
   
    temp=Thermistor(analogRead(analog_4));           // read ADC and convert it to Celsius
    COFFEE_HUMID_ADC = analogRead(analog_5);
    COFFEE_HUMID_V = COFFEE_HUMID_ADC * 0.003225806;//V[mv] convert from count to raw voltage, Constant used in oem, 
    VOLTAGE_RATIO=DEFAULT_VCC/INPUT_VCC;
    COFFEE_HUMID= (0.0375 * COFFEE_HUMID_V*1000 *VOLTAGE_RATIO) -37.7; //1000mv, formula uses mv
  
    if ( temp < -20 || COFFEE_HUMID< 1 || COFFEE_HUMID >99)
    { 
        delay(100);
        if (debug==1) Serial.println(" - Unable to find coffee beans Sensor for 2nd time..giving up"); 
        coffee_sensor_status=0;
    }
    else
    {
        delay(100);
        coffee_sensor_status=1;
    }
  }
  else
  { 
    delay(100);
    coffee_sensor_status=1; 
    if (debug==1) Serial.println("coffee beans sensor detected &&&&&&&&&&&&&&&"); 
  }
  delay(100);  
  digitalWrite(AMBIENT_PWR,LOW);
  digitalWrite(COFFEE_PWR,LOW);
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////

  
  dodelay(3000);

  //****************************checking the presence of ambient temperature&humid sensor*************************
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  digitalWrite(AMBIENT_PWR,HIGH);
  digitalWrite(COFFEE_PWR,LOW);
  dodelay(2000);
  temp_ambient=Thermistor_ambient(analogRead(analog_5));           // read ADC and convert it to Celsius
  AMBIENT_HUMID_ADC = analogRead(analog_4);
  AMBIENT_HUMID_V = AMBIENT_HUMID_ADC * 0.003225806;//V[mv] convert from count to raw voltage, Constant used in oem, 
  VOLTAGE_RATIO=DEFAULT_VCC/INPUT_VCC;
  AMBIENT_HUMID= (0.0375 * AMBIENT_HUMID_V*1000 *VOLTAGE_RATIO) -37.7; //1000mv, formula uses mv
     
  if ( temp_ambient < -20 || AMBIENT_HUMID< 1 || AMBIENT_HUMID >99)
  {
    delay(100);
   // Serial.print(temp_ambient);
    //Serial.print(AMBIENT_HUMID);
    if (debug==1) Serial.println("unable to find AMBIENT sensor "); delay(100);
      Sleepy::loseSomeTime(3000);   //wait 3 sec and try again
   
    temp_ambient=Thermistor_ambient(analogRead(analog_5));           // read ADC and convert it to Celsius
    AMBIENT_HUMID_ADC = analogRead(analog_4);
    AMBIENT_HUMID_V = AMBIENT_HUMID_ADC * 0.003225806;//V[mv] convert from count to raw voltage, Constant used in oem, 
    VOLTAGE_RATIO=DEFAULT_VCC/INPUT_VCC;
    AMBIENT_HUMID= (0.0375 * AMBIENT_HUMID_V*1000 *VOLTAGE_RATIO) -37.7; //1000mv, formula uses mv
  
    if ( temp_ambient < -20 || AMBIENT_HUMID< 1 || AMBIENT_HUMID >99)
    { 
        delay(100);
        if (debug==1) Serial.println(" - Unable to find AMBIENT Sensor for 2nd time..giving up"); 
        ambient_sensor_status=0;
    }
    else
    {
        ambient_sensor_status=1;
    }
  }
  else
  { 
    delay(100);
    ambient_sensor_status=1; 
    if (debug==1) Serial.println("AMBIENT TEMP&HUMID beans sensor detected &&&&&&&&&&&&&&&"); 
  }
delay(100);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  digitalWrite(AMBIENT_PWR,LOW);
  digitalWrite(COFFEE_PWR,LOW);
  digitalWrite(LED,LOW);
  dodelay(2000);

} // end of void setup



void loop() {

    if ((coffee_sensor_status==0) && (ambient_sensor_status==0))        //if neither ambient or sensor is detected  goto forever sleep
  {
    if (debug ==1) Serial.println("No sensor detected, going to slep to save battery");
    wsn_drying_table.coffee_temp=3;  // 3 from all two sensor to indicate that they are not tected
    wsn_drying_table.coffee_humid=3;
    wsn_drying_table.ambient_temp=3;
    wsn_drying_table.ambient_humid=3;
    wsn_drying_table.temp_diff=3;
    wsn_drying_table.humid_diff=3;
    wsn_drying_table.battery=int(analogRead(BATT_ADC)*0.03225806);; 
    power_spi_enable();
    dodelay(2000);
  
    rf12_sleep(RF12_WAKEUP);
    dodelay(100);
    rf12_sendNow(0, &wsn_drying_table, sizeof wsn_drying_table);
    // set the sync mode to 2 if the fuses are still the Arduino default
    // mode 3 (full powerdown) can only be used with 258 CK startup fuses
    rf12_sendWait(2);
    rf12_sleep(RF12_SLEEP);
    dodelay(100);
    power_spi_disable();  
  
    cli();                                      //stop responding to interrupts 
    Sleepy::powerDown();                        //sleep forever
  }

  if (coffee_sensor_status==1)
  {
     digitalWrite(AMBIENT_PWR,LOW);
     digitalWrite(COFFEE_PWR,HIGH);
     dodelay(2000);
     temp=Thermistor(analogRead(analog_4));           // read ADC and convert it to Celsius
     wsn_drying_table.coffee_temp = temp; 
     dodelay(1000);
     coffee_beans_humidity();
     digitalWrite(AMBIENT_PWR,LOW);
     digitalWrite(COFFEE_PWR,LOW);
   }
    dodelay(3000); // wait 3 sec before to switch to second sensor

  if (ambient_sensor_status==1)
  { 
  
     digitalWrite(COFFEE_PWR,LOW);
     digitalWrite(AMBIENT_PWR,HIGH);
    
     dodelay(2000);
     temp_ambient=Thermistor_ambient(analogRead(analog_5));           // read ADC and convert it to Celsius
     wsn_drying_table.ambient_temp=temp_ambient;
     dodelay(1000);
     get_ambient_humidity();
     digitalWrite(AMBIENT_PWR,LOW);
     digitalWrite(COFFEE_PWR,LOW);
   
   }
   
  wsn_drying_table.temp_diff=abs(wsn_drying_table.ambient_temp - wsn_drying_table.coffee_temp);
  wsn_drying_table.humid_diff=abs(wsn_drying_table.ambient_humid - wsn_drying_table.coffee_humid);
  wsn_drying_table.battery= int(analogRead(BATT_ADC)*0.03225806) ;                 //read battery voltage, convert ADC to volts x10
 
    if (debug == 1)
  {
      if (coffee_sensor_status)
      {
      Serial.print(" Coffee Temp:"); Serial.print(wsn_drying_table.coffee_temp);     // display Celsius
     // Serial.print("  COFFEE_Humid_V: "); Serial.print(COFFEE_HUMID_V);
      Serial.print(" COFFEE_HUMID:");  Serial.print(wsn_drying_table.coffee_humid); Serial.print("% ");
      }
      if (ambient_sensor_status)
      {
      Serial.print(" Ambient Temp:"); Serial.print(wsn_drying_table.ambient_temp); 
     // Serial.print(" AMBIENT_Humid_V: "); Serial.print(AMBIENT_HUMID_V);
      Serial.print(" AMBIENT HUMID:"); Serial.print(wsn_drying_table.ambient_humid); Serial.print("% "); 
      }
      Serial.print("Temp diff:"); Serial.print(wsn_drying_table.temp_diff);
      Serial.print(" Humd diff:"); Serial.print(wsn_drying_table.humid_diff); 
      Serial.print(" Batt Voltage:"); Serial.print(wsn_drying_table.battery/10); Serial.println("V");     
      delay(100); 
  }
  
  power_spi_enable();
  dodelay(2000);
  
  rf12_sleep(RF12_WAKEUP);
  dodelay(100);
  rf12_sendNow(0, &wsn_drying_table, sizeof wsn_drying_table);
  // set the sync mode to 2 if the fuses are still the Arduino default
  // mode 3 (full powerdown) can only be used with 258 CK startup fuses
  rf12_sendWait(2);
  rf12_sleep(RF12_SLEEP);
  dodelay(100);
  power_spi_disable();  
  // no need of blinking after sending data
  //digitalWrite(LED,HIGH);
  //dodelay(100);
  //digitalWrite(LED,LOW); 

   //delay loop, wait for time_between_reading minutes
  for (int i=0; i<time_between_readings; i++)
  {
    dodelay(55000); //1 minute should be 60000 but is not because of variation of internal time source
    //caution parameter cannot be more than 65000, maybe find better solution
    //due to internal time source 60000 is longer than 1 minute. so 55s is used.
  }
   
} // end of void loop


void get_ambient_humidity()
{
  AMBIENT_HUMID_ADC = analogRead(analog_4);
  AMBIENT_HUMID_V = AMBIENT_HUMID_ADC * 0.003225806;//V[mv] convert from count to raw voltage, Constant used in oem, 
  VOLTAGE_RATIO=DEFAULT_VCC/INPUT_VCC;
  AMBIENT_HUMID= (0.0375 * (AMBIENT_HUMID_V+0.04)*1000 *VOLTAGE_RATIO) -37.7; //1000mv, 0.04V to compansate voltage drop due to build in 10K resistor; formula uses mv
  wsn_drying_table.ambient_humid=AMBIENT_HUMID;
  delay(500);
}

void coffee_beans_humidity()
{
  COFFEE_HUMID_ADC = analogRead(analog_5);
  COFFEE_HUMID_V = COFFEE_HUMID_ADC * 0.003225806;//V[mv] convert from count to raw voltage, Constant used in oem, 
  VOLTAGE_RATIO=DEFAULT_VCC/INPUT_VCC;
  COFFEE_HUMID= (0.0375 * COFFEE_HUMID_V*1000 *VOLTAGE_RATIO) -37.7; //1000mv, formula uses mv
  wsn_drying_table.coffee_humid=COFFEE_HUMID;
  delay(500); 
}

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



