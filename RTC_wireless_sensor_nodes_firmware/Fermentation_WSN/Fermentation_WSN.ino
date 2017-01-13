#define RF69_COMPAT 1                                                              // Set to 1 if using RFM69CW or 0 is using RFM12B
#include <JeeLib.h>                                                                      //https://github.com/jcw/jeelib - Tested with JeeLib 3/11/14

boolean debug=1;                                       //Set to 1 to few debug serial output, turning debug off increases battery life

#define RF_freq RF12_433MHZ                 // Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
int nodeID = 21;                               // EmonTH temperature RFM12B node ID - should be unique on network
const int networkGroup = 210;                // EmonTH RFM12B wireless network group - needs to be same as emonBase and emonGLCD
//uint32_t networkGroup = 3421749817;    //numeric value from ansible rmc_key
                                                                      

const int time_between_readings= 2;                                   // in minutes
#include <avr/power.h>
#include <avr/sleep.h>                                           

ISR(WDT_vect) { Sleepy::watchdogEvent(); }                            // Attached JeeLib sleep function to Atmega328 watchdog -enables MCU to be put into sleep mode inbetween readings to reduce power consumption 

// Hardwired emonTH pin allocations 

const int LED=            9;
const int BATT_ADC=       1;
const int DIP_switch1=    7;
const int DIP_switch2=    8;
char Sensor_PH[]="PH : ";

 
// Note: Please update emonhub configuration guide on OEM wide packet structure change:
// https://github.com/openenergymonitor/emonhub/blob/emon-pi/configuration.md
typedef struct {                                                      // RFM12B RF payload datastructure
      int ph;
      int ethanol;
      int battery;    
                                                            
} Payload;
Payload wsn_fermentation;


String inputstring = "";                              //a string to hold incoming data from the PC
String sensorstring = "";                             //a string to hold the data from the Atlas Scientific product
boolean input_stringcomplete = false;                 //have we received all the data from the PC
boolean sensor_stringcomplete = false;                //have we received all the data from the Atlas Scientific product                                           
char data[10];    //http://stackoverflow.com/questions/15914220/c-error-incompatible-types-in-assignment-of-char-to-char-2
char *ph=data;   //used to hold a floating point number that is the pH. 
float PH;

////////////////////////////////////////
char Sensor_ETH_BTA[]="Ethanol : ";
float POWER = -2.995;
float COEFFICIENT = 0.9054;
float ETH_BTA_VCC = 5.0;
int ADC_MAX = 1023; 
float Count;
float Voltage;
float SensorReading;
float Time;
int TimeBetweenReadings = 500; // in ms

int ETH_BTA_PIN = A5;
//int ReadingNumber=0;
boolean ph_status;
boolean ethanol_status;


void setup() {                                        //set up the hardware
  inputstring.reserve(10);                            //set aside some bytes for receiving data from the PC
  sensorstring.reserve(30);                           //set aside some bytes for receiving data from Atlas Scientific product
  pinMode(LED,OUTPUT);
  pinMode(activate_ethanol,OUTPUT);
  digitalWrite(activate_ethanol,HIGH);
  digitalWrite(LED,HIGH);
  pinMode(DIP_switch1, INPUT_PULLUP);
  pinMode(DIP_switch2, INPUT_PULLUP);
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
    wsn_fermentation.ph=i; 
    rf12_sendNow(0, &wsn_fermentation, sizeof wsn_fermentation);
    delay(100);
  }
  rf12_sendWait(2);
  wsn_fermentation.ph=0;
  // end of factory test sequence
  
  rf12_sleep(RF12_SLEEP);
  if (debug==1)
  {
    Serial.begin(9600);
    Serial.print(DIP1); Serial.println(DIP2);
    Serial.println(" WSN ");
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
  

  pinMode(BATT_ADC, INPUT);

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
  digitalWrite(LED,LOW);
  ph_status=1;
  ethanol_status=1;
}


void serialEvent() {  
  if (Serial.available() >0) {   
     inputstring = Serial.readString();                  //get the char we just received
     input_stringcomplete = true;                      //if the incoming character is a <CR>, set the flag
     sensor_stringcomplete = true;                   //if the incoming character is a <CR>, set the flag
  }
}

void loop() { 

if (ph_status==1)
{
  if (input_stringcomplete) {                         //if a string from the PC has been received in its entirety                     
    //Serial.println(inputstring);                      //send that string to the Atlas Scientific product
    int len = inputstring.length();
    ph = &inputstring[len - 7];   //http://forum.arduino.cc/index.php?topic=369818.0
    PH= atof(ph);
    wsn_fermentation.ph=PH*10;
    Serial.print(Sensor_PH);
    Serial.print(wsn_fermentation.ph);
    Serial.print(",  ");
    
    inputstring = "";                                 //clear the string
    input_stringcomplete = false;                     //reset the flag used to tell if we have received a completed string from the PC      
    sensorstring = "";                                //clear the string:
    sensor_stringcomplete = false;                    //reset the flag used to tell if we have received a completed string from the Atlas Scientific product
  }
}

if(ethanol_status==1)
{
   Count = analogRead(ETH_BTA_PIN);
  Voltage = Count / ADC_MAX * ETH_BTA_VCC;// convert from count to raw voltage
  SensorReading= COEFFICIENT* pow(Voltage, POWER);
  wsn_fermentation.ethanol=SensorReading*10;
  Serial.print(Sensor_ETH_BTA);
  Serial.print(wsn_fermentation.ethanol);
  Serial.println(" % ");
  delay(TimeBetweenReadings);// delay in between reads for stability
}

  wsn_fermentation.battery= int(analogRead(BATT_ADC)*0.03225806); ;                 //read battery voltage, convert ADC to volts x10
  power_spi_enable();
  
  rf12_sleep(RF12_WAKEUP);
  dodelay(100);
  rf12_sendNow(0, &wsn_fermentation, sizeof wsn_fermentation);
  // set the sync mode to 2 if the fuses are still the Arduino default
  // mode 3 (full powerdown) can only be used with 258 CK startup fuses
  rf12_sendWait(2);
  rf12_sleep(RF12_SLEEP);
  dodelay(100);
  power_spi_disable();  
  
  digitalWrite(LED,HIGH);
  dodelay(100);
  digitalWrite(LED,LOW); 

   //delay loop, wait for time_between_reading minutes
  for (int i=0; i<time_between_readings; i++)
  {
    dodelay(55000); //1 minute should be 60000 but is not because of variation of internal time source
    //caution parameter cannot be more than 65000, maybe find better solution
    //due to internal time source 60000 is longer than 1 minute. so 55s is used.
  }
   
} // end of void loop

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
  



