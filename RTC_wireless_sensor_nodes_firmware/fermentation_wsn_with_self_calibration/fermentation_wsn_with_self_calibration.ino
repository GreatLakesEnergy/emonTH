/*
 * To make this file universal for all three fermentation tanks
 * "PH 1: DIP1=OFF, DIP2=OFF" ==>> NodeID:23
 * "PH 2: DIP1=ON, DIP2=OFF" ==>> NodeID:24
 * "PH 3: DIP1=OFF, DIP2=ON" ==>> NodeID:25
 * DIP1=ON, DIP2=ON" ==>> calibration mode
 * 
 *  This PH sensor has two data serial Protocal, UART and I2C, We used UART protocal due to availability of TX and RX, I@C was impossible because of A4 and A% which are used for other purpose 
 * When you program over FTDI cable, it uses the tx and rx pins, so you need to disconnect anything else that might interfere on those pins, Because of this situation, 
 * to calibrate the PH sensor with FTDI cable is quite hard because you need to talk to the chip(Sending and receiving the response immediately)
 * Because of above mentioned issue and the PH circuit which is interrupt based that affect the ethanol readings, we need to use softwareSerial to do so.
 * --------------------------------------------------------------------
 * EmonTH has free digital pin 5 and 3, so during the process of this calibration we need to:
 * 1. Disconnect the power supply of WSN
 * 2. Disconnect the TX and RX of the PH pins from the TH
 * 2. Connect PH_TX  to digital pin 5; Check where these jumper are connected carefully as it need to be connected back as it was
 * 3. Connect PH_RX to digital pin 3; Check where these jumper are connected carefully as it need to be connected back as it was
 * 4. Check the position of DIP1, and 2 switches so that after calibration you put them back in their position
 * 5. Change the DIP switch to look like DIP1=ON(Position written on it); DIP2=ON(Position written on it)
 * 6. Always Start with PH=7(Midpoint calibration); pour some solution to calibration container, put the PH problem in solution
 * 7. Connect the power supply
 * 7. Check if the LED is steady ON, If so, the WSN is in calibration Mode
 * 8. Wait for 3 minutes and check the LED if it blinks almost 10 times, if so the midpoint calibration is done, change the solution in less than 2 min so that the sensor detect the H particles before calibration
 * 8. Put the sensor probe in ph=10(Highpoint calibration); wait around 3min, and check the LED if it blinks almost 10 times, if so the highpoint calibration is done, change the solution in less than 2 min so that the sensor detect the H particles before calibration
 * 9. Put the sensor probe in ph=4(Highpoint calibration); wait around 3min, and check the LED if it blinks almost 10 times, if so the lowpoint calibration is done, change the solution in less than 2 min so that the sensor detect the H particles before calibration
 * 10. Disconnect the power supply
 * 11. Bring back the DIP switches to their position
 * 12. Connect back the TX and RX of PH circuit to their previous connection
 * 13. Connect the power supply back; the calibration is done
 * 
 */
  
#define RF69_COMPAT 1                                                              // Set to 1 if using RFM69CW or 0 is using RFM12B
#include <JeeLib.h>                                                                      //https://github.com/jcw/jeelib - Tested with JeeLib 3/11/14

boolean debug=1;                                       //Set to 1 to few debug serial output, turning debug off increases battery life

#define RF_freq RF12_433MHZ                 // Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
int nodeID = 23;                               // EmonTH temperature RFM12B node ID - should be unique on network
uint32_t networkGroup = 3421749817;    //EmonTH RFM12B wireless network group, numeric value from ansible rmc_key
                                                                      
const int time_between_readings= 1;     // in minutes
#include <avr/power.h>
#include <avr/sleep.h>                                           

ISR(WDT_vect) { Sleepy::watchdogEvent(); }  // Attached JeeLib sleep function to Atmega328 watchdog -enables MCU to be put into sleep mode inbetween readings to reduce power consumption 

// Hardwired emonTH pin allocations 

const int LED=            9;
const int BATT_ADC=       1;
const int DIP_switch1=    7;
const int DIP_switch2=    8;
char Sensor_PH[]="PH : ";
char ph_data[20];                  //we make a 20 byte character array to hold incoming data from the pH.                           
byte received_from_sensor=0;       //we need to know how many characters have been received.
byte startup=0;                    //used to make sure that our contreol unit takes over control of the pH Circuit properly.
float ph_calibrate=0;                        //used to hold a floating point number that is the pH(During calibration).
byte string_received=0;            //used to identify when we have received a string from the pH circuit.

unsigned long previousCalibration = 0;        // will store last time calibration was updated
const long calibrate_after = 4;     //  wait this time (in min) to make sure that the readings from the ph is kind of stable
const int calibration_delay= 50;   // for calibration time, show end user that the calibration is in progress and send calibration command for multiple time as some time it fails to talk to the ph module
int i;    // for for loop
boolean DIP1;   // used during NodeId config and calibration
boolean DIP2; // used during NodeId config and calibration
const int calibration_sampling= 20; 

 
// Note: Please update emonhub configuration guide on OEM wide packet structure change:
// https://github.com/openenergymonitor/emonhub/blob/emon-pi/configuration.md
typedef struct {                                                      // RFM12B RF payload datastructure
      int ph;
      int ethanol;
      int battery;    
      //int pulseCount;
                                                            
} Payload;
Payload wsn_fermentation;


String inputstring = "";                              //a string to hold incoming data from the PC
boolean input_stringcomplete = false;                 //have we received all the data from the PC                                          
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

#include <SoftwareSerial.h>      //we have to include the SoftwareSerial library, or else we can't use it(PH circuit), it is interrupt based. 
#define rx 5                     //define what pin rx is going to be.
#define tx 3                     //define what pin Tx is going to be.

SoftwareSerial myserial(rx, tx); //define how the soft serial port is going to work.


void setup() {                                        //set up the hardware
  inputstring.reserve(10);                            //set aside some bytes for receiving data from the PC
  myserial.begin(9600);        //enable the software serial port
  pinMode(LED,OUTPUT);
  digitalWrite(LED,HIGH);
  pinMode(DIP_switch1, INPUT_PULLUP);
  pinMode(DIP_switch2, INPUT_PULLUP);
  DIP1= digitalRead(DIP_switch1);
  DIP2= digitalRead(DIP_switch2);
  
  if ((DIP1 == HIGH) && (DIP2 == HIGH)) nodeID=nodeID;
  if ((DIP1 == LOW) && (DIP2 == HIGH)) nodeID=nodeID+1;
  if ((DIP1 == HIGH) && (DIP2 == LOW)) nodeID=nodeID+2;


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
 // myserial.print("c,1\r");   //enable continuous mode 
 // myserial.print("L,1\r");      //Turn the LEDs ON
 for (i=0; i <2; i++)
 {
 delay(50);
 Serial.write("c,1\r");   //enable PH continuous mode 
 Serial.write("L,1\r");      //Turn the PH LEDs ON

 }
  
  digitalWrite(LED,LOW);
  ph_status=1;
  ethanol_status=1;
}


void serialEvent() {  
  if (Serial.available() >0) {   
     inputstring = Serial.readString();                  //get the char we just received
     input_stringcomplete = true;                      //if the incoming character is a <CR>, set the flag
    // sensor_stringcomplete = true;                   //if the incoming character is a <CR>, set the flag
  }
}

void loop() {

      while ((DIP1 == LOW) && (DIP2 == LOW))
  {
      if(myserial.available() > 0){        //if we see that the pH Circuit has sent a character.
     received_from_sensor=myserial.readBytesUntil(13,ph_data,20); //we read the data sent from pH Circuit until we see a <CR>. We also count how many character have been received. 
     ph_data[received_from_sensor]=0;  //we add a 0 to the spot in the array just after the last character we received. This will stop us from transmitting incorrect data that may have been left in the buffer.
     string_received=1;                //a flag used when the Arduino is controlling the pH Circuit to let us know that a complete string has been received.
     }
     calibrate();
     
  }
 
    
if (ph_status==1)
{
  if (input_stringcomplete) {                         //if a string from the serial has been received in its entirety                     
    int len = inputstring.length();
    ph = &inputstring[len - 7];   //http://forum.arduino.cc/index.php?topic=369818.0
    PH= atof(ph);
    wsn_fermentation.ph=PH*10;
    if (debug ==1)
    {
      Serial.print(Sensor_PH);
      Serial.print((float)wsn_fermentation.ph/10);
      Serial.print(",  ");
    }
    
    inputstring = "";                                 //clear the string
    input_stringcomplete = false;                     //reset the flag used to tell if we have received a completed string from the PC      
  }
}


if(ethanol_status==1)
{
   Count = analogRead(ETH_BTA_PIN);
  Voltage = Count / ADC_MAX * ETH_BTA_VCC;// convert from count to raw voltage
  SensorReading= COEFFICIENT* pow(Voltage, POWER);
  wsn_fermentation.ethanol=SensorReading*100;
  if (debug ==1)
  {
      Serial.print(Sensor_ETH_BTA);
      Serial.print((float)wsn_fermentation.ethanol/100);
      Serial.println(" % ");  
      delay(TimeBetweenReadings);// delay in between reads for stability
  }
}
  wsn_fermentation.battery= int(analogRead(BATT_ADC)*0.03225806); ;                 //read battery voltage, convert ADC to volts x10
 // wsn_fermentation.pulseCount=1;
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
  dodelay(500);
  digitalWrite(LED,LOW); 

   //delay loop, wait for time_between_reading minutes
  for (int i=0; i<time_between_readings; i++)
  {
    dodelay(50000); //1 minute should be 60000 but is not because of variation of internal time source
    //caution parameter cannot be more than 65000, maybe find better solution
    //due to internal time source 60000 is longer than 1 minute. so 50s is used.
  }

} // end of void loop

void calibrate()
{
    if(startup==0)
    {                //if the Arduino just booted up, we need to set some things up first. 
        digitalWrite(LED,HIGH);  
        myserial.print("c,0\r");   //take the pH Circuit out of continues mode.
        // myserial.print("c,1\r");   //enable continuous mode 
        //myserial.print("L,1\r");      //Turn the LEDs ON
        //myserial.print("L,0\r");      //Turn the LEDs OFF
        delay(50);                 //on start up sometimes the first command is missed.
        digitalWrite(LED,LOW);
        myserial.print("c,0\r");   //so, let's send it twice.
        delay(50);                 //a short delay after the pH Circuit was taken out of continues mode is used to make sure we don't over load it with commands.
        startup=1;                 //startup is completed, let's not do this again during normal operation.
     }
      
   digitalWrite(LED,HIGH);    
   delay(900);                         //we will take a reading ever 800ms. You can make this much longer or shorter if you like.
   myserial.print("R\r");             //send it the command to take a single reading.
  
   if(string_received==1)  //did we get data back from the ph Circuit?
   {            
      ph_calibrate=atof(ph_data);                //Convert string to float 
     // Serial.println(ph_data);
      string_received=0;  //reset the string received flag.
      }             
     
     if (ph_calibrate >=1 && ph_calibrate <5) 
     {
        if (debug ==1) Serial.print("lowpoint cal. required: ");Serial.println(ph_calibrate);
        unsigned long current_time = millis()/60000;
        
       if(current_time - previousCalibration >= calibrate_after) 
       {
          previousCalibration = current_time;
          calibrate_ph("cal,low,4\r"); 
      }
     }
     
     if (ph_calibrate >=5 && ph_calibrate <8) 
     {
        if (debug ==1) Serial.print("midpoint cal required: ");Serial.println(ph_calibrate);
        unsigned long current_time = millis()/60000;
        
        if(current_time - previousCalibration >= calibrate_after) 
        {
            previousCalibration = current_time;
            calibrate_ph("cal,mid,7\r");
        }
     }
  
     if (ph_calibrate >=8 && ph_calibrate <14)
     {
        if (debug ==1) Serial.print("Highpoint cal required: ");Serial.println(ph_calibrate);
        unsigned long current_time = millis()/60000;
        if(current_time - previousCalibration >= calibrate_after) 
        {
            previousCalibration = current_time;
            calibrate_ph("cal,high,10\r");
        }
     }
} 
void calibrate_ph(String calibration_command)
{
  for (i=0; i<20; i++)
  {
      dodelay(calibration_delay);
      digitalWrite(LED,HIGH);
      dodelay(calibration_delay);
      digitalWrite(LED,LOW);
      myserial.print(calibration_command);   //send the "cal,high,10" command to calibrate to a pH of 10.00 
  }
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
