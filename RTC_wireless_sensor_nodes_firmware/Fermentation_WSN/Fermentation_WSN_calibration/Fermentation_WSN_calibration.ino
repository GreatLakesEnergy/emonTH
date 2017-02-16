/*
 * This PH sensor has two data serial Protocal, UART and I2C, We used UART protocal due to availability of TX and RX, I@C was impossible because of A4 and A% which are used for other purpose 
 * When you program over FTDI cable, it uses the tx and rx pins, so you need to disconnect anything else that might interfere on those pins, Because of this situation, to calibrate the PH sensor with FTDI cable is quite hard because you need to talk to the chip(Sending and receiving the response immediately)
 * Because of above mentioned issue, we need to use softwareSerial to do so.
 * --------------------------------------------------------------------
 * EmonTH has free digital pin 5 and 3, so during the process of this calibration we need to:
 * 1. Disconnect the TX and RX of the PH pins from the TH
 * 2. Connect PH_TX  to digital pin 5
 * 3. Connect PH_RX to digital pin 3
 * 4. Disconnect everthing connected to the emonTH_FTDI header(Note: check where these jumper wire are connected so that you connected it back after calibration)
 * 5. connect FTDI cable
 * 6. Upload This Schetch 
 * 7. Check Serial monitor, (Baud rate:9600; Autoscroll, Carriage return) you can see current PH readings from the sensor
 * 8. You have Four bottle of calibration solution: Storage solution, PH=4, PH=7, PH:10
 * 9. Start with PH=7; pour some solution to calibration container and put it on sensor problem, wait one minutes
 * 10.Type in Command line "Cal,mid,7.00<CR>" (If the response code is enabled, the EZO™ class circuit will respond “*OK<CR>”) The LED will turn Cyan during the calibration.
 * 11. After this response,
 * 12. pour some solution to calibration container PH=4 and put it on sensor problem, wait one minutes again
 * 13. type "Cal,low,4.00<CR>" (If the response code is enabled, the EZO™ class circuit will respond “*OK<CR>”) The LED will turn Cyan during the calibration.
 * 14. repeat process 12 for PH=10 and type "Cal,high,10.00<CR>" and you will get the same response
 * 15. After this Process, Upload the RTC_WSN_firmaware and put back each wire moved to its position
 */


#include <SoftwareSerial.h>                           //we have to include the SoftwareSerial library, or else we can't use it.
#define rx 5                                         //define what pin rx is going to be.
#define tx 3                                          //define what pin tx is going to be.

SoftwareSerial myserial(rx, tx);                      //define how the soft serial port is going to work.


String inputstring = "";                              //a string to hold incoming data from the PC
String sensorstring = "";                             //a string to hold the data from the Atlas Scientific product
boolean input_stringcomplete = false;                 //have we received all the data from the PC
boolean sensor_stringcomplete = false;                //have we received all the data from the Atlas Scientific product
float ph;                                             //used to hold a floating point number that is the pH. 



void setup() {                                        //set up the hardware
  Serial.begin(9600);                                 //set baud rate for the hardware serial port_0 to 9600
  myserial.begin(9600);                               //set baud rate for software serial port_3 to 9600
  inputstring.reserve(10);                            //set aside some bytes for receiving data from the PC
  sensorstring.reserve(30);                           //set aside some bytes for receiving data from Atlas Scientific product
}


void serialEvent() {                                  //if the hardware serial port_0 receives a char
  char inchar = (char)Serial.read();                  //get the char we just received
  inputstring += inchar;                              //add it to the inputString
  if (inchar == '\r') {                               
    input_stringcomplete = true;                      //if the incoming character is a <CR>, set the flag
  }
}

void loop() {                                         //here we go...

  if (input_stringcomplete) {                         //if a string from the PC has been received in its entirety                     
    myserial.print(inputstring);                      //send that string to the Atlas Scientific product
    inputstring = "";                                 //clear the string
    input_stringcomplete = false;                     //reset the flag used to tell if we have received a completed string from the PC      
  }

  if (myserial.available() > 0) {                     //if we see that the Atlas Scientific product has sent a character.
    char inchar = (char)myserial.read();              //get the char we just received
    sensorstring += inchar;
    if (inchar == '\r') {
      sensor_stringcomplete = true;                   //if the incoming character is a <CR>, set the flag
    }
  }


  if (sensor_stringcomplete) {                        //if a string from the Atlas Scientific product has been received in its entirety
    Serial.println(sensorstring);                     //send that string to the PC's serial monitor
    ph = sensorstring.toFloat();                      //convert the string to a floating point number so it can be evaluated by the Arduino

    sensorstring = "";                                //clear the string:
    sensor_stringcomplete = false;                    //reset the flag used to tell if we have received a completed string from the Atlas Scientific product
  }
}
