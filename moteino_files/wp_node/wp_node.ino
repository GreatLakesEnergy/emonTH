#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <avr/wdt.h>
#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming/tree/master/WirelessHEX69

#define NODEID      5       // node ID used for this unit
#define NETWORKID   100
#define FREQUENCY     RF69_915MHZ
//#define IS_RFM69HW  //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define SERIAL_BAUD 115200
#define ACK_TIME    30  // # of ms to wait for an ack
#define ENCRYPTKEY "sampleEncryptKey" //(16 bytes of your choice - keep the same on all encrypted nodes)
#define BLINKPERIOD 500

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
  
#else
  #define LED           9 // Moteinos hsave LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

RFM69 radio;
char input = 0;
long lastPeriod = -1;

#define TEST_LAMP 1
/////////////////////////////////////////////////////////////////////////////
// flash(SPI_CS, MANUFACTURER_ID)
// SPI_CS          - CS pin attached to SPI flash chip (8 in case of Moteino)
// MANUFACTURER_ID - OPTIONAL, 0x1F44 for adesto(ex atmel) 4mbit flash
//                             0xEF30 for windbond 4mbit flash
//                             0xEF40 for windbond 16/64mbit flash
/////////////////////////////////////////////////////////////////////////////
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for windbond 4mbit flash

void setup(){
  pinMode(LED, OUTPUT);
  pinMode(TEST_LAMP,OUTPUT);
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.encrypt(ENCRYPTKEY); //OPTIONAL
#ifdef IS_RFM69HW
  radio.setHighPower(); //only for RFM69HW!
#endif
  Serial.print("Start node...");

  if (flash.initialize())
    Serial.println("SPI Flash Init OK!");
  else
    Serial.println("SPI Flash Init FAIL!");


}

void loop(){

  if (radio.receiveDone())
  {
    Serial.print("Got [");
    Serial.print(radio.SENDERID);
    Serial.print(':');
    Serial.print(radio.DATALEN);
    Serial.print("] > ");
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i], HEX);
    Serial.println();
    CheckForWirelessHEX(radio, flash, true);
    Serial.println();
  }
 
  
  ////////////////////////////////////////////////////////////////////////////////////////////
  // Real sketch code here, let's blink the onboard LED

  
  if ((int)(millis()/BLINKPERIOD) > lastPeriod)
  {
    lastPeriod++;
    digitalWrite(LED, lastPeriod%2);

  }

  float LDR_reading= analogRead(A7);
  Serial.print("Sensor Value: ");
  Serial.println(LDR_reading);

  digitalWrite(TEST_LAMP, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(LDR_reading);              // wait for a second
  digitalWrite(TEST_LAMP, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second
  


  ////////////////////////////////////////////////////////////////////////////////////////////
}
