#include <RFM69.h>          //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming/tree/master/WirelessHEX69

#define NODEID             3  //this node's ID, should be unique among nodes on this NETWORKID
#define NETWORKID          100  //what network this node is on
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY "sampleEncryptKey" //(16 bytes of your choice - keep the same on all encrypted nodes)
//#define IS_RFM69HW             //uncomment only for RFM69HW! Leave out if you have RFM69W!

#define SERIAL_BAUD 115200
#define ACK_TIME    50  // # of ms to wait for an ack
#define TIMEOUT     3000

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
#else
  #define LED           9 // Moteinos hsave LEDs on D9
#endif

RFM69 radio;
char c = 0;
char input[64]; //serial input buffer
byte targetID=0;
int test=1;
void setup(){
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.encrypt(ENCRYPTKEY); //OPTIONAL
  pinMode(test,OUTPUT);
#ifdef IS_RFM69HW
  radio.setHighPower(); //only for RFM69HW!
#endif
  Serial.println("Start wireless gateway...");
  
}

void loop(){
  byte inputLen = readSerialLine(input, 10, 64, 100); //readSerialLine(char* input, char endOfLineChar=10, byte maxLength=64, uint16_t timeout=1000);
  
  if (inputLen==4 && input[0]=='F' && input[1]=='L' && input[2]=='X' && input[3]=='?') {
    if (targetID==0)
      Serial.println("TO?");
    else
      CheckForSerialHEX((byte*)input, inputLen, radio, targetID, TIMEOUT, ACK_TIME, false);
  }
  else if (inputLen>3 && inputLen<=6 && input[0]=='T' && input[1]=='O' && input[2]==':')
  {
    byte newTarget=0;
    for (byte i = 3; i<inputLen; i++) //up to 3 characters for target ID
      if (input[i] >=48 && input[i]<=57)
        newTarget = newTarget*10+input[i]-48;
      else
      {
        newTarget=0;
        break;
      }
    if (newTarget>0)
    {
      targetID = newTarget;
      Serial.print("TO:");
      Serial.print(newTarget);
      Serial.println(":OK");
    }
    else
    {
      Serial.print(input);
      Serial.print(":INV");
    }
  }
  else if (inputLen>0) { //just echo back
    Serial.print("SERIAL IN > ");Serial.println(input);
  }

  if (radio.receiveDone())
  {
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);
    
    if (radio.ACK_REQUESTED)
    {
      radio.sendACK();
      Serial.print(" - ACK sent");
    }
    
    Serial.println();
  }

  digitalWrite(test,HIGH);
  delay(500);
  digitalWrite(test,LOW);
  delay(500);
  Blink(LED,5); //heartbeat
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}
