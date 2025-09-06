/* Cleansee device code
 * Written by Thingitude in 2025 in association with Hastings OBX.
 * You are licensed to use and modify this code however you see fit
 * without any obligation.  Have fun!
 * 
 * This code was written for a Heltec ESP32 LoRa v3 microcontroller.
 * It was first used in an OBX workshop called Stink or Swim on May 10th 2025.
 * It was modified for the 6 Sept 2025 workshop to add 3 LEDs
 *
 * The code is provided with no warranty whatsoever.
 *
 * For further information about the Cleansee project, visit:
 * https://thingitude.com/cleansee-sewage-monitoring-sea-conditions/
 * */

#include "LoRaWan_APP.h"
#include "Heltec.h"
#include <Wire.h>               
#include "HT_SSD1306Wire.h"
#include <ESP32Servo.h>
#include "soc/rtc.h"

#define TOUCH_THRESHOLD 1200

Servo myServo;  // create servo object to control the servo
static SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst
touch_pad_t touchPin;  // we will use GPIO 6 as a touchpin to wake up the display

/*  CHANGE THESE VALUES FOR YOUR DEVICE  */
char * startup_msg = "jj-cleansee";

#define NO_SPILLS 0    // The angle of the pointer when there are no recent spills
#define ONGOING_SPILL 180  // The angle of the pointer when there is an ongoing spill
#define SPILL_24 120      // The angle of the pointer when there was a spill in the past 24 hours
#define SPILL_72 60      // The angle of the pointer when there was a spill in the past 72 hours
#define OUTFALL_MAINT 90 // The angle of the pointer whn the sewage outfall is under maintenance

#define SERVO_LOW 400    // The lower number for 180 degrees MG90S microservo 500
#define SERVO_HIGH 2800  // The upper number for 180 degrees MG90S microservo 2400

const int servoPin=7;       // The GPIO that the orange wire from the servo is attached to
const int pushButton=6;     // The GPIO that the pushbutton is attached to
const int greenLed=5;    // The GPIO for the green LED pin
const int amberLed=4;    // The GPIO for the amber LED pin
const int redLed=3;      // The GPIO for the red LED pin
RTC_DATA_ATTR int ledToLight=0;
/*  LoRaWAN Parameters - CHANGE THE DEV EUI for your device  */
uint8_t devEui[] = { 0x70, 0xb3, 0xd5, 0x7e, 0xd0, 0x06, 0xff, 0xcc };

/*  Other LoRaWAN parameters - DON'T CHANGE THESE IN THE WORKSHOP */
uint8_t appEui[] = { 0x00, 0x12, 0x00, 0x12, 0x00, 0x12, 0x00, 0x12 };
uint8_t appKey[] = { 0x77, 0x77, 0x62, 0xa9, 0xe7, 0xcb, 0x77, 0x77, 0xcb, 0xcf, 0xa1, 0xae, 0x12, 0xc8, 0x77, 0x77 };
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;

/* Number of trials to transmit the frame, if the LoRaMAC layer did not
 * receive an acknowledgment. */
uint8_t confirmedNbTrials = 4;
/* */
/*  You don't *NEED* to change anything below here, but feel free */
/* */

/*the application data transmission duty cycle.  value in [ms].*/
// IF YOUR DEVICE IS BATTERY POWERED I strongly recommend increasing this value to 1 hour or more - 6?
uint32_t appTxDutyCycle = 60000;   // send a message every 5 mins (300 secs or 300,000 microseconds). 1 hour is 3600000


/* Two counters to estimate how long ago the previous downlink was received */
RTC_DATA_ATTR uint64_t lastUpdate;
uint64_t timeCheck;

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
    // We are just sending the same 4 bytes every appTxDutyCycle msecs.
    appDataSize = 4;
    appData[0] = 0x00;
    appData[1] = 0x01;
    appData[2] = 0x02;
    appData[3] = 0x03;
}

//downlink data handle function 
void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  int servoByte;
  char str[30];

  Serial.printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n",mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",mcpsIndication->BufferSize,mcpsIndication->Port);
  Serial.print("+REV DATA:");
  for(uint8_t i=0;i<mcpsIndication->BufferSize;i++)
  {
    Serial.printf("%02X\n",mcpsIndication->Buffer[i]);
  }
  servoByte=mcpsIndication->Buffer[0];
  lastUpdate=rtc_time_get(); //Note the time the downlink was received.
  Serial.printf("Last downlink received: %i \n",lastUpdate);
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  sprintf(str,"Received %d",servoByte);
  display.drawString(64,10,str);
  display.display();

  updateForServoByte(servoByte);
  
  turnOnLed("DOWNLINK WITH PAYLOAD");
  Serial.println("...and now we sleep...");
}

void updateForServoByte(int servoByte) {
  int pos=0;
  myServo.attach(servoPin, SERVO_LOW, SERVO_HIGH);

  switch(servoByte) {
    case 0: // No recent spills
      pos=NO_SPILLS;
      slowWrite(pos,100);
      ledToLight=greenLed;
      break;
    case 1: // ongoing spill
      pos=ONGOING_SPILL;
      slowWrite(pos,100);
      ledToLight=redLed;
      break;
    case 24: // spill in past 24 hrs
      pos=SPILL_24;
      slowWrite(pos,100);
      ledToLight=redLed;
      break;
    case 72: // spill in past 72 hours
      pos=SPILL_72;
      slowWrite(pos,100);
      ledToLight=amberLed;
      break;
    case 255: // site under maintenance
      pos=OUTFALL_MAINT;
      slowWrite(pos,100);
      ledToLight=0;
      break;
    default:
      pos=OUTFALL_MAINT;
      slowWrite(pos,100);
      break;
  }
  myServo.write(pos);
  delay(2000);
  Serial.print("Set servo angle to ");
  Serial.print(pos);
  Serial.println(" degrees.");
  myServo.detach();
}

String howLongSinceDownlink() {
  const uint32_t oneSec=124500;
  char timeStr[14];
  uint64_t timeRemainder=0;
  uint64_t timeDiff=0;
  timeDiff=timeCheck-lastUpdate;
  Serial.printf("Last update: %i \n",lastUpdate);
  Serial.printf("Time now: %i \n",timeCheck);
  Serial.printf("Time difference: %i \n",timeDiff);

  uint8_t hrs=(timeDiff/(oneSec*60*60));
  timeRemainder=(timeDiff%(oneSec*60*60));
  uint8_t mins=(timeRemainder/(oneSec*60));
  timeRemainder=(timeRemainder%(oneSec*60));
  uint8_t secs=(timeRemainder/oneSec);
  sprintf(timeStr,"%i:%i:%i ago",hrs,mins,secs);
  return(String(timeStr));
}

void slowWrite(int pos, int gap) {
  // Purpose of this function is to make the servo move more smoothly and slowly
  int i=0;
  // Get current position of servo
  int currentPos=myServo.read();

  if(currentPos >180) {
    currentPos=currentPos % 180;
  }
  delay(1000);
  Serial.printf("Current Pos is %i, and target Pos is %i\n",currentPos, pos);
  
  if(currentPos < pos) {
    //Rotate in increments
    for(i=currentPos;i<pos;i++) {
      myServo.write(i);
      delay(gap);
    }
  } else if(currentPos > pos) {
    //Rotate in decrements
    for(i=currentPos;i>pos;i--) {
      myServo.write(i);
      delay(gap);    
    }
  }
}

void cycleLeds() {
  digitalWrite(greenLed, HIGH);
  delay(1000);
  digitalWrite(greenLed, LOW);
  digitalWrite(amberLed, HIGH);
  delay(1000);
  digitalWrite(amberLed, LOW);
  digitalWrite(redLed, HIGH);
  delay(1000);
  digitalWrite(redLed, LOW);
  delay(1000);
  turnOnLed("CYCLE LEDs");
}

void turnOnLed(char * status) {
  digitalWrite(greenLed, LOW);
  digitalWrite(amberLed, LOW);
  digitalWrite(redLed, LOW);
  Serial.printf("*** %s ***\n",status);
  Serial.printf("*** LED to light is %i ***\n",ledToLight);
  if(ledToLight >0) {
    digitalWrite(ledToLight, HIGH);
  }
}

String bytesToHex(const uint8_t* buf, size_t len,
                  char sep = ' ', int bytesPerLine = 8, bool lowercase = false) {
  const char* D = lowercase ? "0123456789abcdef" : "0123456789ABCDEF";
  size_t lines = bytesPerLine ? (len + bytesPerLine - 1) / bytesPerLine : 1;
  size_t seps  = sep ? (len ? (len - 1) : 0) : 0;
  size_t nl    = lines ? (lines - 1) : 0;
  String s; s.reserve(len * 2 + seps + nl);

  for (size_t i = 0; i < len; ++i) {
    if (i && sep && (i % bytesPerLine)) s += sep;
    if (i && bytesPerLine && (i % bytesPerLine) == 0) s += '\n';
    uint8_t v = buf[i];
    s += D[v >> 4];
    s += D[v & 0x0F];
  }
  return s;
}

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  Heltec.VextON();
	display.init();											// initialise the display
	display.setFont(ArialMT_Plain_16);	//	set the font
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64,10,startup_msg);
  display.setFont(ArialMT_Plain_10);	//	set the font
  display.drawString(64,28,bytesToHex(devEui, 8));
   display.setFont(ArialMT_Plain_16);	//	set the font
  display.display();
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  myServo.setPeriodHertz(50);// Standard 50hz servo
  pinMode(greenLed, OUTPUT);
  pinMode(amberLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  cycleLeds();
  turnOnLed("Setup()");  
  //pinMode(pushButton, INPUT_PULLUP);
  //touchSleepWakeUpEnable(pushButton,TOUCH_THRESHOLD);   // set up sleep wakeup on the touchpad
}

void loop()
{
  switch( deviceState )
  {
    case DEVICE_STATE_INIT:
    {
#if(LORAWAN_DEVEUI_AUTO)
      LoRaWAN.generateDeveuiByChipID();
#endif
      LoRaWAN.init(loraWanClass,loraWanRegion);
      //both set join DR and DR when ADR off 
      LoRaWAN.setDefaultDR(3);
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      timeCheck=rtc_time_get(); //Note the time of the check.
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_CENTER);
      display.drawString(64,4,"Last updated");
      display.drawString(64,22,howLongSinceDownlink());
      display.drawString(64,40,"Checking...");
      display.display();
      cycleLeds();
      delay(2000);
      display.clear();
      delay(1000);
      prepareTxFrame( appPort );
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      turnOnLed("DEVICE_STATE_SEND");
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      // Schedule next packet transmission
      txDutyCycleTime = appTxDutyCycle + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
     // IF YOUR DEVICE IS BATTERY POWERED uncomment the line below and comment out Mcu.timehandler and Radio.IrqProcess
      //LoRaWAN.sleep(loraWanClass);
      Mcu.timerhandler();
      Radio.IrqProcess();
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}
