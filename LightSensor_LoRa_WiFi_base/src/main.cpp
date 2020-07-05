// Building up LoRa using the HELTEC ESP32 WiFi board as a receiving base.
// dmf 6.27.20
//
//
// 6.28.20 OK, simple RadioHead LoRa library implementation works. 
//         OK, simple reporting as in the T2 Whispernode code works. 
//         OK, simple display handling is working now. 
//         OK, have WiFi connection working now. 
// 7.2.20  OK, have basic ThingSpeak upload working OK
// 7.3.20  Restructured to require both lightlevel and voltage for display/upload
//         OK, uploading three fields to ThinkSpeak at once (lightlevel, voltage, RSSI) 
// 7.4.20  Revised node code (sender)
// 
// TODO: 
//      1st - implement a clean 'waiting' indicator to display dots on OLED when no data
//      2nd - annotate to note the hidden details that are defaults in RadioHead
//            (and which differ from HelTec); break them out so everything is 
//            explicitly configured (or can be). 
//      3rd - return to sensor circuit, which is saturating in outdoor light
//      4th - adjust ThingSpeak plotting to use log scale for light levels
//      5th - Revise node code (sender) to send light level and voltage as 
//            a struct to simplify future wake up - report - sleep behavior
//
//
// For the HELTEC ESP32_LoRa board I am using: 
// ESP32 Chip ID = 78A0FCBF713C
// 'license' is 0xE425455E,0x76C57FC1,0x4F2610DC,0xF431F294
//
// 7.1.20 Dropping attempt to move to Adafruit IO: 
// This version compiles and runs on the HelTec board. However, in order 
// to use the Adafruit IO service, the WiFi-relevant code would have to be
// completely re-written because the Adafruit IO uses its own versions of 
// the WiFi library. AND there's a SleepyDog dependency that is not relevant
// for this configuration but which prevents compilation. Not worth the 
// effort that's been wasted so far. It's the same problem as with the 
// Heltec libraries - it's not well-defined independent pieces of code, 
// but hidden dependencies and layers that end up screwing up everything 
// when trying to merge differently developed systems together. 
// 

// ----------------------------------------------------------

// Using the espressif arduino framework, the Arduino.h include pulls the file located here:  
// ~/.platformio/packages/framework-arduinoespressif32@2.10002.190628/cores/esp32
#include <Arduino.h>                // Arduino stuff
// Using the espressif arduino framework, the WiFi.h include pulls the file located here:  
// ~/.platformio/packages/framework-arduinoespressif32@2.10002.190628/libraries/WiFi/src
// See https://techtutorialsx.com/2017/06/29/esp32-arduino-getting-started-with-wifi/
// for discussion of how to use the WiFi class. Note:
// "An important point to note is that there’s an extern variable defined in the header file 
// called WiFi. This is a variable of WiFi class and as we will see, we will use it to access
// much of the WiFi functionality."
#include "WiFi.h"                   // ESP32 WiFi stuff
// Using the espressif arduino framework, the Wire.h include pulls the file located here:  
// ~/.platformio/packages/framework-arduinoespressif32@2.10002.190628/libraries/Wire/src
#include <Wire.h>                   // i2c stuff
// Using the espressif arduino framework, the SSD1306Wire.h include pulls the file located here:  
// ~/.platformio/lib/Heltec ESP32 Dev-Boards_ID6051/src/oled
// Documentation appears to be here: https://github.com/ThingPulse/esp8266-oled-ssd1306
#include "oled/SSD1306Wire.h"       // Heltec display stuff
// Using the espressif arduino framework, the SPI.h include pulls the file located here:  
// ~/.platformio/packages/framework-arduinoespressif32@2.10002.190628/libraries/SPI/src
#include <SPI.h>                    // SPI stuff
// Using the espressif arduino framework, the LoRa.h include pulls the file located here:  
// ~/.platformio/lib/Heltec ESP32 Dev-Boards_ID6051/src/lora
// #include "lora/LoRa.h"
// but we will use the RadioHead LoRa library instead, which pulls the file located here:
// ~/.platformio/lib/RadioHead_ID124
#include <RH_RF95.h>
// T2Message library (for now - transition to JLMESSAGE which is not yet complete)
#include <T2Message.h>
// Adafruit IO HTTP library is not working for us with this device configuration. 
// Can't use the Adafruit MQTT library, either. Both have a 'Sleepy Dog' dependency
// that fails. Plus, it looks like the libraries invoke WiFi101.h which is in 
// conflict with the ESP32 WiFi.h library. 
// Try the old HttpClient?  This pulls /Users/freymann/.platformio/lib/HttpClient_ID66
// But the old 'HttpClient' I've used before with Particle is completely different code. Shit. 
// #include <HttpClient.h>
// Screw it; back to ThingSpeak - 
#include "ThingSpeak.h"


// define the frequency and power; will be set 
// with calls to myRadio.methods()
#define RADIO_FREQUENCY 916.0         // 915 MHz
#define RADIO_TX_POWER 13             // ranges from xx - yy 

// HELTEC board pins from lora/LoRa.h - 
#define LORA_DEFAULT_SS_PIN     18    // **
#define LORA_DEFAULT_RESET_PIN  14    // **
#define LORA_DEFAULT_DIO0_PIN   26    // ** 
// Display reset pin is defined by a call to resetDisplay(16) in:
//  ~/.platformio/lib/Heltec ESP32 Dev-Boards_ID6051/src/oled/OLEDDisplay.cpp
#define DISPLAY_RESET_PIN       16

// These are defined in Arduino.h but bring them out for clarity
// static const uint8_t Vext = 21;
// static const uint8_t LED  = 25;
// static const uint8_t RST_OLED = 16;
// static const uint8_t SCL_OLED = 15;
// static const uint8_t SDA_OLED = 4;

// create an instance of our LoRa radio
RH_RF95 myRadio(LORA_DEFAULT_SS_PIN, LORA_DEFAULT_DIO0_PIN);              
// and an instance of the display - i2c address, SDA, SCL, RST, geometry
SSD1306Wire myDisplay(0x3c, SDA_OLED, SCL_OLED, RST_OLED);    

// T2 Message
T2Message myMsg;

// Define the sender and receiver
#define nodeAddr 0x88
#define baseAddr 0x0B

// Used to convert float uW/cm2 byte data for send and receive
union fsend_t{
  float f;
  byte c[sizeof(float)];
};

// Global Variables to hold the Received data
uint16_t batVoltage = 0;
uint16_t supVoltage = 0;

// Global Variables to hold the Received data
float     lightLeveluWcm2 = 0;

// Data buffer 
uint8_t radioBuf[(RH_RF95_MAX_MESSAGE_LEN)];

// WiFi and Adafruit IO access tokens 
#include "wifi_tokens.h"
#include "thingspeak_tokens.h"

// WiFi 
WiFiClient  client;

// ThingSpeak 
unsigned long myChannelNumber = CHANNELID;
const char * myWriteAPIKey = WRITEAPIKEY;

// trigger data display and upload
bool haveLightLevel = false; 
bool haveBatVoltage = false; 

// track signal strength
int16_t rssiValue;          // int16_t RHGenericDriver::lastRssi()
int     SNRValue;           // int RH_RF95::lastSNR()

// waiting counter
int8_t iWaiting = 0; 

// Function prototypes
void runReceiver(); 
void displayReceived();     // light levels to OLED display 
void sendReceived();        // light levels to ThingSpeak 
void displayWaiting(); 
void WIFISetUp();


// ----------------------------------------------------------

void setup() {
  
  // Manual reset 
  // See https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/rfm9x-test
  pinMode(LORA_DEFAULT_RESET_PIN, OUTPUT);
  digitalWrite(LORA_DEFAULT_RESET_PIN, HIGH);

  // Indicator 
  pinMode(LED, OUTPUT);  

  // Serial
  Serial.begin(115200);
  Serial.println(F("LoRa ESP32 testing"));

  // Radio
  myRadio.init();
  myRadio.setFrequency(RADIO_FREQUENCY);
  myRadio.setTxPower(RADIO_TX_POWER);

  // Display
  myDisplay.init();
  myDisplay.clear();

  // WiFi 
  WIFISetUp();

  // ThingSpeak
  ThingSpeak.begin(client);  

}

// ----------------------------------------------------------

void loop(){

  // Run the Receiving function
  runReceiver();            // checks for and processes incoming data

  // If new data available, display and upload
  if (haveLightLevel && haveBatVoltage){         // requires both light level and voltage
    displayReceived();      // to OLED
    sendReceived();         // to ThingSpeak
    // reset 
    haveLightLevel = false; 
    haveBatVoltage = false; 
  }
}

// ----------------------------------------------------------

void runReceiver()          // Handle incoming LoRa data feed
{
  /* RFM69 Receive */
  uint8_t radioBufLen = sizeof(radioBuf);

  union fsend_t floatConvert;

  if(myRadio.recv(radioBuf, &radioBufLen)) 
  {
    myMsg.setSerializedMessage(radioBuf, radioBufLen);

    // Message filtering based on the message headers
    if(myMsg.idx == 0x06 && myMsg.src == nodeAddr && myMsg.dst == baseAddr) {            
                             // This is for the Voltage Sender (IDX = 0x06)
      switch(myMsg.sdx)
      {
      case 0x64: // Battery Voltage
        // Concatenate 2 bytes into a uint16_t variable
        batVoltage = myMsg.data[2] << 8;
        batVoltage |= myMsg.data[3];

        // set flag
        haveBatVoltage = true; 

        // reschedule print task
        // myTaskPrint.status(JLTASK_STATUS_SCHEDULED);

        // grab the signal strength measures 
        rssiValue = myRadio.lastRssi(); 
        SNRValue = myRadio.lastSNR(); 

        break;
        
      default: // Can define an operation for everything else
        // Do something
        Serial.println(F("Unexpected message received: "));
        // myMsg.printMessage();
        break;
      }
    }  else if (myMsg.idx == 0x05 && myMsg.src == nodeAddr && myMsg.dst == baseAddr) {
                              // This is for sensors (various) (IDX = 0x05)
        switch(myMsg.sdx)
        {
        case 0x0A: // Light Levels
          // Allocate 4 bytes to the fsend_t union 
          floatConvert.c[0] = myMsg.data[0];
          floatConvert.c[1] = myMsg.data[1];        
          floatConvert.c[2] = myMsg.data[2];        
          floatConvert.c[3] = myMsg.data[3];    
          // and assign the float value 
          lightLeveluWcm2 = floatConvert.f; 

          // set flag
          haveLightLevel = true; 

          // re-schedule to print this new data 
          // myLevelsPrint.status(JLTASK_STATUS_SCHEDULED);
          
          // grab the signal strength measures 
          rssiValue = myRadio.lastRssi(); 
          SNRValue = myRadio.lastSNR(); 

          break;

        default: // Can define an operation for everything else
          // Do something
          Serial.println(F("Unexpected sensor message received: "));
          // myMsg.printMessage();
          break;
      }
    }
  }
}

void displayReceived() {     // Display data on the OLED (2s total)
    
    // Terminal output 
    Serial.println(F("### Latest Light Level Readings ###"));
    Serial.print(F("       TSL230R: ")); 
    Serial.print(lightLeveluWcm2); 
    Serial.println(F(" uW/cm2 ")); 
    // Signal strength
    Serial.print("   RSSI ");
    Serial.print(rssiValue); 
    Serial.println(" dB"); 
    Serial.print("   SNR "); 
    Serial.print(SNRValue);
    Serial.println(" dB"); 
    // Battery voltage  
    Serial.println(F("### Latest Voltage Readings ###"));
    Serial.print(F("       Battery: ")); 
    Serial.print(batVoltage); 
    Serial.println(F(" millivolts. "));
    Serial.println();

    // OLED display - first the lightlevels 
    myDisplay.drawString(0, 0, "TSL203R: ");
    myDisplay.drawString(0, 20, " " + (String)(lightLeveluWcm2) + " uW/cm2");
    myDisplay.drawString(0, 40, "With RSSI " + (String) (rssiValue) + "db");
    myDisplay.drawString(0, 50, "With SNR " + (String) (SNRValue) + "db");
    myDisplay.display();
    delay(1000);          // pause
    myDisplay.clear();
    // OLED display - then the voltage 
    myDisplay.drawString(0, 0, "Latest Voltage Readings");
    myDisplay.drawString(0, 20, "Battery: " + (String) (batVoltage) + " mV");
    myDisplay.display();
    delay(1000);          // pause
    myDisplay.clear();  
    myDisplay.display();  // clear the screen
}

void displayWaiting() {         // *** REVISE *** THIS IS NOT CLEAN. IF DELAY (500), BLOCK RECEIVING *** 
  
  if (iWaiting == 0) {
    myDisplay.drawString(0, 0, "."); 
    iWaiting++; 
  } else if (iWaiting == 1) {
    myDisplay.drawString(0, 0, "..");
    iWaiting++;
  } else if (iWaiting == 2) {
    myDisplay.drawString(0, 0, "..."); 
    iWaiting = 0; 
  }
  myDisplay.display();
  delay(5); 
  myDisplay.clear();      // clears buffer, not display
  
}

void sendReceived() {

  // set the fields with the values
  ThingSpeak.setField(1, lightLeveluWcm2);
  ThingSpeak.setField(2, batVoltage);
  ThingSpeak.setField(3, rssiValue);
  
  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }

  // testing
  delay(500); 
}

void WIFISetUp(void)      // HelTec WiFi setup code, more or less
{
	// Set WiFi to station mode and disconnect from an AP if it was previously connected
	WiFi.disconnect(true);
	delay(100);
	WiFi.mode(WIFI_STA);
	WiFi.setAutoConnect(true);
	WiFi.begin(ssid, password);
	delay(100);

	byte count = 0;
	while(WiFi.status() != WL_CONNECTED && count < 10)
	{
		count ++;
		delay(500);
		myDisplay.drawString(0, 0, "Connecting...");
		myDisplay.display();
	}

	myDisplay.clear();
	if(WiFi.status() == WL_CONNECTED)
	{
		myDisplay.drawString(0, 0, "Connecting...OK.");
    myDisplay.drawString(0,20, (String) ssid);
		myDisplay.display();
    delay(500);
	}
	else
	{
		myDisplay.drawString(0, 0, "Connecting...Failed");
		myDisplay.display();
	  delay(500);
	}
	myDisplay.drawString(0, 40, "WIFI Setup done");
	myDisplay.display();
	delay(1000);
  myDisplay.clear();
}

