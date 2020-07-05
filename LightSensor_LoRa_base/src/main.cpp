// base test - implemented using Whispernode board
// dmf 6.10.20 - with annotations 
// dmf 6.15.20 - working with LightSensor_LoRa_node
// dmf 6.16.20 - ok, this is working well. changed mytask behavior so 
// that it is under control of the runReceiver() function - printing only 
// occurs when there is new data received. see comments. 
// meets expectations for transmission across our glass/concrete/metal 
// building (so far). 

/*
 * Talk2 Example: Voltage - Base
 * http://talk2.wisen.com.au
 *
 * This example demonstrate how to receive a message and print on the Serial.
 * The message to be received is the voltage readings from the Power Supply
 * and the Battery, sent by a remote node.
 * 
 * - This example works together with "Voltage - Node"
 * - The messages are formatted using the Talk2 Message frame
 * - Remember to adjust the frequency and TX Power to match your board
 *
 *  Copyright 2015-2016 by Mike Musskopf.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <T2WhisperNode.h>
#include <JLTask.h>

/* You need to configure the Whisper Node Version */
// dmf - changed
//#define T2_WPN_BOARD T2_WPN_VER_RF69
#define T2_WPN_BOARD T2_WPN_VER_LORA

#define RADIO_FREQUENCY 916.0
#define RADIO_TX_POWER 13
#define RADIO_ENCRYPTION_KEY { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F } // Only used by RF69

#if T2_WPN_BOARD == T2_WPN_VER_RF69
#include <RH_RF69.h>
RH_RF69 myRadio;
#elif T2_WPN_BOARD == T2_WPN_VER_LORA
#include <RH_RF95.h>
RH_RF95 myRadio;
#endif

// LED
T2Led myLedBlue(T2_WPN_LED_1);
T2Led myLedYellow(T2_WPN_LED_2);

// Radio
uint8_t radioBuf[(T2_MESSAGE_HEADERS_LEN + T2_MESSAGE_MAX_DATA_LEN)];

// T2 Message
T2Message myMsg;
#define nodeAddr 0x88
#define baseAddr 0x0B

// T2 Data Buffer
T2DataBuffer myDataBuffer;

// dmf - need function prototypes 
void runReceiver(); 

// Stand-alone Task
void taskPrintFunction(JLTask * Task); // Implemented at the bottom of this file
JLTask myTaskPrint(&taskPrintFunction);


// Global Variables to hold the Received data
uint16_t batVoltage = 0;
uint16_t batVoltageCounter = 0;
uint16_t supVoltage = 0;
uint16_t supVoltageCounter = 0;

// Used to convert float uW/cm2 data for send
union fsend_t{
  float f;
  byte c[sizeof(float)];
};

// Global Variables to hold the Received data
float     lightLeveluWcm2 = 0;
uint16_t  lightLevelCounter = 0;

void taskPrintLevels(JLTask * Task);    // added for light levels 
JLTask myLevelsPrint(&taskPrintLevels); 

void setup()
{
  // Serial
  Serial.begin(115200);
  Serial.println(F("Voltage - Base"));

  // Radio
  myRadio.init();
  myRadio.setFrequency(RADIO_FREQUENCY);
  myRadio.setTxPower(RADIO_TX_POWER);
#if T2_WPN_BOARD == T2_WPN_VER_RF69
  uint8_t myRadioEncryptionKey[] = RADIO_ENCRYPTION_KEY;
  myRadio.setEncryptionKey(myRadioEncryptionKey);
#endif  

  // Stand-alone Task
  myTaskPrint.runInterval(1000);                  // Task will tun every 1000 milliseconds
  myTaskPrint.runCount(JLTASK_INFINITE);          // Task continues running infinitely 
  myTaskPrint.status(JLTASK_STATUS_PAUSED);       // But Task is paused at the start

  myLevelsPrint.runInterval(1500);
  myLevelsPrint.runCount(JLTASK_INFINITE);        // Task schedule will be under control of receiver
  myLevelsPrint.status(JLTASK_STATUS_PAUSED);

  // LED Example
  myLedYellow.setBlink(20, 3000, -1);   // LED will lit for 20ms every 3000ms, forever
}

void loop()
{
  // Stand-alone Task - We run it as a non-blocking Task to print the readings
  myTaskPrint.run();
  myLevelsPrint.run(); 

  // force runReceiver control of Task status
  myTaskPrint.status(JLTASK_STATUS_PAUSED);
  myLevelsPrint.status(JLTASK_STATUS_PAUSED);


  // LED Example
  myLedBlue.run();
  myLedYellow.run();
  
  // Run the Receiving function
  runReceiver();
}

void runReceiver()
{
  /* RFM69 Receive */
  uint8_t radioBufLen = sizeof(radioBuf);

  union fsend_t floatConvert;

  if(myRadio.recv(radioBuf, &radioBufLen)) 
  {
    myMsg.setSerializedMessage(radioBuf, radioBufLen);
    // Uncomment bellow to print every received message, just be careful as
    // delays here can cause messages to be lost.
    // myMsg.printMessage();

    // We can perform some message filtering based on its headers
    // This is for the Voltage Sender (IDX = 0x06)
    if(myMsg.idx == 0x06 && myMsg.src == nodeAddr && myMsg.dst == baseAddr)
    {

      // Lets blink something
      myLedBlue.setBlink(10, 0, 1); // LED will lit for 10ms only once, so the interval doesn't matter
      
      switch(myMsg.sdx)
      {
      case 0x64: // Battery Voltage
        // Increment Counter
        batVoltageCounter++;
        // Concatenate 2 bytes into a uint16_t variable
        batVoltage = myMsg.data[2] << 8;
        batVoltage |= myMsg.data[3];

        // reschedule print task
        myTaskPrint.status(JLTASK_STATUS_SCHEDULED);

        break;

      case 0x65: // Power Supply Voltage
        // Increment Counter
        supVoltageCounter++;
        // Concatenate 2 bytes into a uint16_t variable
        supVoltage = myMsg.data[2] << 8;
        supVoltage |= myMsg.data[3];

        // reschedule print task
        myTaskPrint.status(JLTASK_STATUS_SCHEDULED);

        break;
        
      default: // Can define an operation for everything else
        // Do something
        Serial.println(F("Unexpected message received: "));
        myMsg.printMessage();
        break;
      }
    }  else if (myMsg.idx == 0x05 && myMsg.src == nodeAddr && myMsg.dst == baseAddr) {
        // and this is for sensors (IDX = 0x05)

        // as above 
        myLedBlue.setBlink(10, 0, 1); // LED will lit for 10ms only once, so the interval doesn't matter

        switch(myMsg.sdx)
        {
        case 0x0A: // Light Levels
          // Increment Counter
          lightLevelCounter++;
          
          floatConvert.c[0] = myMsg.data[0];
          floatConvert.c[1] = myMsg.data[1];        
          floatConvert.c[2] = myMsg.data[2];        
          floatConvert.c[3] = myMsg.data[3];    

          lightLeveluWcm2 = floatConvert.f; 

          // re-schedule to print this new data 
          myLevelsPrint.status(JLTASK_STATUS_SCHEDULED);
          
          break;

        default: // Can define an operation for everything else
          // Do something
          Serial.println(F("Unexpected sensor message received: "));
          myMsg.printMessage();
          break;
        }


    }



  }
}

void taskPrintFunction(JLTask * Task)
{
  // We simply print the last data we got
  Serial.println(F("### Latest Voltage Readings ###"));
  Serial.print(F("       Battery: ")); Serial.print(batVoltage); Serial.print(F(" millivolts. ")); Serial.println(batVoltageCounter);
  Serial.print(F("  Power Supply: ")); Serial.print(supVoltage); Serial.print(F(" millivolts. ")); Serial.println(supVoltageCounter);
  Serial.println();

}

void taskPrintLevels(JLTask * Task)
{
  // We simply print the last data we got
  Serial.println(F("### Latest Light Level Readings ###"));
  Serial.print(F("       TSL230R: ")); Serial.print(lightLeveluWcm2); 
  Serial.print(F(" uW/cm2 ")); 
  Serial.println(lightLevelCounter);
  // dmf 6.27.20 
  Serial.print("    SNR "); 
  Serial.print(myRadio.lastSNR());
  Serial.println(" dB"); 
  Serial.println();


  }