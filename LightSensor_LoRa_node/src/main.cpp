// TSL320R Light Sensor as Lora Node - implemented using Whispernode board
// dmf 6.11.20
// 
// 6.14.20 -  Finally got an auto-ranging algorithm to work. nonlinearity/saturation 
//            was a pain in the ass. all good for now. 
// 6.16.20 -  Functioning but there is an issue with saturation - to debug
// 6.16.20 -  Possible issue: the device is saturating in the shade outside. This is not the 
//            expected behavior - see notes: sunlight should not saturate at low sensitivity 
//            setting. There is some problem with the reading/calculation or with the 
//            configuration of the sensor. 
// 7.4.20 -   Converting to use the LowPower.h library. Will need substantial rethinking. 
//            OK, configured to sleep 1 minute (8s * CYCLESCOUNT seconds), do a sensitivity 
//            test, measure the light levels, and send light levels and voltage via LoRa. 
//            LEDs are on during the send. Running ok. 
// 7.5.20     Added TSL230_S0 control pin (A4). Modified setSensitivity() to include '0' and 
//            added delayMicroseconds(100) to handle recovery from Power Down. 
//            Tested hardwiring frequency divider to 10 - Issue!  with high illumination, 
//            frequency scalar to 10 stalls the program, as the ISR becomes too busy! 
// 7.5.20 -   *** Need to figure out error in light levels processing ***
//
// much of the code TSL320R based on: 
// http://home.teampaulc.org/arduino/tsl230r-to-arduino-interface
//
// Note use of the Arduino F macro for print strings - 
// "F() really isn’t a function, it is a #define macro which lives in WString.h
// WString.h:#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(PSTR(string_literal)))
// That long string of code tells the compiler to keep a string inside of PROGMEM and not allow it to consume RAM."
// https://www.baldengineer.com/arduino-f-macro.html
//
// TODO:
//      1.  Figure out light level saturation in daylight issue. 
//      2.  Consider changing frequency divider from 100 to 10 (change a pin from 3V3 to GND)
//      3.  Device saturates at ~200,000 uW/cm2 - add a flag/warning (e.g. if > 200000 set to 200000)
//          Need to break out the S0 pin 1 in order to power down sensor when sleeping (S0 S1 both LOW)
// 

// include references ~/.platformio/lib/Talk2_ID697/src/T2WhisperNode.h
#include <T2WhisperNode.h>
// include references ~/.platformio/lib/RadioHead_ID124/RH_RF95.h
#include <RH_RF95.h>
// include references ~/.platformio/lib/Low-Power_ID38/LowPower.h
#include <LowPower.h>
// include references ~/.platformio/lib/elapsedMillis_ID1002/elapsedMillis.h
#include <elapsedMillis.h>

// ---------------
// Whispernode Radio Config
#define RADIO_FREQUENCY 916.0
#define RADIO_TX_POWER 13

// create a RFM95 radio object 
RH_RF95 myRadio;

// myFlash is defined here in order to turn it off
T2Flash myFlash;

// LEDs
T2Led myLedBlue(T2_WPN_LED_1);
T2Led myLedYellow(T2_WPN_LED_2);

// Message buffer
uint8_t radioBuf[(T2_MESSAGE_HEADERS_LEN + T2_MESSAGE_MAX_DATA_LEN)];

// T2Message is a wrapper to structure messages with information to allow  
// filtering on sender, receiver, message type, etc. It is not intrinsic 
// to LoRa, but is a system T2 uses for their work based on the CAN BUS 
// protocol.  Explained here: http://talk2forum.wisen.com.au/whisper-node-temp-example-t40.html
// May be useful to maintain this structure, adapting it to whatever I might 
// need in the future. However, would anticipate (guess) it will be different 
// if move to LoRaWAN? 
// IDX - Index - type of message [use 0x05 Sensors]
// CMD - Command - what the message is representing [use 0x00 Event]
// SDX - SubIndex - what type of data (in this case) [use 0x0A and define that locally as 'Light']
// SRC - Source - use 0x88 as in the example, but whatever. Configure for pairing. 
// DST - Destination - use 0x0B as in the example, but whatever. Configure for pairing
// DATA [0-7] - [Figure this out] [See discussion of byte shifting] Idea is that you have
// to split the value into individual bytes in order to send, and then reverse the protocol
// at the receiving end. If sending a float (4-bytes) need to split those up and assign
// them to the payload. Suggests that the first byte of DATA[] should be an indicator 
// of the type of payload to follow so that it can be reconstructed properly. Or, could 
// tag with SDX so that decode could know what type of data in packet. (e.g. SDX=0x0A is uW/cm2
// which is sent as a Float). Use a Union to handle the byte conversion. 
// Discussion of issues:
// http://talk2lora.blogspot.com/2016/11/building-iot-temperature-sensor-episode_27.html
// https://www.thethingsnetwork.org/forum/t/how-to-send-float-value-and-convert-it-into-bytes/9646/8
// https://www.thethingsnetwork.org/forum/t/how-to-efficiently-assemble-and-extract-a-mix-of-integer-and-float-data-for-ttn/22908/3
// https://forum.arduino.cc/index.php?topic=391851.0
//

// T2 Message
T2Message myMsg;

// These allow filtering messages by sender/receiver 
#define nodeAddr 0x88
#define baseAddr 0x0B

// Use to convert float uW/cm2 data for send
union fsend_t{
  float f;
  byte c[sizeof(float)];
};

// ---------------
// TSL230 Config 
#define TSL230_INT        3     // TSL230 Hardware Interrupt (ARDUINO INT1)
#define TSL230_S1         A1    // TSL230 S1 Pin Control (set to GND: Low Sensitivy, Set to HIGH: High Sensitivity)
#define TSL230_S0         A4    // TSL230 S0 Pin Control (S0, S1 to GND - Power Off) 
// My test board for the TSL230 is configured: 
// S0 HIGH, S2 HIGH, S3 HIGH
// SO, S1 set the sensitivity - TSL230_S1 (Digital Pin 1) HIGH -> low sensitivity; TSL230_S1 LOW -> high sensitivity
// S2, S3 set the frequency divider (100)
// OUTPUT pin square wave is captured on Interrupt pin TSL230_INT (Digital Pin 3)
// Board uses ~3mA at 3V3 Supply 
uint8_t sensitivity;            // toggle between 1 or 100

// counters are uint32_t (unsigned long); range 0 - 4,294,967,295 (2^32 - 1); 
// max output frequency is 1MHz or 1,000,000 cycles/s; frequency is scaled by 100 on my test board
// so max frequency will be 10,000 cycles/s (10 pulses/millisecond); measured values indoor ambient 
// daytime with sensitivity set to 100 are ~2kHz, or 2000 cycles/s (2 pulses/millisecond, 500uS period), 
// and with sensitivity set to 1, ~20 Hz, or 20 cycles/s (0.02 pulses/millisecond, 50000uS period).
// (These period values for low ambient light hold when calculated explicitly in the code during tests.
// However, code-calculated values with flashlight illumination are NOT consistent!)
volatile uint32_t  pulseCount = 0;      // accumulated count in ISR; needs to be accessible for reset 
volatile uint32_t  curPulseCount = 0;   // pulses accumulated over interval; accessed to obtain frequency

elapsedMillis readTime;
uint16_t interval = 1000;               // time spent counting pulses; default 1s; available range 0 - 65s (65536 ms)

#define MININTERVAL 100                 // limit interval to vary from 100ms to 10s 
#define MAXINTERVAL 10000

uint8_t scale = 100;                    // frequency scaling; hardwired on my test board to 100 (divide-by)

// IMPORTANT - per the datasheet the Illuminance limits for the sensitivity settings:  
//                sensitivity = 1     0.2uW/cm2 -> 200k uW/cm2 
//                sensitivity = 100   0.002uW/cm2 -> 2000 uW/cm2 
// Exceeding these limits gives garbage data (e.g. as tested, if measure 
// 10,000 uW/cm2 input with sensitivity set to 100, you get 644 uW/cm2)
// This garbage makes auto-ranging difficult (e.g. if too high illuminance get
// paradoxical low pulse rate because the device saturates/fails). Bright flashlight
// illumination goes to 155000+ uW/cm2 input. Therefore, a better strategy may be to 
// make two measurements, one at each sensitivity level and set the sensitivity 
// based on consistency (for high levels) and counts (for low levels)  
#define LOWLIGHTCRIT  200.0             // uW/cm2 criteria for low level input (so, set sensitivity to 100)

#define CYCLESCOUNT 1                   // Number of cycles of sleep (so: CYCLESCOUNT * 8S sleep, effectively)
uint16_t countCycles = 0;

float uWattCm2;                         // calculated light level 
float voltageAdjust = 0.992;            // supply voltage affects frequency (datasheet Fig 5); 3V3 supply
float coverAdjust = 1.0;                // fudge factor to account for intervening (e.g cover) materials
float scaleHzToIllumination = 0.1233;   // fit to datasheet Figure 1 (but note +/-20% error expected for TLS230R)

bool readingLight; 
uint16_t readingInterval = 1000;        // milliseconds 
elapsedMillis elapsedRead; 
uint16_t processTime; 

// ---------------
// function prototypes
void sendVoltage();
void add_pulse();  
void setSensitivity(uint8_t level); 
void setInterval(uint16_t tMillis);
void adjustLevel(); 
void calcUwattCm2();
void sendIllumination();


void setup()
{
  // Serial
  Serial.begin(115200);
  Serial.println(F("LightSensor_LoRa - Node"));

  // Radio
  myRadio.init();
  myRadio.setFrequency(RADIO_FREQUENCY);
  myRadio.setTxPower(RADIO_TX_POWER);

  // Radio - put it to sleep to save energy
  // "Sets the radio into low-power sleep mode. If successful, the transport will stay in sleep
  // mode until woken by changing mode it idle, transmit or receive (eg by calling send(), recv(),
  // available() etc). Caution: there is a time penalty as the radio takes a finite time to wake 
  // from sleep mode. Return true if sleep mode was successfully entered."
  myRadio.sleep();

  // Flash - We're not using, so just power it down to save energy
  myFlash.init(T2_WPN_FLASH_SPI_CS);
  myFlash.powerDown();

  // setup for the light sensor interface
  pinMode(TSL230_INT, INPUT);       // Interrupt pin
  
  pinMode(TSL230_S0, OUTPUT);
  pinMode(TSL230_S1, OUTPUT);
  setSensitivity(1);     // initialize as low sensitivity (S0 HIGH S1 LOW)

  // begin monitoring the light sensor
  attachInterrupt(digitalPinToInterrupt(TSL230_INT), add_pulse, RISING);
  elapsedRead = 0;
  readingLight = true; 

}

void loop() {

  // a low-rent way to count periodically using low power mode cycling 
  if (countCycles == CYCLESCOUNT) {   // come out of sleep cycling
    
    // We've slept enough so reattach the interrupt. 
    attachInterrupt(digitalPinToInterrupt(TSL230_INT), add_pulse, RISING);

    // Test the level adjustment before making a reading 
    adjustLevel();        // This sets sensitivity, turning TSL230 on if in Power Down mode

    // Reset and make the reading that will be output 
    pulseCount = 0;                 // reset the count
    elapsedRead = 0;                // start light level accumulation timer now
    readingLight = true;            // flag that light level readings are active 
    countCycles = 0;                // reset

  } else if (!readingLight) {        // we are not reading, go back into sleep mode

    // Using Low-Power library to put the MCU to Sleep for 8 seconds
    // Note: this statement will corrupt anything preceding it that has
    // not completed before being called (e.g. need a delay(500) after 
    // a Serial.print statement)
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  

    countCycles++;

  } else if (elapsedRead > readingInterval) {     // process the accumulated light level data 

    interval = elapsedRead; 
    curPulseCount = pulseCount; 

    // send the data
    sendVoltage();
    sendIllumination();

    // don't want this interfering (here or with SLEEP)
    detachInterrupt(digitalPinToInterrupt(TSL230_INT)); 

    // configure to re-enter low power mode
    readingLight = false;           // no longer measuring light levels
    myRadio.sleep();                // turn off the radio 
    countCycles = 0;                // reset the sleep cycle count

    // Power down TSL230
    setSensitivity(0); 

    // need this because next call is to powerDown; allow things to settle
    delay(500); 

  } 

}

// light sensor functions 
void add_pulse() {        // ISR 
  pulseCount++;
 }

void setSensitivity(uint8_t level) {
  // TSL230R has three sensitivity levels, 1x, 10x, 100x
  // only two levels (1x, 100x) are available with my test board
  // These are toggled by changing the state of pin TSL230_S1

  noInterrupts(); 

  switch (level)
  {
    case 1:                             // S0 HIGH and S1 LOW = 1x sensitivity
      digitalWrite(TSL230_S0, HIGH); 
      digitalWrite(TSL230_S1, LOW);  
      sensitivity = 1;

      break;
    case 100:                           // S0 HIGH and S1 HIGH = 100x sensitivity
      digitalWrite(TSL230_S0, HIGH); 
      digitalWrite(TSL230_S1, HIGH);  
      sensitivity = 100;

      break;
    case 0:                           // Both S0 and S1 LOW => POWER DOWN device
      digitalWrite(TSL230_S0, LOW);
      digitalWrite(TSL230_S1, LOW); 
      sensitivity = 0;
      break; 
  }

  delayMicroseconds(100);   // TSL230 recovery time from Power Down

  curPulseCount = 0;   // reset the counts after changing sensitivity

  interrupts();   

  return;
}

void sendVoltage()
{
  uint16_t voltage = 0;

  myLedYellow.turnOn();

  // Get Voltage readings from supply/battery
  voltage = T2Utils::readVoltage(T2_WPN_VBAT_VOLTAGE, T2_WPN_VBAT_CONTROL);
  myMsg.sdx = 0x64; // Battery subIndex

  uint8_t radioBufLen = 0;

  // Prepare Message
  myMsg.cmd = 0x00;
  myMsg.idx = 0x06;
  myMsg.src = nodeAddr;
  myMsg.dst = baseAddr;
  myMsg.data[0] = 0x01; // Operation Last Reading
  myMsg.data[1] = 0x01; // Battery/Supply Index, if multiple supplies
  myMsg.data[2] = voltage >> 8;
  myMsg.data[3] = voltage;
  myMsg.len = 4; //Update length
  
  // Encode Message and get the full length
  myMsg.getSerializedMessage(radioBuf, &radioBufLen);
  
  // Send it
  myRadio.send(radioBuf, radioBufLen);
  myRadio.waitPacketSent(100);

  myLedYellow.turnOff();
}

void sendIllumination(){
    
    union fsend_t floatConvert;

    myLedBlue.turnOn(); 

    Serial.print("curr pulse count ");
    Serial.println(curPulseCount);

    calcUwattCm2();

    Serial.print("uW/cm2 ");
    Serial.println(uWattCm2);

    floatConvert.f = uWattCm2;
  
    for(byte i = 0; i < 4; i++){
     Serial.print("byte ");
      Serial.println(floatConvert.c[i]);
    }
    Serial.println(floatConvert.f);

    uint8_t radioBufLen = 0;

    // Prepare Message
    myMsg.cmd = 0x00;
    myMsg.idx = 0x05;       // sensor
    myMsg.sdx = 0x0A;       // light level 
    myMsg.src = nodeAddr;
    myMsg.dst = baseAddr;
    myMsg.data[0] = floatConvert.c[0];
    myMsg.data[1] = floatConvert.c[1];
    myMsg.data[2] = floatConvert.c[2];
    myMsg.data[3] = floatConvert.c[3];
    myMsg.len = 4; //Update length
    
    // Encode Message and get the full length
    myMsg.getSerializedMessage(radioBuf, &radioBufLen);

    // Send it
    myRadio.send(radioBuf, radioBufLen);
    myRadio.waitPacketSent(100);

    myLedBlue.turnOff(); 
}

void calcUwattCm2() {
  // convert the pulse count to light level 

  // convert to Hz and obtain the true frequency
  uint32_t frequency = (float) curPulseCount / ((float) interval / 1000.0);           
  frequency = frequency * (float) scale;                          // adjust for frequency divider                               
  frequency = frequency / (float) sensitivity;                    // adjust frequency for sensitivity level 
  frequency = frequency / voltageAdjust;                          // adjust frequency for supply voltage
  frequency = frequency / coverAdjust;                            // adjust frequency for intervening material

  // calculate to a 640nm irradiance equivalent based on datasheet Figure 1
  // From the log-log plot, the best fit is: Illumination = 0.123309146 * Hz 
  // (This is close to the factor of 10 in the template code, however note 
  // that the Illumination is already expressed in uW/cm2, so no additional
  // correction for sensor area is applied here)
  uWattCm2 = frequency * scaleHzToIllumination;

  return;
}

void adjustLevel() {
  // determine the sensitivity level with uW/cm2 using two criteria: 
  // a) the sensitivity level that gives the highest value
  //    - at high light levels the signal saturates so apparent uW/cm2 decreases
  // b) unless the highest value is < 100 uW/cm2, then sensitivity set to 100
  //    - this handles the noise in very low count low sensitivity measurements
  // Other ideas tested (counts total, count rates, etc) don't work. 

  // have to re-attach the interrupt pin for the test 
  /* attachInterrupt(digitalPinToInterrupt(TSL230_INT), add_pulse, RISING); */

  // initialize the test
  setSensitivity(1);      // pulseCount and readTime are reset here
  pulseCount = 0;         // initialize the running count 
  elapsedRead = 0;        // start timer

  while (elapsedRead < readingInterval) {
    // wait for a read to be entered in the ISR
  }

  interval = elapsedRead; 
  curPulseCount = pulseCount; 

  calcUwattCm2(); 

  float valueSetting1 = uWattCm2; 

  // initialize the second test
  setSensitivity(100);
  pulseCount = 0;   
  elapsedRead = 0;          // start timer

  while (elapsedRead < readingInterval) {
    // wait for a read to be entered in the ISR
  }
  interval = elapsedRead;
  curPulseCount = pulseCount; 

  calcUwattCm2(); 

  float valueSetting100 = uWattCm2; 

  if (max(valueSetting1, valueSetting100) < LOWLIGHTCRIT) {
    // leave at high sensitivity level
    Serial.println("Adjusting for low light (max)"); 
  } else if (valueSetting1 > valueSetting100) {
    setSensitivity(1);    // reset sensitivity
    Serial.println("Adjusting for high light "); 
  } else {
    // don't need to reset sensitivity
    Serial.println("Adjusting for low light (value)"); 
  }

  // need a delay here after the adjustment 
  delay(500);

  return; 
}

