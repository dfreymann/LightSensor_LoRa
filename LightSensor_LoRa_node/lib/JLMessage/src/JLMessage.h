// JLMessage - from T2Message 
// dmf - 6.16.20 
// Version formalizing the message header fields 
// defined to be used with Jaldi Labs programs 

// 6.16.20 in development. not at all ready for use yet

/*
 * Talk2 Protocol - Message Library
 * http://talk2.wisen.com.au
 *
 * This file is part of Talk2 Protocol - Message Library.

 * Talk2 Protocol - Message Library
 * is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Talk2 Protocol - Message Library
 * is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef JLMESSAGE_H_
#define JLMESSAGE_H_

/* External Libraries */
#include <Arduino.h>

/* Additional Libraries */

#define JL_MESSAGE_MAX_DATA_LEN 8
#define JL_MESSAGE_HEADERS_LEN 5

// Header fields 
// IDX - Index - type of message [use 0x05 Sensors]
// CMD - Command - what the message is representing [use 0x00 Event]
// SDX - SubIndex - what type of data (in this case) [use 0x0A and define that locally as 'Light']
// SRC - Source - use 0x88 as in the example, but whatever. Configure for pairing. 
// DST - Destination - use 0x0B as in the example, but whatever. Configure for pairing
// See:  http://talk2forum.wisen.com.au/whisper-node-temp-example-t40.html for usage at Talk2:
// IDX
// This determine the type of the message and we have the following standards:
// 0x00 to 0x02: Reserved
// 0x03: Non-standard. Different for each node Type/Model - High Priority
// 0x04: Actuators
// 0x05: Human Interaction Device and Sensors
// 0x06: Node: Details, Status and Configuration
// 0x07: Non-standard. Different for each node Type/Model - Low Priority
// CMD - Command
// This is used to tell the if kind of action the message is representing.
// 0x00: Event - Imagine a sensor being triggered, this will cause an "event" to be broadcasted
// 0x01: Write - This is used to change a register value, or to control an actuator for example
// 0x02: Read - This can be used to get values from a sensor, pooling information from a register
// 0x03: Return - This is the data sent back from a Read (or Write) or when you have an error after
// a Write
// SDX - Subindex
// The subindex, is used to specify exactly what object you're working with. For example, 
// a Temperature sensor would have a IDX=0x05 (Sensors) and the SIDX=0x0A (Temperature).
// You can build your own mapping of objects here. For example 0x0A for Temperature, 0x0B for 
// Hunidity, etc.
// SRC and DST
// Well, this is quite clear. But we also suggest to reserve a few values, like 0x00 or 0xFF for 
// broadcast, so all nodes that get the message will process it. But again, it might vary according 
// to your needs and implementation.

// note that IDX and CMD are packed within the first byte, e.g. 
//   this->idx = (msg[0] & 0x1C) >> 2;
//   this->cmd = msg[0] & 0x03;
//   this->sdx = msg[1];
//   this->src = msg[2];
//   this->dst = msg[3];
//   this->rtr = (msg[4] & 0x80) >> 7;
//   this->len = msg[4] & 0x7F;
//

// Defines for use with Jaldi Labs LoRa 
// IDX Index
// note that these are packed 
#define IDX_SENSOR 0x05         // sensor
// sdx SubIndex
// note that these are packed 
#define SDX_LIGHTLEVEL  0x0A    // light level
#define SDX_TEMPERATURE 0x0B    // temperature
#define SDX_HUMIDITY    0x0C    // humidity

class JLMessage
{
  public:
    JLMessage();
    virtual ~JLMessage();

    /* Load the data and headers from a array */
    void setSerializedMessage(uint8_t * msg, uint8_t len);

    /* Return all message, headers+data, and the full length */
    void getSerializedMessage(uint8_t * msg, uint8_t * len);

    /* Prints the whole message using Serial.print */
    void printMessage();

    uint8_t idx;
    uint8_t sdx;
    uint8_t cmd;
    uint8_t src;
    uint8_t dst;
    uint8_t rtr;
    uint8_t len;
    uint8_t data[JL_MESSAGE_MAX_DATA_LEN];

  private:


};

#endif /* JLMESSAGE_H_ */
