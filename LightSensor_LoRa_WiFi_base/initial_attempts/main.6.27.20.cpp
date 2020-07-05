// testing HELTEC board. 
// dmf 6.27.20
// 
// only thing that happens is WIFI reports connect
// and LoRa waits and does nothing. 

// For the HELTEC ESP32_LoRa board I have: 
// ESP32 Chip ID = 78A0FCBF713C
// 'license' is 0xE425455E,0x76C57FC1,0x4F2610DC,0xF431F294


#include "Arduino.h"
#include "heltec.h"
#include "WiFi.h"
#include "string.h"
#include "stdio.h"

#define BAND    915E6  //you can set band here directly,e.g. 868E6,915E6

String rssi = "RSSI --";
String packSize = "--";
String packet;

unsigned int counter = 0;

bool receiveflag = false; // software flag for LoRa receiver, received data makes it true.

long lastSendTime = 0;        // last send time
int interval = 1000;          // interval between sends
uint64_t chipid;


void WIFISetUp(void)
{
	// Set WiFi to station mode and disconnect from an AP if it was previously connected
	WiFi.disconnect(true);
	delay(100);
	WiFi.mode(WIFI_STA);
	WiFi.setAutoConnect(true);
	WiFi.begin("foo_bar","9412441646");//fill in "Your WiFi SSID","Your Password"
	delay(100);

	byte count = 0;
	while(WiFi.status() != WL_CONNECTED && count < 10)
	{
		count ++;
		delay(500);
		Heltec.display -> drawString(0, 0, "Connecting...");
		Heltec.display -> display();
	}

	Heltec.display -> clear();
	if(WiFi.status() == WL_CONNECTED)
	{
		Heltec.display -> drawString(0, 0, "Connecting...OK.");
		Heltec.display -> display();
//		delay(500);
	}
	else
	{
		Heltec.display -> clear();
		Heltec.display -> drawString(0, 0, "Connecting...Failed");
		Heltec.display -> display();
		//while(1);
	}
	Heltec.display -> drawString(0, 10, "WIFI Setup done");
	Heltec.display -> display();
	delay(500);
}

void WIFIScan(unsigned int value)
{
	unsigned int i;
    WiFi.mode(WIFI_STA);

	for(i=0;i<value;i++)
	{
		Heltec.display -> drawString(0, 20, "Scan start...");
		Heltec.display -> display();

		int n = WiFi.scanNetworks();
		Heltec.display -> drawString(0, 30, "Scan done");
		Heltec.display -> display();
		delay(500);
		Heltec.display -> clear();

		if (n == 0)
		{
			Heltec.display -> clear();
			Heltec.display -> drawString(0, 0, "no network found");
			Heltec.display -> display();
			//while(1);
		}
		else
		{
			Heltec.display -> drawString(0, 0, (String)n);
			Heltec.display -> drawString(14, 0, "networks found:");
			Heltec.display -> display();
			delay(500);

			for (int i = 0; i < n; ++i) {
			// Print SSID and RSSI for each network found
				Heltec.display -> drawString(0, (i+1)*9,(String)(i + 1));
				Heltec.display -> drawString(6, (i+1)*9, ":");
				Heltec.display -> drawString(12,(i+1)*9, (String)(WiFi.SSID(i)));
				Heltec.display -> drawString(90,(i+1)*9, " (");
				Heltec.display -> drawString(98,(i+1)*9, (String)(WiFi.RSSI(i)));
				Heltec.display -> drawString(114,(i+1)*9, ")");
				//            display.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
				delay(10);
			}
		}

		Heltec.display -> display();
		delay(800);
		Heltec.display -> clear();
	}
}

bool resendflag=false;
bool deepsleepflag=false;
void interrupt_GPIO0()
{
  delay(10);

  // dmf 
  Serial.println("in interrrupt_GPIO"); 

  if(digitalRead(0)==0)
  {
      if(digitalRead(LED)==LOW)
      {resendflag=true;}
      else
      {
        deepsleepflag=true;
      }     
  }
}

void send()
{
    LoRa.beginPacket();
    LoRa.print("hello ");
    LoRa.print(counter++);
    LoRa.endPacket();
}
void displaySendReceive()
{
    Heltec.display -> drawString(0, 50, "Packet " + (String)(counter-1) + " sent done");
    Heltec.display -> drawString(0, 0, "Received Size  " + packSize + " packages:");
    Heltec.display -> drawString(0, 10, packet);
    Heltec.display -> drawString(0, 20, "With " + rssi + "db");
    Heltec.display -> display();
    delay(100);
    Heltec.display -> clear();
}
void onReceive(int packetSize)//LoRa receiver interrupt service
{
	//if (packetSize == 0) return;

	// dmf 
	Serial.println("in onReceive"); 

	packet = "";
    packSize = String(packetSize,DEC);

    while (LoRa.available())
    {
		packet += (char) LoRa.read();
    }

    Serial.println(packet);
    rssi = "RSSI: " + String(LoRa.packetRssi(), DEC);    
    receiveflag = true;    
}


#define LED 25
char Readback[50];

void setup()
{
	Heltec.begin(true /*DisplayEnable Enable*/, true /*LoRa Enable*/, true /*Serial Enable*/, true /*LoRa use PABOOST*/, BAND /*LoRa RF working band*/);

	delay(300);
	Heltec.display -> clear();

	// dmf 6.27.20 required for compatibility with RH_RF95 configuration used with WhisperNode
	// see https://appcodelabs.com/using-a-software-defined-radio-to-debug-lora-communication-problems
	LoRa.setSyncWord(18);	// 0x12
	LoRa.enableCrc(); 

	WIFISetUp();
	WiFi.disconnect(); //重新初始化WIFI
	WiFi.mode(WIFI_STA);
	delay(100);

	WIFIScan(1);

	chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
	Serial.printf("ESP32ChipID=%04X",(uint16_t)(chipid>>32));//print High 2 bytes
	Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.

  	attachInterrupt(0,interrupt_GPIO0,FALLING);
	LoRa.onReceive(onReceive);
  	send();
	// dmf
	LoRa.receive(1); 
  	// LoRa.receive();
  	displaySendReceive();

	pinMode(LED,OUTPUT);
  	digitalWrite(LED,LOW); 

	// dmf   
	Serial.println("end of Setup()");
}


void loop()
{
 if(deepsleepflag)
 {
  // dmf 
  Serial.println("in deepsleepflag");

  LoRa.end();
  LoRa.sleep();
  delay(100);
  pinMode(4,INPUT);
  pinMode(5,INPUT);
  pinMode(14,INPUT);
  pinMode(15,INPUT);
  pinMode(16,INPUT);
  pinMode(17,INPUT);
  pinMode(18,INPUT);
  pinMode(19,INPUT);
  pinMode(26,INPUT);
  pinMode(27,INPUT);
  digitalWrite(Vext,HIGH);
  delay(2);
  esp_deep_sleep_start();
 }
 if(resendflag)
 {
	// dmf 
  	Serial.println("in resendflag");
  
   resendflag=false;
   send();      
   LoRa.receive();
   displaySendReceive();
 }
 if(receiveflag)
 {
	// dmf 
  	Serial.println("in receiveflag");
  
    digitalWrite(25,HIGH);
    displaySendReceive();
    delay(1000);
    receiveflag = false;  
    send();
    LoRa.receive();
    displaySendReceive();
  }
   // try to parse packet
  int packetSize = LoRa.parsePacket();

  // dmf 
  // Serial.println("looping...");  ok, loops. 

  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");
    // read packet
    while (LoRa.available()) {
	  sprintf(Readback+strlen(Readback),"%c",(char)LoRa.read());
    }
    Serial.print(Readback);
	if(strncmp(Readback, "OpenLED", strlen(Readback)) == 0) {
		digitalWrite(LED, HIGH); 
	 }
	else if(strncmp(Readback, "CloseLED", strlen(Readback)) == 0) {
		digitalWrite(LED, LOW); 
	 }
	 memset(Readback,0,50);
    // print RSSI of packet
    Serial.print(" with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}

/* 
  Check the new incoming messages, and print via serialin 115200 baud rate.
  
  by Aaron.Lee from HelTec AutoMation, ChengDu, China
  成都惠利特自动化科技有限公司
  www.heltec.cn
  
  this project also realess in GitHub:
  https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
*/

