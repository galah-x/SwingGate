// Sample RFM69 receiver/gateway sketch, with ACK and optional encryption, and Automatic Transmission Control
// Passes through any wireless received messages to the serial port & responds to ACKs
// It also looks for an onboard FLASH chip, if present
// **********************************************************************************
// Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID        15    //unique for each node on same network
#define GATEID        14
#define GATEWAYID      1
#define NETWORKID     100  //the same on all nodes that talk to each other
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
//#define USE_ENCRYPT   1 // should I use encryption
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!


//*********************************************************************************************
//Auto Transmission Control - dials down transmit power to save battery
//Usually you do not need to always transmit at max output power
//By reducing TX power even a little you save a significant amount of battery power
//This setting enables this gateway to work with remote nodes that have ATC enabled to
//dial their power down to only the required level
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
//*********************************************************************************************
#define SERIAL_BAUD   38400
//#define SERIAL_BAUD   115200
//#define SERIAL_BAUD   19200

#define DOSERIALIN


// basic moteino uses the atmega328

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network

#define BUFLEN 50
char buff[BUFLEN];
byte buff_pointer;
byte fetching_line;

void setup() {
  
  Serial.begin(SERIAL_BAUD);
  delay(10);
  sprintf(buff, "\nSwingGateTuner 20190309 I'm %d", NODEID);
  Serial.println(buff);

  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
#ifdef USE_ENCRYPT
  radio.encrypt(ENCRYPTKEY);
#else 
  radio.encrypt(null);
#endif
  radio.promiscuous(promiscuousMode);
  //radio.setFrequency(919000000); //set frequency to some custom frequency
  sprintf(buff, "Listening at %d Mhz... Gateway %d", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915, GATEWAYID);
  Serial.println(buff);
  if (flash.initialize())
  {
    Serial.print("SPI Flash Init OK. Unique MAC = [");
    flash.readUniqueId();
    for (byte i=0;i<8;i++)
    {
      Serial.print(flash.UNIQUEID[i], HEX);
      if (i!=8) Serial.print(':');
    }
    Serial.println(']');
    
    //alternative way to read it:
    //byte* MAC = flash.readUniqueId();
    //for (byte i=0;i<8;i++)
    //{
    //  Serial.print(MAC[i], HEX);
    //  Serial.print(' ');
    //}
  }
  else
    Serial.println("SPI Flash MEM not found (is chip soldered?)...");
    
#ifdef ENABLE_ATC
  Serial.println("RFM69_ATC Enabled (Auto Transmission Control)");
#endif

  fetching_line = 0;

}


/************************* Loop ************/



byte ackCount=0;
uint32_t packetCount = 0;
void loop() {
  
  //process any serial input
#ifdef DOSERIALIN
  if (Serial.available() > 0)
  {
    char input = Serial.read();

    if (fetching_line == 1)
      {
	if (input == '\n')
	  {
	    if (buff[0] == 'p')
	      {
		do_post();
	      }
	    if (buff[0] == 'f')
	      {
		do_flag();
	      }
      if (buff[0] == 'W')
        {
    //    sprintf(buff, "sending W %d chars\n", 5);
     //   Serial.println(buff);
         radio.sendWithRetry(GATEID, buff, 5);
        }
      if (buff[0] == 'R')
        {
//        sprintf(buff, "sending R %d chars\n", 3);
//        Serial.println(buff);
         radio.sendWithRetry(GATEID, buff, 3);
        }
	    if (buff[0] == 'X')
	      {
  //sprintf(buff, "sendin %d chars", 2);
  //Serial.println(buff);
	       radio.sendWithRetry(GATEID, buff+1, 2);
	      }
	    
	    fetching_line = 0;
	  }
	else
	  {
	    if (buff_pointer < BUFLEN)
	      {
		buff[buff_pointer++] = input;
	      }
	  }
      }
    else
      {
      
	if (input == 'r') //d=dump all register values
	  radio.readAllRegs();
	  
	//    if (input == 'E') //E=enable encryption
	//  radio.encrypt(ENCRYPTKEY);
	//if (input == 'e') //e=disable encryption
	//  radio.encrypt(null);
	
	//if (input == 'p')
	//{
	//  promiscuousMode = !promiscuousMode;
	//  radio.promiscuous(promiscuousMode);
	//  Serial.print("Promiscuous mode ");Serial.println(promiscuousMode ? "on" : "off");
	// }
	
	//if (input == 'd') //d=dump flash area
	//  {
	//    Serial.println("Flash content:");
	//    int counter = 0;
	//    
	//    while(counter<=256){
	//      Serial.print(flash.readByte(counter++), HEX);
	//      Serial.print('.');
	//    }
	//    while(flash.busy());
	//    Serial.println();
	//  }
	
	
	if (input == 't')
	  {
	    byte temperature =  radio.readTemperature(-1); // -1 = user cal factor, adjust for correct ambient
	    byte fTemp = 1.8 * temperature + 32; // 9/5=1.8
	    Serial.print( "Radio Temp is ");
	    Serial.print(temperature);
	    Serial.print("C, ");
	    Serial.print(fTemp); //converting to F loses some resolution, obvious when C is on edge between 2 values (ie 26C=78F, 27C=80F)
	    Serial.println('F');
	  }

	if ((input == 'R') || (input == 'W') || (input == 'X')) // Read/Write eerom
	  {
	    buff[0] = input; // remember command
	    fetching_line = 1;
	    buff_pointer = 1;
	  }
	  

	if (input == 'p') // POST a write to relay
	  {
	    buff[0] = input; // remember command
	    fetching_line = 1;
	    buff_pointer = 1;
	    //	    Serial.println("post req started...");
	  }
	
	if (input == 'f') // read FLAG from relay
	  {
	    buff[0] = input; // remember command
	    fetching_line = 1;
	    buff_pointer = 1;
	    //Serial.println("flag req started...");
	  }
      }
  }  // end of do_serial
#endif

  if (radio.receiveDone())
  {
//    Serial.print("#[");
//    Serial.print(++packetCount);
//    Serial.print(']');
//    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    if (promiscuousMode)
    {
      Serial.print("to [");Serial.print(radio.TARGETID, DEC);Serial.print("] ");
    }
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);
//    Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");
    
    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
//      Serial.print(" - ACK sent.");

      // When a node requests an ACK, respond to the ACK
      // and also send a packet requesting an ACK (every 3rd one only)
      // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
//      if (ackCount++%3==0)
//      if (0)
//      {
//        Serial.print(" Pinging node ");
//        Serial.print(theNodeID);
//        Serial.print(" - ACK...");
//        delay(3); //need this when sending right after reception .. ?
//        if (radio.sendWithRetry(theNodeID, "ACK TEST", 8, 0))  // 0 = only 1 attempt, no retries
//          Serial.print("ok!");
//        else Serial.print("nothing");
//      }
    }
    Serial.println();
    Blink(LED,3);
  }
}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

// post the given byte to the RELAY gateway.
// cmd format (in buff) paadd
// where p is the POST command
// aa is the address byte formatted %02x
// dd is the data byte formatted %02x 

void do_post (void)
{
  int address;
  int data;
  if (buff_pointer >= 5)
    {
      address = ((buff[1] & 0x0f) << 4 ) + (buff[2] & 0x0f);
      data  = ((buff[3] & 0x0f) << 4 ) + (buff[4] & 0x0f);
      // now reuse buff for tx
      sprintf(buff, "%02x POST%02x %c", NODEID, address, data);  
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
    }
}

// get the flag byte for the given address from the RELAY gateway.
// cmd format (in buff) faa
// where f is the FLAG command
// aa is the address byte formatted %02x

void do_flag (void)
{
  int address= 0;
  int data = 0;
  byte done;
  int len;
  unsigned long starttime;
  //Serial.println("flag req");
  if (buff_pointer >= 3)
    {
      address = ((buff[1] & 0x0f) << 4 ) + (buff[2] & 0x0f);
      //      Serial.print("flag addr=");
      // Serial.println(address, HEX);
      // now reuse buff for tx
      sprintf(buff, "%02x FLAG%02x", NODEID, address);  
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      // radio.send(GATEWAYID, buff, strlen(buff));

      starttime=millis();

      done = 0; 
      while ((200 > (millis()-starttime)) && (done==0))
	{
	  if (radio.receiveDone())
	    {

	      done=1;
	      len = radio.DATALEN;
	      if  (radio.DATALEN >= 1)
		{
		  data = radio.DATA[0];
		}
	      if (radio.ACKRequested())
		{
		  radio.sendACK();
		}
	      //	      Serial.println("packet");
	      // Serial.println(millis() - starttime);   // 9 or 10ms observed
	    }
	  //	  Serial.println("waiting");
	}

      //      Serial.println("done waiting");
      
      // now reuse buff for serial print
      sprintf(buff, "FLAG%02x = %02x", address, data);  
      Serial.println(buff);
      //sprintf(buff, "radio.receivedone %02x", done);  
      //Serial.println(buff);
      //sprintf(buff, "radio.DATALEN %02x", len);  
      //Serial.println(buff);
      //sprintf(buff, "radio.DATA[0] %02x", radio.DATA[0]);  
      //Serial.println(buff);
    }
}
