//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'SwingGate for moteino Time-stamp: "2019-01-28 09:27:24 john"';


// Given the controller boards have been destroyed by lightning for the last 2 summers running,
// going to engineer my own so I can fix it more easily

// basically I have to drive an unlock solenoid at about 1.5A for a second or so at start of open
// and I have to drive a swing linear actuator at run current of a couple of A (~15A stall) 
// interface to a pushbutton  (radio remotes bang a relay as the PB)
// and a toggle switch controlling whether to autoclose or not, how hard can it be?

// Sample RFM69 sender/node sketch for the SonarMote - Distance tracker
// Can be used for inventory control - ex to measure distance in a multi lane cigarette pack rack
// http://lowpowerlab.com/sonarmote
// Ultrasonic sensor (HC-SR04) connected to D6 (Trig), D7 (Echo), and power enabled through D5
// This sketch sleeps the Moteino and sensor most of the time. It wakes up every few seconds to take
//   a distance reading. If it detects an approaching object (car) it increases the sampling rate
//   and starts lighting up the LED (from green to yellow to red to blinking red). Once there is no more
//   motion the LED is turned off and the cycle is back to a few seconds in between sensor reads.
// Button is connected on D3. Holding the button for a few seconds enters the "red zone adjust" mode (RZA).
//   By default the red zone limit is at 25cm (LED turns RED below this and starts blinking faster and faster).
//   In RZA, readings are taken for 5 seconds. In this time you have the chance to set a new red zone limit.
//   Valid new red zone readings are between the RED__LIMIT_UPPER (default 25cm) and MAX_ADJUST_DISTANCE (cm).
//   In RZA mode the BLU Led blinks fast to indicate new red limit distance. It blinks slow if the readings are invalid
//   If desired this value could be saved to EEPROM to persist if unit is turned off
// Get the RFM69 at: https://github.com/LowPowerLab/
// Make sure you adjust the settings in the configuration section below !!!

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
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>      //included with Arduino IDE (www.arduino.cc)
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include <RFM69_OTA.h>


//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID        9    //unique for each node on same network
#define GATEWAYID     1    //node Id of the receiver we are sending data to
#define NETWORKID     100  //the same on all nodes that talk to each other including this node and the gateway
#define TAPID         4    //the TAP channel, dfor the posted off
#define FREQUENCY     RF69_915MHZ //others: RF69_433MHZ, RF69_868MHZ (this must match the RFM69 freq you have on your Moteino)
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
// #define USE_ENCRYP
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
//#define SENDLOOPS    80 //default:80 //if no message was sent for this many sleep loops/cycles, then force a send
//#define READ_SAMPLES 3
//#define HYSTERESIS   1.3  //(cm) only send a message when new reading is this many centimeters different
//#define HYSTERESIS   0  //(cm) only send a message when new reading is this many centimeters different
//#define DIST_READ_LOOPS 2 //read distance every this many sleeping loops (ie if sleep time is 8s then 2 loops => a read occurs every 16s)
//*********************************************************************************************
// #define BUZZER_ENABLE  //uncomment this line if you have the BUZZER soldered and want the sketch to make sounds
// #define SERIAL_EN      //uncomment if you want serial debugging output

#define CHECK_BATTERY

//*********************************************************************************************
#define SLEEP_FASTEST SLEEP_15MS
#define SLEEP_FAST SLEEP_250MS
#define SLEEP_SEC SLEEP_1S
#define SLEEP_LONG SLEEP_2S
#define SLEEP_LONGER SLEEP_4S
#define SLEEP_LONGEST SLEEP_8S
period_t sleepTime = SLEEP_LONGEST; //period_t is an enum type defined in the LowPower library (LowPower.h)
//period_t sleepTime = SLEEP_2S; //period_t is an enum type defined in the LowPower library (LowPower.h)
//*********************************************************************************************
#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8
#endif

// these are the IOs used for the swing gate linear actuator

#define DRN1             4  // direction pin for IBT_2 H bridge for swing motor first side
#define EN1_N            3  // pwn/enable for  IBT_2 H bridge for swing motor first side
#define DRN2             6  // direction pin for IBT_2 H bridge for swing motor second side
#define EN2_N            5  // pwn/enable for  IBT_2 H bridge for swing motor second side
#define IS1              A5 // current sense for IBT_2 H bridge for swing motor first side
#define IS2              A4 // current sense for IBT_2 H bridge for swing motor second side
#define BACKEMF1         A7 // sense motor backemf when freewheeling, 
#define BACKEMF2         A6 // sense motor backemf when freewheeling, 

// these are the IOs used for the swing gate lock (unlocker)
#define LOCK             9  // its the enable pin for the paralleled 2x2A L298 H bridge, used as a protected NFET.
                            // draws too much current (50mA) if I leave it enabled and drive the logic input active. 

// these are the IOs used for the swing gate sense inputs
#define START_STOP_N     7   // active low 'start/stop input pin
#define AUTO_CLOSE       A2  // if high, autoclose after 30 seconds
#define BATT_ADC         A3  // analog IO for measuring battery

// The general plan for the swing linear actuator is to go slowly for a couple of seconds, then increase to max for main traverse,
// then slow down again a bit before it reaches the limit. 
// There is no limit swithch, it finds the limit by measuring motor back-emf. If its stalled, back-emf is low.
// Back emf is measured by keeping the low side drive enabled, disable the high side, wait a bit,
// then measure voltage on the high side
//
// I plan to bit bang the PWM so I can synchronize measurement and drive. 100hz should be achievable and adequate.
//
// hopefully the first time it runs it can go slow all the way, and automatically figure out how long the fast traverse time
// should be for subsequent operations. 

#define DRN1_OPEN         HIGH
#define DRN1_CLOSE        LOW
#define DRN2_OPEN         LOW
#define DRN2_CLOSE        HIGH

#define LOCK_UNLOCK       HIGH
#define LOCK_LOCKED       LOW

// the battery adc sense uses a 404k resistor and 100k, for attenuation of 0.198.
// Alternatively using the 3v3 rail as reference (its a small switcher off 12)
// V = count /1024 * (3.3 / 0.1984)  or V = count * 0.01624
#define BATT_GAIN 0.01624

char buff[50]; //this is just an empty string used as a buffer to place the payload for the radio
char buff2[10]; //this is just an empty string used for float conversions

RFM69 radio;
SPIFlash flash(FLASH_SS, 0xEF30);


char current_state;

#define STATE_CLOSED           0
#define STATE_OPENING_START    1
#define STATE_OPENING_TRAVERSE 2
#define STATE_OPENING_ENDING   3
#define STATE_OPEN             4
#define STATE_CLOSING_START    5
#define STATE_CLOSING_TRAVERSE 6
#define STATE_CLOSING_ENDING   7
#define STATE_UNKNOWN          8

int traverse_runtime;
int start_runtime;
int end_runtime;

void setup() {

  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif	 
#ifdef USE_ENCRYPT
  radio.encrypt(ENCRYPTKEY);
#else
  radio.encrypt(null);
#endif
  
  radio.sleep();

  pinMode(DRN1, OUTPUT);
  pinMode(EN1_N, OUTPUT);
  pinMode(DRN2, OUTPUT);
  pinMode(EN2_N, OUTPUT);
  pinMode(LOCK, OUTPUT);
  pinMode(START_STOP_N, INPUT);
  pinMode(AUTO_CLOSE, INPUT_PULLUP);

  analogReference(DEFAULT);
  
  digitalWrite(DRN1, DRN1_OPEN);
  digitalWrite(EN1_N, LOW);
  digitalWrite(DRN2, DRN2_OPEN);
  digitalWrite(EN2_N, LOW);
  digitalWrite(LOCK, LOCK_LOCKED);

  //  radio.sendWithRetry(GATEWAYID, "START", 5);

  // radio.sleep();
}

/************************** MAIN ***************/


void loop() {
  byte light_lvl;
  unsigned int batt_adc;
  float batt_v;
  
  
  if (0) {
    // do a batt voltage update about once per day
    // do a few adc reads to let is settle. Use the last one
    batt_adc =  analogRead(BATT_ADC);
    batt_v = BATT_GAIN * batt_adc; 
    dtostrf(batt_v, 5, 2, buff2);
    sprintf(buff, "%02x Batt=%sV", NODEID, buff2  );  
    radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
    
    delay(50);
    radio.sleep();
  }
  
  // digitalWrite(LOCK,digitalRead(START_STOP_N) ^ digitalRead(AUTO_CLOSE)); 
  digitalWrite(LOCK, digitalRead(AUTO_CLOSE)); 
  
}


void Blink(byte pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(2);
  digitalWrite(pin, LOW);
}

