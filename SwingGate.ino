//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'SwingGate for moteino Time-stamp: "2019-01-28 18:37:38 john"';


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

const byte DRN1     = 4;  // direction pin for IBT_2 H bridge for swing motor first side
const byte EN1      = 3;  // pwn/enable for  IBT_2 H bridge for swing motor first side
const byte DRN2     = 6;  // direction pin for IBT_2 H bridge for swing motor second side
const byte EN2      = 5;  // pwn/enable for  IBT_2 H bridge for swing motor second side
const byte IS1      = A5; // current sense for IBT_2 H bridge for swing motor first side
const byte IS2      = A4; // current sense for IBT_2 H bridge for swing motor second side
const byte BACKEMF2 = A7; // sense motor backemf when freewheeling, 
const byte BACKEMF1 = A6; // sense motor backemf when freewheeling, 

// these are the IOs used for the swing gate lock (unlocker)
const byte LOCK     = 9;  // its the enable pin for the paralleled 2x2A L298 H bridge, used as a protected NFET.
// draws too much current (50mA) if I leave it enabled and drive the logic input active. 

// these are the IOs used for the swing gate sense inputs
const byte START_STOP_N = 7;   // active low 'start/stop input pin
const byte AUTO_CLOSE   = A2;  // if high, autoclose after 30 seconds
const byte BATT_ADC     = A3;  // analog IO for measuring battery

const byte RUN_DIRECTION = A2;  // for testing simplified motor ver
const byte RUN_FWD = 1;
const byte RUN_BWD = 0;

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

#define DRN1_OPEN         1
#define DRN1_CLOSE        0
#define DRN2_OPEN         0
#define DRN2_CLOSE        1

#define PWM_ON            1
#define PWM_OFF           0

#define LOCK_UNLOCK       1
#define LOCK_LOCKED       0

// the battery adc sense uses a 404k resistor and 100k, for attenuation of 0.198.
// Alternatively using the 3v3 rail as reference (its a small switcher off 12)
// V = count /1024 * (3.3 / 0.1984)  or V = count * 0.01624
#define BATT_GAIN 0.01624

char buff[50]; //this is just an empty string used as a buffer to place the payload for the radio
char buff2[10]; //this is just an empty string used for float conversions

RFM69 radio;
SPIFlash flash(FLASH_SS, 0xEF30);

const byte ANA_FILTER_TERMS = 4;
const unsigned int  bemf_min_val = 1500; // 300*5
const unsigned int  current_max_val = 1000; // 200*5
const unsigned int bemf_init_val = 500;
const unsigned int current_init_val = 0;
unsigned int bemf[ANA_FILTER_TERMS];
unsigned int is[ANA_FILTER_TERMS];
byte filt_pointer = 0;

byte state;

#define STATE_CLOSED           0
#define STATE_OPENING_START    1
#define STATE_OPENING_TRAVERSE 2
#define STATE_OPENING_ENDING   3
#define STATE_OPEN             4
#define STATE_CLOSING_START    5
#define STATE_CLOSING_TRAVERSE 6
#define STATE_CLOSING_ENDING   7
#define STATE_UNKNOWN          8

// play with DC motor to get started
const byte STATE_STOPPED = 100;
const byte STATE_UNLOCK  = 101;
const byte STATE_BWD_START = 102;
const byte STATE_ACCEL = 103;
const byte STATE_RUN = 104;
const byte STATE_RUN_SLOW = 105;
const byte STATE_TO_STOP = 106;



int traverse_runtime;
int start_runtime;
int end_runtime;

byte drn_enable;
unsigned int runtime;  
unsigned int ontime;
unsigned int offtime;
unsigned int  on_current;
unsigned int  back_emf;
unsigned int  biggest_on_current_seen;
unsigned int  smallest_back_emf_seen;
unsigned int  on_current_max;
unsigned int  back_emf_min;
byte on_current_pin;
byte back_emf_pin;
unsigned int ticks;
unsigned int mask_input;
unsigned int run_state;

const byte mask_input_period = 100 ; // 1 second if pwm loop is 10ms 

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
  pinMode(EN1, OUTPUT);
  pinMode(DRN2, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(LOCK, OUTPUT);
  pinMode(START_STOP_N, INPUT);
  pinMode(AUTO_CLOSE, INPUT_PULLUP);

  analogReference(DEFAULT);
  
  digitalWrite(DRN1, DRN1_OPEN);
  digitalWrite(EN1,  PWM_OFF);
  digitalWrite(DRN2, DRN2_OPEN);
  digitalWrite(EN2,  PWM_OFF);
  digitalWrite(LOCK, LOCK_LOCKED);

  //  radio.sendWithRetry(GATEWAYID, "START", 5);

  // radio.sleep();
  state = STATE_STOPPED;
  ticks=0;
  mask_input=0;
  filt_pointer = 0;
}

/************************** MAIN ***************/


void loop() {
  unsigned int batt_adc;
  float batt_v;
  byte i;
  if (0) {
    // do a batt voltage update about once per day
    // do a few adc reads to let is settle. Use the last one
  }
  
  // digitalWrite(LOCK,digitalRead(START_STOP_N) ^ digitalRead(AUTO_CLOSE)); 

  ticks++;
  if (mask_input)
    {
      mask_input--;
    }
  
  // basic pwm forward routine
  if (runtime > 0)
    {
      // on part of pwm cycle
      digitalWrite(drn_enable, PWM_ON);
      delay(ontime);
      // implement running average filter... read oldest
      on_current = is[filt_pointer];
      // replace oldest with new term
      is[filt_pointer] = analogRead(on_current_pin); 
      // sum all the stored terms. don't bother to average, scale the limit.
      for (i=0; i < ANA_FILTER_TERMS; i++) {
	on_current += is[i];
      }
      // off part of pwm cycle
      digitalWrite(drn_enable, PWM_OFF);
      delay(1);
      // implement running average filter... read oldest
      back_emf = bemf[filt_pointer];
      // replace oldest with new term
      bemf[filt_pointer] = analogRead(back_emf_pin);      
      // sum all the stored terms. don't bother to average, scale the limit.
      for (i=0; i < ANA_FILTER_TERMS; i++) {
	back_emf += bemf[i];
      }
      // peak detect the problem values. Skip the basic initial acceleration from stopped phase, which is just a second long
      if (!((state == STATE_UNLOCK) || (state == STATE_BWD_START)))
	{
	  if (on_current > biggest_on_current_seen)
	    {
	      biggest_on_current_seen = on_current;
	    }
	  
	  if (back_emf < smallest_back_emf_seen)
	    {
	      smallest_back_emf_seen = back_emf;
	    }
	}
      if (filt_pointer == (ANA_FILTER_TERMS -1) )
	{
	  filt_pointer = 0;
	} else {
	filt_pointer++;
      }
      delay(offtime);
      
      runtime--;
      if (runtime == 0) {
	update_timed_state();
      }
      if  (((digitalRead(START_STOP_N) == 0) && (mask_input == 0))
	   && (!((state != STATE_UNLOCK) ||  (state != STATE_BWD_START))  
	       && ((back_emf < back_emf_min) || (on_current > on_current_max)))
	   )
	{
	  update_error_state();
	}
    }
  else
    {
      delay(ontime+offtime+1);
      if ((digitalRead(START_STOP_N) == 0) && (mask_input == 0))
	{
	  update_error_state();
	}
    }
}

const byte SLOW_ONTIME = 2; 
const byte SLOW_OFFTIME = 7;   // + 1 for stabilize backemf measure 

const byte MED_ONTIME = 6; 
const byte MED_OFFTIME = 3;   // + 1 for stabilize backemf measure 

const byte FAST_ONTIME = 9; 
const byte FAST_OFFTIME = 0;   // + 1 for stabilize backemf measure 


// update the state variables 
void update_timed_state(void)
{
  unsigned int batt_adc;
  float batt_v;
  switch (state)
    {
    case STATE_STOPPED :
      break;

    case STATE_UNLOCK :
      digitalWrite(LOCK, LOCK_LOCKED);
      state = STATE_ACCEL;
      runtime = 300;
      break;
      
    case STATE_BWD_START :
      digitalWrite(LOCK, LOCK_LOCKED);
      state = STATE_ACCEL;
      runtime = 300;
      break;
      
    case STATE_ACCEL :
      state = STATE_RUN;
      ontime = MED_ONTIME;
      offtime = MED_OFFTIME;
      runtime = 50;
      break;

    case STATE_RUN :
      state = STATE_RUN_SLOW;
      ontime = FAST_ONTIME;
      offtime = FAST_OFFTIME;
      runtime = 500;
      break;

    case STATE_RUN_SLOW :
      state = STATE_TO_STOP;
      runtime = 500;
      ontime = SLOW_ONTIME;
      offtime = SLOW_OFFTIME;
      break;
    
    case STATE_TO_STOP :
      state = STATE_STOPPED;
      runtime = 0;
      sprintf(buff,"%02x minBEMF %d (%d) maxI %d (%d)", NODEID,
	      smallest_back_emf_seen, back_emf_min,
	      biggest_on_current_seen, on_current_max);
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      delay(100);
      batt_adc =  analogRead(BATT_ADC);
      batt_v = BATT_GAIN * batt_adc; 
      dtostrf(batt_v, 5, 2, buff2);
      sprintf(buff, "%02x Batt=%sV", NODEID, buff2  );  
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      radio.sleep();
      break;
    }
}

      
void update_error_state(void)
{
  int i;
  switch (state)
    {
    case STATE_STOPPED :
      if (digitalRead(START_STOP_N) == 0)
	{
	  mask_input = mask_input_period;
	  if (digitalRead(RUN_DIRECTION) == RUN_FWD)
	    {
	      state = STATE_UNLOCK;
	      runtime = 100;
	      drn_enable = EN1 ;
	      on_current_pin = IS1 ;
	      back_emf_pin = BACKEMF1;
	      biggest_on_current_seen = 0;
	      smallest_back_emf_seen  = -1;
	      
	      back_emf_min = bemf_min_val ;
	      on_current_max = current_max_val ;
	      digitalWrite(DRN1, 1);
	      digitalWrite(EN1, PWM_OFF);
	      digitalWrite(DRN2, 0);
	      digitalWrite(EN2, PWM_ON);
	      digitalWrite(LOCK, LOCK_UNLOCK);
	      
	    } else {
	    // RUN_BWD
	    state = STATE_BWD_START;
	    runtime = 100;
	    drn_enable = EN2 ;
	    on_current_pin = IS2 ;
	    back_emf_pin = BACKEMF2;
	      
	    back_emf_min = bemf_min_val ;
	    on_current_max = current_max_val ;
	    digitalWrite(DRN1, 0);
	    digitalWrite(EN1, PWM_ON);
	    digitalWrite(DRN2, 1);
	    digitalWrite(EN2, PWM_OFF);
	  }
	  ontime = SLOW_ONTIME;
	  offtime = SLOW_OFFTIME;
	  for (i=0; i < ANA_FILTER_TERMS; i++) {
	    bemf[i] = bemf_init_val;
	    is[i]   = current_init_val;
	  }
	}
      break;

    case STATE_UNLOCK :
      digitalWrite(LOCK, LOCK_LOCKED);
      
      if ((digitalRead(START_STOP_N) == 0) 
	  //	  || (back_emf < back_emf_min) 
	  //      || (on_current > on_current_max)
	  )
	{
	  sprintf(buff,"%02x unlock %d (%d) %d (%d)", NODEID, back_emf, back_emf_min, on_current, on_current_max);
	  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
	  radio.sleep();
	  state = STATE_STOPPED;
	  mask_input = mask_input_period;
	  digitalWrite(EN1, PWM_OFF);
	  digitalWrite(EN2, PWM_OFF);
	  runtime=0;
	}
      break;
      
    case STATE_BWD_START :
      if ((digitalRead(START_STOP_N) == 0) 
	  //	  || (back_emf < back_emf_min) 
	  //      || (on_current > on_current_max)
	  )
	{
	  sprintf(buff,"%02x bwd start %d (%d) %d (%d)", NODEID, back_emf, back_emf_min, on_current, on_current_max);
	  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
	  radio.sleep();
	  state = STATE_STOPPED;
	  mask_input = mask_input_period;
	  digitalWrite(EN1, PWM_OFF);
	  digitalWrite(EN2, PWM_OFF);
	  runtime=0;
	}
      break;
      
    case STATE_ACCEL :
      if ((digitalRead(START_STOP_N) == 0) 
	  || (back_emf < back_emf_min) 
	  || (on_current > on_current_max)
	  )
	{
	  sprintf(buff,"%02x accel %d (%d) %d (%d)", NODEID, back_emf, back_emf_min, on_current, on_current_max);
	  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
	  radio.sleep();
	  state = STATE_STOPPED;
	  mask_input = mask_input_period;
	  digitalWrite(EN1, PWM_OFF);
	  digitalWrite(EN2, PWM_OFF);
	  runtime=0;
	}
      
      break;
    case STATE_RUN :
      if ((digitalRead(START_STOP_N) == 0) 
	  || (back_emf < back_emf_min) 
	  || (on_current > on_current_max)
	  )
	{
	  sprintf(buff,"%02x run %d (%d) %d (%d)", NODEID, back_emf, back_emf_min, on_current, on_current_max);
	  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
	  radio.sleep();
	  state = STATE_STOPPED;
	  mask_input = mask_input_period;
	  digitalWrite(EN1, PWM_OFF);
	  digitalWrite(EN2, PWM_OFF);
	  runtime=0;
	}
      
      break;

    case STATE_RUN_SLOW :
      if ((digitalRead(START_STOP_N) == 0) 
	  || (back_emf < back_emf_min) 
	  || (on_current > on_current_max)
	  )
	{
	  sprintf(buff,"%02x run slow %d (%d) %d (%d)", NODEID, back_emf, back_emf_min, on_current, on_current_max);
	  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
	  radio.sleep();
	  state = STATE_STOPPED;
	  mask_input = mask_input_period;
	  digitalWrite(EN1, PWM_OFF);
	  digitalWrite(EN2, PWM_OFF);
	  runtime=0;
	}
      break;
    
    case STATE_TO_STOP :
      if ((digitalRead(START_STOP_N) == 0) 
	  || (back_emf < back_emf_min) 
	  || (on_current > on_current_max)
	  )
	{
	  state = STATE_STOPPED;
	  mask_input = mask_input_period;
	  digitalWrite(EN1, PWM_OFF);
	  digitalWrite(EN2, PWM_OFF);
	  runtime=0;
	}
      
      sprintf(buff,"%02x to_stop %d (%d) %d (%d)", NODEID, back_emf, back_emf_min, on_current, on_current_max);
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));

      radio.sleep();
      break;
    }
}
  
      
	      
	      
  
void Blink(byte pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(2);
  digitalWrite(pin, LOW);
}

