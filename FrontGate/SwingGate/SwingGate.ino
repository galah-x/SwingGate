//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'SwingGate for moteino Time-stamp: "2020-04-18 14:14:29 john"';


// Given the controller boards have been destroyed by lightning for the last 2 summers running,
// going to engineer my own so I can fix it more easily

// basically I have to drive an unlock solenoid at about 1.5A for a second or so at start of open
// optionally, when the lock pin is fitted.
// Also the lock bracket the pin goes into is a royal pain. As the gate moves aroud in wet weather,
// it jams and clashes.
//
// and I have to drive a swing linear actuator at run current of a couple of A (~15A stall) 
// interface to a pushbutton  (radio remotes bang a relay pretending to be the PB)
// and a toggle switch controlling whether to autoclose or not,
// how hard can it be?

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>      //included with Arduino IDE (www.arduino.cc)
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include <RFM69_OTA.h>
#include <EEPROM.h>


//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************

// define this for current proto build front gate
// #define PROTOTYPE

// define this for the back gate, not the front gate
// #define BACKGATE
#include "whichgate.h"


// add printfs at state boundaries. hiccups 
#define DEBUG


#ifdef FRONTGATE
#define NODEID        9    //frontgate unique for each node on same network
#endif
#ifdef BACKGATE
#define NODEID        14    // backgate unique for each node on same network
#endif

#define GATEWAYID     1    //node Id of the receiver we are sending data to
#define NETWORKID     100  //the same on all nodes that talk to each other including this node and the gateway
#define FREQUENCY     RF69_915MHZ //others: RF69_433MHZ, RF69_868MHZ (this must match the RFM69 freq you have on your Moteino)
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
// #define USE_ENCRYP
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!

//*********************************************************************************************
#ifdef __AVR_ATmega1284P__
#define LED           15 // Moteino MEGAs have LEDs on D15
#define FLASH_SS      23
#else
#define LED           9 // Moteinos have LEDs on D9
#define FLASH_SS      8
#endif


#define AUTOCLOSE_SKIPS_UNLOCK




// these are the IOs used for the swing gate linear actuator
// for the prototype
const uint8_t DRN1     = 4;  // direction pin for IBT_2 H bridge for swing motor first side
const uint8_t EN1      = 3;  // pwn/enable for  IBT_2 H bridge for swing motor first side
const uint8_t DRN2     = 6;  // direction pin for IBT_2 H bridge for swing motor second side
const uint8_t EN2      = 5;  // pwn/enable for  IBT_2 H bridge for swing motor second side
const uint8_t IS1      = A5; // current sense for IBT_2 H bridge for swing motor first side
const uint8_t IS2      = A4; // current sense for IBT_2 H bridge for swing motor second side

#ifdef PROTOTYPE
// for the lashed together prototype
 const uint8_t BACKEMF2 = A7; // sense motor backemf when freewheeling, 
 const uint8_t BACKEMF1 = A6; // sense motor backemf when freewheeling, 
#else
// for the r1 PCB
 const uint8_t BACKEMF2 = A6; // sense motor backemf when freewheeling, 
 const uint8_t BACKEMF1 = A7; // sense motor backemf when freewheeling, 
#endif

// these are the IOs used for the swing gate lock (unlocker)
const uint8_t LOCK     = 9;  // its the enable pin for the paralleled 2x2A L298 H bridge, used as a protected NFET.
// draws too much current (50mA) if I leave it enabled and drive the logic input active. 

// these are the IOs used for the swing gate sense inputs
const uint8_t START_STOP_N = 7;   // active low 'start/stop input pin
const uint8_t AUTO_CLOSE   = A2;  // if high, autoclose after 30 seconds
const uint8_t BATT_ADC     = A3;  // analog IO for measuring battery

const uint8_t PROXIMITY_PWR = A0;  // enable open and close detectors
const uint8_t PROXIMITY_N   = A1;  // proximity detectors.. its an open collector output, so low = detected.

// The general plan for the swing linear actuator is to go slowly for a couple of seconds, then increase to max for main traverse,
// then slow down again a bit before it reaches the limit. 
// There is no limit switch, it finds the limit by measuring motor back-emf. If its stalled, back-emf is low.
// Back emf is measured by keeping the low side drive enabled, disable the high side, wait a bit,
// then measure voltage on the high side
//
// I plan to bit bang the PWM so I can synchronize measurement and drive. 100hz should be achievable and adequate.
//
// hopefully the first time it runs it can go slow all the way, and automatically figure out how long the fast traverse time
// should be for subsequent operations.

// maybe allow the radio to open a closed gate, and change the autoclose sw state. Probably only when idle. 

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

char buff[60]; //this is just an empty string used as a buffer to place the payload for the radio
char buff2[10]; //this is just an empty string used for float conversions

RFM69 radio;
SPIFlash flash(FLASH_SS, 0xEF30);

const uint8_t ANA_FILTER_TERMS = 4;

// Hi/lo as these are 16 bit unsigned quantities
// slow/fast for when the gate is traversing at full or limited PWMd speed
// open/close depending on which way its going.

// assign EEPROM addresses
#ifdef USE_EEPROM
const uint8_t EEPROM_initialized_loc = 0;
const uint8_t EEPROM_initialized_val = 0x5c;
const uint8_t EEPROM_loc_hi_slow_open_bemf_min = 1;
const uint8_t EEPROM_loc_lo_slow_open_bemf_min = 2;
const uint8_t EEPROM_loc_hi_slow_open_current_max = 3;
const uint8_t EEPROM_loc_lo_slow_open_current_max = 4;
const uint8_t EEPROM_loc_hi_fast_open_bemf_min = 5;
const uint8_t EEPROM_loc_lo_fast_open_bemf_min = 6;
const uint8_t EEPROM_loc_hi_fast_open_current_max = 7;
const uint8_t EEPROM_loc_lo_fast_open_current_max = 8;
const uint8_t EEPROM_loc_hi_bemf_init = 9;
const uint8_t EEPROM_loc_lo_bemf_init = 10;
const uint8_t EEPROM_loc_hi_current_init = 11;
const uint8_t EEPROM_loc_lo_current_init = 12;
const uint8_t EEPROM_loc_hi_slow_close_bemf_min = 13;
const uint8_t EEPROM_loc_lo_slow_close_bemf_min = 14;
const uint8_t EEPROM_loc_hi_slow_close_current_max = 15;
const uint8_t EEPROM_loc_lo_slow_close_current_max = 16;
const uint8_t EEPROM_loc_hi_fast_close_bemf_min = 17;
const uint8_t EEPROM_loc_lo_fast_close_bemf_min = 18;
const uint8_t EEPROM_loc_hi_fast_close_current_max = 19;
const uint8_t EEPROM_loc_lo_fast_close_current_max = 20;
#endif

#ifdef BACKGATE
  #define slow_open_bemf_min_val     1950
  #define slow_open_current_max_val  1000
  #define fast_open_bemf_min_val     2500
  #define fast_open_current_max_val  1800
  #define slow_close_bemf_min_val     1950
  #define slow_close_current_max_val  1000
  #define fast_close_bemf_min_val     2500
  #define fast_close_current_max_val  1800
#else 
  #define slow_open_bemf_min_val     1800
  #define slow_close_bemf_min_val    1200
  #define fast_open_bemf_min_val     2500
  #define fast_close_bemf_min_val    2000
  #define slow_open_current_max_val  900
  #define slow_close_current_max_val 1000
  #define fast_open_current_max_val  1450
  #define fast_close_current_max_val 1750
#endif
  #define bemf_init_val  500
  #define current_init_val  0 




#ifdef USE_EEPROM

#define fast_close_I_max (EEPROM.read(EEPROM_loc_hi_fast_close_current_max) << 8 + EEPROM.read(EEPROM_loc_lo_fast_close_current_max))
#define slow_close_I_max (EEPROM.read(EEPROM_loc_hi_slow_close_current_max) << 8 + EEPROM.read(EEPROM_loc_lo_slow_close_current_max))
#define fast_open_I_max  (EEPROM.read(EEPROM_loc_hi_fast_open_current_max) << 8 + EEPROM.read(EEPROM_loc_lo_fast_open_current_max))
#define slow_open_I_max  (EEPROM.read(EEPROM_loc_hi_slow_open_current_max) << 8 + EEPROM.read(EEPROM_loc_lo_slow_open_current_max))

#define fast_close_BEMF_min (EEPROM.read(EEPROM_loc_hi_fast_close_bemf_min) << 8 + EEPROM.read(EEPROM_loc_lo_fast_close_bemf_min))
#define slow_close_BEMF_min (EEPROM.read(EEPROM_loc_hi_slow_close_bemf_min) << 8 + EEPROM.read(EEPROM_loc_lo_slow_close_bemf_min))
#define fast_open_BEMF_min  (EEPROM.read(EEPROM_loc_hi_fast_open_bemf_min) << 8 + EEPROM.read(EEPROM_loc_lo_fast_open_bemf_min))
#define slow_open_BEMF_min  (EEPROM.read(EEPROM_loc_hi_slow_open_bemf_min) << 8 + EEPROM.read(EEPROM_loc_lo_slow_open_bemf_min))

#else
#define fast_close_I_max    fast_close_current_max_val  
#define slow_close_I_max    slow_close_current_max_val
#define fast_open_I_max     fast_open_current_max_val 
#define slow_open_I_max     slow_open_current_max_val 
			  
#define fast_close_BEMF_min fast_close_bemf_min_val    
#define slow_close_BEMF_min slow_close_bemf_min_val   
#define fast_open_BEMF_min  fast_open_bemf_min_val    
#define slow_open_BEMF_min  slow_open_bemf_min_val      
#endif


uint16_t bemf[ANA_FILTER_TERMS];
uint16_t im[ANA_FILTER_TERMS];
uint8_t filt_pointer = 0;

// which way is the motor currently moving / moved last time
char last_drn;
const uint8_t DRN_CLOSING = 'C';
const uint8_t DRN_OPENING = 'O';

// is the gate closed?
bool closed;   // true == closed

// did someone hit the button except to start it initially.
uint8_t buttoned;
const uint8_t MANUAL = 1;
const uint8_t AUTO = 0;

uint8_t radio_autoclose;
uint8_t radio_start;

uint8_t state;

// controls motion
const uint8_t STATE_STOPPED      = 0;  // not running 
const uint8_t STATE_REV          = 1;  // get started, slowly, in reverse to let the lock open.
                                    // Ignore stall currents
const uint8_t STATE_START        = 2;  // get started, slowly. Ignore stall currents
const uint8_t STATE_ACCEL        = 3;  // accelerate. higher current limit
const uint8_t STATE_RUN_FAST     = 4;  // middle fast traverse. higher current limit
const uint8_t STATE_RUN_SLOW     = 5;  // go slowly.  Check current+bemf limys,
                                       // its intended the limit gets hit in this state
const uint8_t STATE_REACHED_LIMIT = 6; // limit swich found.  
const uint8_t STATE_MISSED_LIMIT = 7;  // problem, no limit found. timeout to here. 

// in general, from closed, the normal flow was
// STOPPED --pb--> START --time-> ACCEL --time-> RUN_FAST --time-> RUN_SLOW --stalled-> STOPPED 
// but to make the lock work, I have to take pressure off it by running backwards briefly.
// STOPPED --pb--> REV --time-> START --time-> ACCEL --time-> RUN_FAST --time-> RUN_SLOW --stalled-> STOPPED 

// in general, from open, the normal flow is
// STOPPED --pb/timeout--> START --time-> ACCEL --time-> RUN_FAST --time-> RUN_SLOW --stalled-> STOPPED 

// in general, from confused, the normal flow is
// STOPPED --pb-->  START --time-> RUN_SLOW --stalled-> STOPPED 



const uint16_t min_ticks_in_final_traverse = 300; // 3 secs
const uint16_t max_ticks_in_final_traverse = 500; // 5 secs
const uint16_t tick_shift = 3;  // ie gain of 1/8, less than 4.5 from the PWM terms. 
//const uint16_t idle_ticks_auto_close = 3000; // 30 seconds.
const uint16_t idle_ticks_auto_close = 2000; // 30 seconds.   note that in idle, I sleep for 15ms rather than the 10ms normal basic time step. Hence the odd ratio.  

// uint16_t traverse_runtime;
// uint16_t start_runtime;
// uint16_t end_runtime;

uint8_t drn_enable;
uint16_t run_runtime;
uint16_t runtime;  
uint16_t ontime;
uint16_t offtime;
uint16_t  on_current;
uint16_t  back_emf;
uint16_t  biggest_Irun_seen;
uint16_t  smallest_bemf_seen;

uint8_t on_current_pin;
uint8_t back_emf_pin;
uint16_t ticks;
uint16_t hide_debounce_button;


const uint8_t debounce_button_period = 100 ; // 1 second if pwm loop is 10ms 

const uint8_t SLOW_ONTIME  = 2; 
const uint8_t SLOW_OFFTIME = 7;   // + 1 for stabilize backemf measure 

const uint8_t MED_ONTIME   = 6; 
const uint8_t MED_OFFTIME  = 3;   // + 1 for stabilize backemf measure 

const uint8_t FAST_ONTIME  = 9; 
const uint8_t FAST_OFFTIME = 0;   // + 1 for stabilize backemf measure 






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
  
  //  radio.sleep();

  pinMode(DRN1, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(DRN2, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(LOCK, OUTPUT);
  pinMode(START_STOP_N, INPUT);
  pinMode(AUTO_CLOSE, INPUT_PULLUP);
  pinMode(PROXIMITY_PWR, OUTPUT);
  pinMode(PROXIMITY_N, INPUT_PULLUP);

  analogReference(DEFAULT);
  
  digitalWrite(DRN1, DRN1_OPEN);
  digitalWrite(EN1,  PWM_OFF);
  digitalWrite(DRN2, DRN2_OPEN);
  digitalWrite(EN2,  PWM_OFF);
  digitalWrite(LOCK, LOCK_LOCKED);
  digitalWrite(PROXIMITY_PWR, 0);
  
  
  //  radio.sendWithRetry(GATEWAYID, "START", 5);

  sprintf(buff, "%02x SwingGate 20200418 ac=%x", NODEID, digitalRead(AUTO_CLOSE));
  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
  
  // radio.sleep();
  state = STATE_STOPPED;
  last_drn = 'C';
  ticks=0;
  hide_debounce_button=0;
  filt_pointer = 0;
  closed = false;
  run_runtime = 0;
  runtime = 0;
  buttoned = AUTO;
  biggest_Irun_seen = 0;
  smallest_bemf_seen  = 10000;
  radio_autoclose = 0;
  radio_start = 0;

  // this bit sets up values in EEROM only in the case eerom in unconfigured
#ifdef USE_EEPROM
    
  if (EEPROM.read(EEPROM_initialized_loc) != EEPROM_initialized_val)

    {
      EEPROM.write(EEPROM_initialized_loc, EEPROM_initialized_val);
      
      EEPROM.write(EEPROM_loc_hi_slow_open_bemf_min, (slow_open_bemf_min_val >> 8)) ;
      EEPROM.write(EEPROM_loc_lo_slow_open_bemf_min, (slow_open_bemf_min_val & 0xff)) ;

      EEPROM.write(EEPROM_loc_hi_slow_open_current_max, (slow_open_current_max_val >> 8)) ;
      EEPROM.write(EEPROM_loc_lo_slow_open_current_max, (slow_open_current_max_val & 0xff)) ;

      EEPROM.write(EEPROM_loc_hi_fast_open_bemf_min, (fast_open_bemf_min_val >> 8)) ;
      EEPROM.write(EEPROM_loc_lo_fast_open_bemf_min, (fast_open_bemf_min_val & 0xff)) ;

      EEPROM.write(EEPROM_loc_hi_fast_open_current_max, (fast_open_current_max_val >> 8)) ;
      EEPROM.write(EEPROM_loc_lo_fast_open_current_max, (fast_open_current_max_val & 0xff)) ;

      EEPROM.write(EEPROM_loc_hi_slow_close_bemf_min, (slow_close_bemf_min_val >> 8)) ;
      EEPROM.write(EEPROM_loc_lo_slow_close_bemf_min, (slow_close_bemf_min_val & 0xff)) ;

      EEPROM.write(EEPROM_loc_hi_slow_close_current_max, (slow_close_current_max_val >> 8)) ;
      EEPROM.write(EEPROM_loc_lo_slow_close_current_max, (slow_close_current_max_val & 0xff)) ;

      EEPROM.write(EEPROM_loc_hi_fast_close_bemf_min, (fast_close_bemf_min_val >> 8)) ;
      EEPROM.write(EEPROM_loc_lo_fast_close_bemf_min, (fast_close_bemf_min_val & 0xff)) ;

      EEPROM.write(EEPROM_loc_hi_fast_close_current_max, (fast_close_current_max_val >> 8)) ;
      EEPROM.write(EEPROM_loc_lo_fast_close_current_max, (fast_close_current_max_val & 0xff)) ;

      
      EEPROM.write(EEPROM_loc_hi_bemf_init, (bemf_init_val >> 8)) ;
      EEPROM.write(EEPROM_loc_lo_bemf_init, (bemf_init_val & 0xff)) ;

      EEPROM.write(EEPROM_loc_hi_current_init, (current_init_val >> 8)) ;
      EEPROM.write(EEPROM_loc_lo_current_init, (current_init_val & 0xff)) ;

    }
#endif    
}

/************************** MAIN ***************/

void (* resetFunction) (void) = 0; // declare reset func at adress 0



void loop() {
  uint8_t i;
  uint8_t address;
  uint8_t dataval;
  uint8_t senderid;
  
  ticks++;
  if (hide_debounce_button)
    {
      hide_debounce_button--;
    }
  
  // basic pwm routine
  if (runtime > 0)
    {
      // on part of pwm cycle
      digitalWrite(drn_enable, PWM_ON);
      delay(ontime);
      // implement running average on_current filter... read oldest
      on_current = im[filt_pointer];
      // replace oldest with new term
      im[filt_pointer] = analogRead(on_current_pin); 
      // sum all the stored terms. Skip the average divide by scaling the limit.
      for (i=0; i < ANA_FILTER_TERMS; i++) {
	on_current += im[i];
      }
      // off part of pwm cycle
      digitalWrite(drn_enable, PWM_OFF);
      delay(1);
      // implement running average filter... read oldest
      back_emf = bemf[filt_pointer];
      // replace oldest with new term
      bemf[filt_pointer] = analogRead(back_emf_pin);      
      // and sum all the stored terms. don't bother to average, scale the limit instead.
      for (i=0; i < ANA_FILTER_TERMS; i++) {
	back_emf += bemf[i];
      }
      // with ANA_FILTER_TERMS=4, back_emf and on_current are now the sum of the last 5 readings.
      // dividing by 5 would yield running_average filter... So I guess this is scaled running average
      
      //  Skip the basic initial acceleration from stopped phase, which is just a second long
      // calculate limit values
      if ( state != STATE_START)
	{
	  if (on_current > biggest_Irun_seen)
	    {
	      biggest_Irun_seen = on_current;
	    }
	  
	  if (back_emf < smallest_bemf_seen)
	    {
	      smallest_bemf_seen = back_emf;
	    }
	}
      // update the pointer to the running average filter ring buffer
      if (filt_pointer == (ANA_FILTER_TERMS -1) )
	{
	  filt_pointer = 0;
	}
      else
	{
	  filt_pointer++;
	}
      delay(offtime);


      runtime--;
      
      if (runtime == 0) {
#ifdef DEBUG
	sprintf(buff, "%02x timed exit of state %d min_bemf=%d max_i=%d", NODEID, state, smallest_bemf_seen, biggest_Irun_seen);
	radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
#endif
	update_timed_state();
#ifdef DEBUG
	sprintf(buff, "%02x new state is %d", NODEID, state);
	radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
	smallest_bemf_seen = 10000;
	biggest_Irun_seen = 0;
#endif
	
      }
      
      if  ((digitalRead(START_STOP_N) == 0) && (hide_debounce_button == 0))
	{
	  update_button_state();
	  hide_debounce_button = debounce_button_period;
	}
      if (runtime > 0) 
	if  (
	     ((( state == STATE_RUN_SLOW) || ( state == STATE_MISSED_LIMIT) || ( state == STATE_RUN_FAST)) && (digitalRead(PROXIMITY_N) == 0)) ||
	     
	     ((( state == STATE_RUN_SLOW) || ( state == STATE_MISSED_LIMIT)) && (last_drn == 'C')
	      && ((back_emf < slow_close_BEMF_min) || (on_current > slow_close_I_max))) ||
	     ((( state == STATE_RUN_SLOW) || ( state == STATE_MISSED_LIMIT)) && (last_drn == 'O')
	      && ((back_emf < slow_open_BEMF_min) || (on_current > slow_open_I_max))) ||
	     (( state == STATE_RUN_FAST) && (last_drn == 'C')
	      && ((back_emf < fast_close_BEMF_min) || (on_current > fast_close_I_max))) ||
	     (( state == STATE_RUN_FAST) && (last_drn == 'O')
	      && ((back_emf < fast_open_BEMF_min) || (on_current > fast_open_I_max)))
	     )
	  {
	    update_motor_state();
	  }
    }
  
  else
    // not running. Check for button press, or an automatic close timeout, or maybe a radio command
    {
      //      for (i=0; i < ontime+offtime+1; i++)
      //	{
      if (radio.receiveDone())
	{
	  CheckForWirelessHEX(radio, flash, true);
	  
	  if  (radio.DATALEN == 2)
	    {   
	      radio_start     = radio.DATA[0] & 1;
	      radio_autoclose = radio.DATA[1] & 1;
	      if ((radio.DATA[0] & 0xff) == 2) 
		{
		  resetFunction();
		}
	    }
#ifdef USE_EEPROM
	  if  ((radio.DATALEN == 5) && (radio.DATA[0] == 'W')) // Waadd all in hex 
	    {
	      address = ((radio.DATA[1] & 0x0f) << 4) | (radio.DATA[2] & 0x0f);
	      dataval = ((radio.DATA[3] & 0x0f) << 4) | (radio.DATA[3] & 0x0f);
	      EEPROM.write(address,dataval);
	    }
	  if  ((radio.DATALEN == 3) && (radio.DATA[0] == 'R')) // Waadd all in hex 
	    {
	      address = ((radio.DATA[1] & 0x0f) << 4) | (radio.DATA[2] & 0x0f);
	      dataval = EEPROM.read(address);
	      senderid = radio.SENDERID;
	      sprintf(buff, "%02x:%02x", address,dataval);
	      radio.sendWithRetry(senderid, buff, strlen(buff));
	    }
#endif
	  if (radio.ACKRequested())
	    {
	      radio.sendACK();
	    }
	} // end of if (radio.receiveDone())
      //delay(1);       // whats this for?
      LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
      // }
      if ((digitalRead(START_STOP_N) == 0) && (hide_debounce_button == 0) 
	  || (radio_start == 1)
	  )
	{
	  update_button_state();
	  hide_debounce_button = debounce_button_period;
	  radio_start = 0;
	}
      if (ticks > idle_ticks_auto_close)
	{
	  ticks=0;

	  if (((digitalRead(AUTO_CLOSE)==1) ^ (radio_autoclose == 1))
	      && (!closed)
	      && (buttoned==AUTO)
	      )
	    {
	      now_closing();
	      state = STATE_START;
	      runtime = 100;
	      // init the running average filters
	      for (i=0; i < ANA_FILTER_TERMS; i++) {
		bemf[i] = bemf_init_val;
		im[i]   = current_init_val;
	      }
	    }   
	}
    }
}


// update the state variables
// come here when a timeout expires
void update_timed_state(void)
{
  uint16_t batt_adc;
  float batt_v;
  switch (state)
    {
    case STATE_STOPPED :
      break;

    case STATE_REV :
      now_opening();
      state = STATE_START;
      ontime = SLOW_ONTIME;
      offtime = SLOW_OFFTIME;
      runtime = 200;
      break;

    case STATE_START :
      digitalWrite(LOCK, LOCK_LOCKED);
      if (run_runtime && (buttoned == AUTO))
	{
	  state = STATE_ACCEL;
	  ontime = MED_ONTIME;
	  offtime = MED_OFFTIME;
	  runtime = 200;
	}
      else
	{
	  state = STATE_RUN_SLOW;
	  runtime = 6000;
	} 
      
      break;
      
    case STATE_ACCEL :
      state = STATE_RUN_FAST;
      ontime = FAST_ONTIME;
      offtime = FAST_OFFTIME;
      runtime = run_runtime;
      break;

    case STATE_RUN_FAST :
      state = STATE_RUN_SLOW;
      ontime = SLOW_ONTIME;
      offtime = SLOW_OFFTIME;
      runtime = 6000;
      ticks = 0;
      break;

    case STATE_RUN_SLOW :
      // just going to be in missed_limit briefly so I know what happened.
      state = STATE_MISSED_LIMIT;
      runtime = 200;
      ontime = SLOW_ONTIME;
      offtime = SLOW_OFFTIME;
      break;
    
      // this is an exit for limit sw now
    case STATE_MISSED_LIMIT :
      stop_motor();
      // run_runtime = 0;
      if (last_drn == 'C') 
	sprintf(buff,"%02x missed lim C minBEMF %d (%d) maxI %d (%d)", NODEID,
		smallest_bemf_seen, slow_close_BEMF_min,
		biggest_Irun_seen, slow_close_I_max);
      else
	sprintf(buff,"%02x missed lim O minBEMF %d (%d) maxI %d (%d)", NODEID,
		smallest_bemf_seen, slow_open_BEMF_min,
		biggest_Irun_seen, slow_open_I_max);
	
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      //      radio.sleep();
      break;
    }
}

// generally make a PB press do something expected, rather than a cancel.
//   I guess if traversing, stop.
//   if stopped, traverse slowly in the opposite drection to last time.

// this is entered when a button is pressed. It is debounced elsewhere.

void update_button_state(void)
{
  int i;
  switch (state)
    {
    case STATE_STOPPED :
      if (last_drn == 'C')
	sprintf(buff,"%02x button start C %d (%d) %d (%d)", NODEID, back_emf, slow_close_BEMF_min, on_current, slow_close_I_max);
      else 
	sprintf(buff,"%02x button start O %d (%d) %d (%d)", NODEID, back_emf, slow_open_BEMF_min, on_current, slow_open_I_max);
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      //      radio.sleep();
      if (closed)
	{
#ifdef AUTOCLOSE_SKIPS_UNLOCK
	  if ((digitalRead(AUTO_CLOSE)==1) ^ (radio_autoclose == 1))
	    {
	      now_opening();
	      buttoned = AUTO;
	      state = STATE_START;
	      runtime = 100;
	    }
	    else
	      {
#endif
	  now_closing();
	  digitalWrite(LOCK, LOCK_UNLOCK);
	  buttoned = AUTO;
	  state = STATE_REV;
	  runtime = 100;
#ifdef AUTOCLOSE_SKIPS_UNLOCK
	      }
#endif
	}
      else
	{
	  if (buttoned == MANUAL)
	    {
	      if (last_drn == 'O')
		{
		  now_closing();
		}
	      else
		{
	      now_opening();
		}
	    }
	  else 
	    {
	      // was open 
	      now_closing();
	    }
	  state = STATE_START;
	  runtime = 100;
	}
      // init the running average filters
      for (i=0; i < ANA_FILTER_TERMS; i++) {
	bemf[i] = bemf_init_val;
	im[i]   = current_init_val;
      }
      break;

    case STATE_START :
    case STATE_ACCEL :
    case STATE_RUN_FAST :
    case STATE_RUN_SLOW :
    case STATE_MISSED_LIMIT :

      if (last_drn == 'C')
	sprintf(buff,"%02x button stop C %d (%d) %d (%d) %d", NODEID, back_emf, slow_close_BEMF_min, on_current, slow_close_I_max, state);
      else 
	sprintf(buff,"%02x button stop O %d (%d) %d (%d) %d", NODEID, back_emf, slow_open_BEMF_min, on_current, slow_open_I_max, state);
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      //      radio.sleep();
      //   buttoned = MANUAL; 
      stop_motor();
      closed = false;
      break;
    }
}

// entered on stall or overcurrent or proximity switch
void update_motor_state(void)
{
  int i;
  uint16_t batt_adc;
  float batt_v;
  switch (state)
    {
      
    case STATE_START :
    case STATE_ACCEL :
    case STATE_RUN_FAST :
      if (last_drn == 'C')
	sprintf(buff,"%02x stall fast C V=%d/%d I=%d/%d s=%d rt=%d", NODEID, back_emf, fast_close_BEMF_min, on_current, fast_close_I_max, state, run_runtime);
      else 
	sprintf(buff,"%02x stall fast O V=%d/%d I=%d/%d s=%d rt=%d", NODEID, back_emf, fast_open_BEMF_min, on_current, fast_open_I_max, state, run_runtime);
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      //      radio.sleep();
      stop_motor();
      run_runtime = 0;
      closed = false;
      buttoned = AUTO;
      ticks=0;
      break;

      // stall in RUN_SLOW is the desired exit state
      
    case STATE_RUN_SLOW :
    case STATE_MISSED_LIMIT :
      if (digitalRead(PROXIMITY_N) == 0) {
	if (last_drn == 'C')
	    sprintf(buff,"%02x limit runslow V=%d/%d I=%d/%d s=%d rt=%d %c", NODEID, back_emf, slow_close_BEMF_min, on_current, slow_close_I_max, state, run_runtime, last_drn);
	else
	  sprintf(buff,"%02x limit runslow V=%d/%d I=%d/%d s=%d rt=%d %c", NODEID, back_emf, slow_open_BEMF_min, on_current, slow_open_I_max, state, run_runtime, last_drn);
      }
      else {
	
	
	if (last_drn == 'C')
	    sprintf(buff,"%02x stalled runslow V=%d/%d I=%d/%d s=%d rt=%d %c", NODEID, back_emf, slow_close_BEMF_min, on_current, slow_close_I_max, state, run_runtime, last_drn);
	else
	  sprintf(buff,"%02x stalled runslow V=%d/%d I=%d/%d s=%d rt=%d %c", NODEID, back_emf, slow_open_BEMF_min, on_current, slow_open_I_max, state, run_runtime, last_drn);
      }
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      //      radio.sleep();
      update_runtimes(ticks);
      buttoned = AUTO;
      stop_motor();
      
      delay(100);
      batt_adc =  analogRead(BATT_ADC);
      batt_v = BATT_GAIN * batt_adc; 
      dtostrf(batt_v, 5, 2, buff2);
      sprintf(buff, "%02x Batt=%sV rrt=%d rac=%d ac=%d", NODEID, buff2, run_runtime, radio_autoclose, digitalRead(AUTO_CLOSE)   );  
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      
      ticks=0;
      if (last_drn == 'C')
	{
	  closed = true;
	}
      else
	{
	  closed = false;
	}
    }
}

// on successful traverse, get passed the time spent in the final slow traverse.
// if too small, spend less time in the fast traverse.
// if too big, , spend more time in the fast traverse.
// fast traverse is currently 4.5 times as fast as slow traverse.
// but will fine tune each time

void update_runtimes (int ticks)
{
  if (ticks < min_ticks_in_final_traverse)
    {
      // bit fast, spend less time in fast traverse
      sprintf(buff,"%02x fast runtime too long %d ticks %d", NODEID, run_runtime, ticks);
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      //      radio.sleep();
      run_runtime -= ((min_ticks_in_final_traverse - ticks) >>  tick_shift);
    }
  else
    {
      if (ticks > max_ticks_in_final_traverse)
	{
	  // bit slow, spend more time in fast traverse
	  sprintf(buff,"%02x fast runtime too short %d ticks %d", NODEID, run_runtime, ticks);
	  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
	  run_runtime += ((ticks - max_ticks_in_final_traverse) >>  tick_shift);
	}
    }
}


void now_opening (void)
{
  digitalWrite(PROXIMITY_PWR,1);  // enable proximity switches
  drn_enable = EN1 ;
  on_current_pin = IS1 ;
  back_emf_pin = BACKEMF1;
  biggest_Irun_seen = 0;
  smallest_bemf_seen  = 10000;
  last_drn = 'O';
  ontime = SLOW_ONTIME;
  offtime = SLOW_OFFTIME;
  digitalWrite(DRN2, 0);
  digitalWrite(EN1,  PWM_OFF);
  digitalWrite(DRN1, 1);
  digitalWrite(EN2,  PWM_ON);
}


void now_closing (void)
{
  digitalWrite(PROXIMITY_PWR,1);  // enable proximity switches
  // runtime = 100;
  drn_enable = EN2 ;
  on_current_pin = IS2 ;
  back_emf_pin = BACKEMF2;
  biggest_Irun_seen = 0;
  smallest_bemf_seen  = 10000;
  last_drn = 'C';
  ontime = SLOW_ONTIME;
  offtime = SLOW_OFFTIME;
  digitalWrite(DRN1, 0);
  digitalWrite(EN2, PWM_OFF);
  digitalWrite(EN1, PWM_ON);
  digitalWrite(DRN2, 1);
}

void stop_motor (void)
{
  digitalWrite(DRN1, 0);
  digitalWrite(DRN2, 0);
  digitalWrite(EN1, PWM_OFF);
  digitalWrite(EN2, PWM_OFF);
  digitalWrite(PROXIMITY_PWR,0);  // power down proximity switches

  state = STATE_STOPPED;
  runtime=0;

}
