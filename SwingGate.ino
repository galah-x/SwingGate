//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'SwingGate for moteino Time-stamp: "2019-02-05 20:28:55 john"';


// Given the controller boards have been destroyed by lightning for the last 2 summers running,
// going to engineer my own so I can fix it more easily

// basically I have to drive an unlock solenoid at about 1.5A for a second or so at start of open
// and I have to drive a swing linear actuator at run current of a couple of A (~15A stall) 
// interface to a pushbutton  (radio remotes bang a relay as the PB)
// and a toggle switch controlling whether to autoclose or not, how hard can it be?

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
const unsigned int  bemf_min_val = 1800; // 300*5
const unsigned int  current_max_val = 600; // 200*5
const unsigned int bemf_init_val = 500;
const unsigned int current_init_val = 0;
unsigned int bemf[ANA_FILTER_TERMS];
unsigned int im[ANA_FILTER_TERMS];
byte filt_pointer = 0;

// which way is the motor currently moving / moved last time
byte last_drn;
const byte DRN_CLOSING = 0;
const byte DRN_OPENING = 1;

// is the gate closed?
byte closed;
const byte IS_CLOSED = 1;
const byte NOT_CLOSED = 0;

// did someone hit the button except to start it initially.
byte buttoned;
const byte MANUAL = 1;
const byte AUTO = 0;

byte state;

// controlls motion
const byte STATE_STOPPED      = 0;
const byte STATE_START        = 1;
const byte STATE_ACCEL        = 2;
const byte STATE_RUN          = 3;
const byte STATE_RUN_SLOW     = 4;
const byte STATE_MISSED_LIMIT = 5;

const int min_ticks_in_final_traverse = 300; // 3 secs
const int max_ticks_in_final_traverse = 500; // 5 secs
const int tick_shift = 2;  // ie gain of 4, less than 4.5 from the PWM terms. 
const int idle_ticks_auto_close = 3000; // 30 seconds.

int traverse_runtime;
int start_runtime;
int end_runtime;

byte drn_enable;
unsigned int run_runtime;
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
unsigned int hide_debounce_button;


const byte debounce_button_period = 100 ; // 1 second if pwm loop is 10ms 

const byte SLOW_ONTIME  = 2; 
const byte SLOW_OFFTIME = 7;   // + 1 for stabilize backemf measure 

const byte MED_ONTIME   = 6; 
const byte MED_OFFTIME  = 3;   // + 1 for stabilize backemf measure 

const byte FAST_ONTIME  = 9; 
const byte FAST_OFFTIME = 0;   // + 1 for stabilize backemf measure 




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
  last_drn = DRN_CLOSING;
  ticks=0;
  hide_debounce_button=0;
  filt_pointer = 0;
  closed = IS_CLOSED;
  run_runtime = 0;
  buttoned = AUTO;
  biggest_on_current_seen = 0;
  smallest_back_emf_seen  = 10000;
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
      // sum all the stored terms. don't bother to average, scale the limit.
      for (i=0; i < ANA_FILTER_TERMS; i++) {
	back_emf += bemf[i];
      }
      //  Skip the basic initial acceleration from stopped phase, which is just a second long
      // calculate limit values
      if ( state != STATE_START)
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
	}
      else
	{
	  filt_pointer++;
	}
      delay(offtime);
      
      runtime--;
      if (runtime == 0) {
	update_timed_state();
      }
      if  ((digitalRead(START_STOP_N) == 0) && (hide_debounce_button == 0))
	{
	  update_button_state();
	  hide_debounce_button = debounce_button_period;
	}
      if  (( state != STATE_START)  
	   && ((back_emf < back_emf_min) || (on_current > on_current_max))
	   )
	{
	  update_motor_state();
	}
    }
  else
    {
      delay(ontime+offtime+1);
      if ((digitalRead(START_STOP_N) == 0) && (hide_debounce_button == 0))
	{
	  update_button_state();
	  hide_debounce_button = debounce_button_period`<;
	}
      if (ticks > idle_ticks_auto_close)
	{
	  ticks=0;

	  if ((digitalRead(AUTO_CLOSE)==1)  && (closed == NOT_CLOSED) && (buttoned==AUTO))
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
void update_timed_state(void)
{
  unsigned int batt_adc;
  float batt_v;
  switch (state)
    {
    case STATE_STOPPED :
      break;

    case STATE_START :
      digitalWrite(LOCK, LOCK_LOCKED);
      if (run_runtime && (buttoned == AUTO))
	{
	  state = STATE_ACCEL;
	}
      else
	{
	  state = STATE_RUN_SLOW;
	}
      runtime = 200;
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
      runtime = run_runtime;
      break;

    case STATE_RUN_SLOW :
      ticks = 0;
      state = STATE_MISSED_LIMIT;
      runtime = 6000;
      ontime = SLOW_ONTIME;
      offtime = SLOW_OFFTIME;
      break;
    
      // this is an abnormal exit now
    case STATE_MISSED_LIMIT :
      state = STATE_STOPPED;
      stop_motor();
      runtime = 0;
      run_runtime = 0;
      sprintf(buff,"%02x missed limit minBEMF %d (%d) maxI %d (%d)", NODEID,
	      smallest_back_emf_seen, back_emf_min,
	      biggest_on_current_seen, on_current_max);
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      radio.sleep();
      break;
    }
}

// changes needed...
// OK redo the direction switch as an auto switch 
// manage an unknown start case (maybe free as it will jam and hopefully next time is right)
// OK auto timing for the midrange traverse
// test bemf does what I hope it will
// generally make a PB press do something expected, rather than a cancel.
//   I guess if traversing, stop.
//   if stopped, traverse slowly in the opposite drection to last time.
      
void update_button_state(void)
{
  int i;
  switch (state)
    {
    case STATE_STOPPED :
      sprintf(buff,"%02x button start %d (%d) %d (%d)", NODEID, back_emf, back_emf_min, on_current, on_current_max);
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      radio.sleep();
      if (closed == IS_CLOSED)
	{
	  now_opening();
	  digitalWrite(LOCK, LOCK_UNLOCK);
	  delay(750);
	  buttoned = AUTO;
	}
      else
	{
	  if (buttoned == MANUAL)
	    {
	      if (last_drn == DRN_OPENING)
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
	}
      state = STATE_START;
      runtime = 100;
      // init the running average filters
      for (i=0; i < ANA_FILTER_TERMS; i++) {
	bemf[i] = bemf_init_val;
	im[i]   = current_init_val;
      }
      break;

    case STATE_START :
    case STATE_ACCEL :
    case STATE_RUN :
    case STATE_RUN_SLOW :
    case STATE_MISSED_LIMIT :
      
      sprintf(buff,"%02x button stop %d (%d) %d (%d) %d", NODEID, back_emf, back_emf_min, on_current, on_current_max, state);
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      radio.sleep();
      state = STATE_STOPPED;
      buttoned = MANUAL; 
      stop_motor();
      runtime=0;
      closed = NOT_CLOSED;
      break;
    }
}

// entered on stall or overcurrent
void update_motor_state(void)
{
  int i;
  switch (state)
    {
      
    case STATE_START :
    case STATE_ACCEL :
    case STATE_RUN :

      sprintf(buff,"%02x accel %d (%d) %d (%d) %d %d", NODEID, back_emf, back_emf_min, on_current, on_current_max, state, run_runtime);
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      radio.sleep();
      state = STATE_STOPPED;
      runtime=0;
      stop_motor();
      runtime = 0;
      closed = NOT_CLOSED;
      buttoned = MANUAL;
      ticks=0;
      break;


    case STATE_RUN_SLOW :
    case STATE_MISSED_LIMIT :
      sprintf(buff,"%02x run slow %d (%d) %d (%d) %d %d", NODEID, back_emf, back_emf_min, on_current, on_current_max, state, run_runtime);
      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
      radio.sleep();
      update_runtimes(ticks);
      buttoned = AUTO;
      runtime=0;
      stop_motor();
      ticks=0;
      if (last_drn == DRN_CLOSING)
	{
	  closed = IS_CLOSED;
	}
      else
	{
	  closed = NOT_CLOSED;
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
      // bit fast, spend less tim ein fast traverse
      run_runtime -= (ticks - min_ticks_in_final_traverse) >>  tick_shift;
    }
  else
    {
      if (ticks > max_ticks_in_final_traverse)
	{
	  // bit slow, spend more time in fast traverse
	  run_runtime += (ticks - max_ticks_in_final_traverse) >>  tick_shift;
	}
    }
}


void now_opening (void)
{
  drn_enable = EN1 ;
  on_current_pin = IS1 ;
  back_emf_pin = BACKEMF1;
  biggest_on_current_seen = 0;
  smallest_back_emf_seen  = 10000;
  last_drn = DRN_OPENING;
  back_emf_min = bemf_min_val ;
  on_current_max = current_max_val ;
  ontime = SLOW_ONTIME;
  offtime = SLOW_OFFTIME;
  digitalWrite(DRN2, 0);
  digitalWrite(EN1,  PWM_OFF);
  digitalWrite(DRN1, 1);
  digitalWrite(EN2,  PWM_ON);
}


void now_closing (void)
{
  // runtime = 100;
  drn_enable = EN2 ;
  on_current_pin = IS2 ;
  back_emf_pin = BACKEMF2;
  biggest_on_current_seen = 0;
  smallest_back_emf_seen  = 10000;
  last_drn = DRN_CLOSING;
  back_emf_min = bemf_min_val ;
  on_current_max = current_max_val ;
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
}
