/* Sleep_tests25
 * 
	v25 Adds code to convert the pressure and temperature floating point values 
	into truncated character strings before writing to the SD card, in part to 
	try speeding up write times, but also because the extra digits of precision 
	currently being transmitted are bogus anyways. 
 
	v24 Trying to make periodic sampling regime functional. This version will
	only generate 1 file per day, and simply append new samples to the end of
	the daily file during each sampling interval (usually 30 minute periods). 
	Change the values of STARTMINUTE and DATADURATION below to change the start
	time and length of data taking period. STARTMINUTE can be set from 0 to 59
	and DATADURATION can be from 1 to 60.
	This version is set to use the DS3231 Chronodot's 32.768kHz output to run
	TIMER2. During lowPowerSleep (when not taking data), this version will 
	briefly flash the LED on D9 to indicate when it has awakened and returned
	to sleep.
 
 * v23 Trying to implement a periodic sampling regime that starts at a 
 * user-designated time and continues for some number of minutes, then goes 
 * into a deep watchdog-timed sleep for the remainder of the hour. But for some
 * reason the sketch isn't properly waking from the 8s sleep when it finishes
 * the data-taking cycle. It does sleep 8s cycles fine BEFORE the data taking
 * but not after. Very strange. 
 * 
  v22 Added code to update the file creation date and modify date so that
  it makes some sense. Writing 1 second's worth of data to the SD card
  takes about 120ms based on timing this sketch, in large part because 
  there are a lot of values being streamed to the card. 

  v21 Changed the end of the setup loop slightly so that TIMER2 doesn't
  start running until the real time clock has started a new second.
  With the f_wdt flag set to 1 initially, the main loop should immediately
  take a data reading (as if it is at the 0 ms mark).
  This is an attempt to better sync the 32.768 watch crystal with the timing
  of the real time clock. v20 was dropping 1 second of data every once in
  a while, probably because the two clocks were out of sync.

  v20 Change from built-in SD library to the SdFat library.
  https://code.google.com/p/sdfatlib/downloads/list <-- sdfatlib20131225.zip
  Fixing the write-to-SD interval at 1 second (4 samples) since the write
  time scales linearly with the buffer size, so there is no gain from increasing
  the time between SD writes.
  Moved all filename initialization code to the function initFileName so that
  it can be called from anywhere in the program to generate a new filename
  based on the current date.
  Added code to keep track of day (oldday) and generate a new filename when
  the new date is different from the day stored in oldday. This should start
  a new output file every night at midnight.

  v19 Implement a file naming scheme based on date + time from Chronodot
  DS3231 real time clock. Filenames will follow the pattern YYMMDDXX.CSV
  where the XX is a counter (0 - 99) for files generated on the same day.
  It also adds a 2nd indicator LED to Arduino
  pin D8 (PINB0, physical pin 14) to indicate when the card initialization
  fails.

  v18 Switched to new MS5803_05/14 style library for pressure sensor. These
  new libraries correctly carry out the pressure calculations without
  variable overflows.

  v17 This sketch implements a button on pin 2 (interrupt 0) to stop the
  data logging process (see the function endRun). Once stopped, the
  watchdog timer is set to 8 second timeouts, and the LED is flashed
  briefly to indicate that data logging has stopped and the processor
  is sleeping. This version also implements a set of data storage
  arrays so that writes to the SD card only happen once per second.

  Hardware:
  ATMEGA328P on a breadboard, with bootloader set to run on internal
  8MHz oscillator. LED on pins D8 + D9. Powered from a 3V3 regulator.

                                       +---\/---+                      10k ohm
    GND---BUTTON--------(RESET)  PC6  |1       28|  PC5  (ADC5 / SCL) --/\/\-- +3V3
                          (RXD)  PD0  |2       27|  PC4  (ADC4 / SDA) --/\/\-- +3V3
                          (TXD)  PD1  |3       26|  PC3  (ADC3)         10k ohm
    GND---BUTTON---------(INT0)  PD2  |4       25|  PC2  (ADC2)
                         (INT1)  PD3  |5       24|  PC1  (ADC1)
                     (XCK / T0)  PD4  |6       23|  PC0  (ADC0)
                  +3.3V -------  VCC  |7       22|  GND
                                 GND  |8       21|  AREF
DS3231 32kHz ---(XTAL1 / TOSC1)  PB6  |9       20|  AVCC
                (XTAL2 / TOSC2)  PB7  |10      19|  PB5  (SCK) --- CLK on SD Card
                           (T1)  PD5  |11      18|  PB4  (MISO) --- D0 on SD Card
                         (AIN0)  PD6  |12      17|  PB3  (MOSI / OC2) --- DI on SD Card
                         (AIN1)  PD7  |13      16|  PB2  (SS / OC1B) --- CS on SD Card
 GND --/\/\--- LED ------(ICP1)  PB0  |14      15|  PB1  (OC1A)  --- LED --/\/\-- GND
       560ohm                          +--------+                          560ohm


  Programmed via AVR MKII ISP.

  Communicating via FTDI Friend at 57600 bps.

  Chronodot DS3231 hooked up via I2C to pins A4 + A5 (SDA + SCL), along
  with 10k ohm pull-up resistors to 3V3.
  Chronodot DS3231 32.768kHz output pin hooked to XTAL1, with 10kOhm
  pull-up resistor to +Vcc.

  MS5803 pressure sensor hooked up via I2C:
               --------------
   SCL     ---|1   MS5803   8|--- NC
   GND     ---|2            7|--- SDA
   +3V3    ---|3            6|--- +3V3
   NC      ---|4            5|--- +3V3
               --------------
  (NC = no connection)
  -------------------------------------------------------------------
  The sketch:
  During setup, we enable the Chronodot's 32.768kHz 
  output (as of v24, instead of using a separate 32.768kHz crystal 
  oscillator) hooked to XTAL1
  to provide the timed interrupt. The Chronodot 32kHz pin should be
  connected to XTAL1, with a 10kOhm pull-up resistor 
  between XTAL1 and +Vcc (3.3V). 

  We set the prescaler on TIMER2 so that it only interrupts
  every 0.25 second (see commented code below for other prescaler
  options). In the main loop, whenever TIMER2 rolls over and causes an
  interrupt (see ISR at the bottom), the statements in the
  main loop's if() statement should execute, including reading the
  time, reading the MS5803 sensor, printing those values to the
  serial port, and then going to sleep via the goToSleep function.

  In the goToSleep function you'll see two bitSet(PINB,1)
  operations before and after the actual sleep. This toggles
  the output of PB1 (Arduino digital pin 9, physical pin 15 on the
  AVR 28-pin DIP), which lets you see the
  duration of the sleep vs. wake cycles with an oscilloscope.
  bitSet(PINB,1) is the equivalent of Arduino's
  digitalWrite(9, !digitalRead(9)), but much faster.
  
  Change the STARTMINUTE and DATADURATION values in the preamble
  below to set when the unit should wake up and start taking 
  data, and for how long during each hour it should take data.
  For example, STARTMINUTE = 0 and DATADURATION = 30 would 
  take data every hour starting at minute 00, and record 30 
  minutes worth of data. For continuous recording, set 
  STARTMINUTE = 0 and DATADURATION = 60. 
  
  When not recording, the sketch goes into a lower power 
  sleep (lowPowerSleep) mode using the watchdog timer set 
  to 8 second timeouts to conserve power further.

 */
#include <SPI.h>
#include <Wire.h>
#include <RTClib.h>
#include <RTC_DS3231.h>
#include <MS5803_05.h>
#include <SdFat.h>

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <wiring_private.h>
#include <avr/wdt.h>

#define LED 9 // Arduino pin D9, physical pin 15 on AVR
#define ERRLED 8 // Arduino pin D8, physical pin 14 on AVR

#define ECHO_TO_SERIAL 1 // echo data to serial port, set to 0 to turn off

#define STARTMINUTE 0 // minute of hour to start taking data, 0 to 59
#define DATADURATION 60 // # of minutes to collect data, 1 to 60

#define SAMPLES_PER_SECOND 4// number of samples taken per second
#define SAMPLE_BUFFER 4 // size of sample buffer arrays

const byte chipSelect = 10; // define the Chip Select pin for SD card
char filename[] = "LOGGER00.CSV";

// Declare data arrays
uint32_t unixtimeArray[SAMPLE_BUFFER]; // store unixtime values temporarily
byte mscountArray[SAMPLE_BUFFER]; // store mscount values temporarily
float pressureArray[SAMPLE_BUFFER]; // store pressure readings temporarily
float tempCArray[SAMPLE_BUFFER]; // store temperature readings temporarily
byte loopCount = 0; // counter to keep track of data sampling loops

byte precision = 2; // decimal precision for stored pressure data values
char pressBuffer [10]; // character buffer to hold string-converted pressure
char tempBuffer [7]; // character buffer to hold string-converted temperature

volatile int f_wdt = 1; // TIMER2 flag (watchdog or TIMER2 counter)
volatile byte mscount = 0; // fractional second count for timestamp (0,25,50,75)
// endMinute is the minute value after which the program should stop taking
// data for the remainder of the hour
uint8_t endMinute = STARTMINUTE + DATADURATION - 1;
// wakeMinute is the minute value when the program should start preparing to
// wake up and take data again. This will be set during the setup loop
uint8_t wakeMinute = 59;

DateTime oldtime; // used to track time in main loop
DateTime newtime; // used to track time
uint8_t oldday; // used to track when a new day starts

RTC_DS3231 RTC;
MS_5803 sensor = MS_5803(512);
SdFat sd;
SdFile logfile;  // for sd card, this is the file object to be written to


 
//---------------------------------------------------------
// SETUP loop
void setup() {

	pinMode(chipSelect, OUTPUT);  // set chip select pin for SD card to output
	// Set indicator LED as output
	pinMode(LED, OUTPUT);
	pinMode(ERRLED, OUTPUT);
	// Set interrupt 0 (physical pin 2) as input, use internal pullup resistor
	pinMode(2, INPUT_PULLUP);

#if ECHO_TO_SERIAL
	Serial.begin(57600);
	Serial.println();
	delay(500);
	Serial.println(F("Starting up..."));
#endif
	// Calculate what minute of the hour the program should turn the 
	// accurate timer back on to start taking data again
	if (STARTMINUTE == 0) {
		wakeMinute = 59;
	} else {
		wakeMinute = STARTMINUTE - 1;
	}
	// if endMinute value is above 60, just set it to 59
	if (endMinute >= 60) endMinute = 59;
	


	//--------RTC SETUP ------------
	Wire.begin();
	RTC.begin(); // Start Chronodot
	RTC.enable32kHz(false); // Stop 32.768kHz output from Chronodot
	// The Chronodot can also put out several different square waves
	// on its SQW pin (1024, 4096, 8192 Hz), though I don't use them
	// in this sketch. The code below disables the SQW output to make
	// sure it's not using any extra power
	RTC.SQWEnable(false); // Stop the SQW output pin
	DateTime starttime = RTC.now(); // get initial time
	//-------End RTC SETUP-----------

	// Initialize the SD card object
	// Try SPI_FULL_SPEED, or SPI_HALF_SPEED if full speed produces
	// errors on a breadboard setup. 
	if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
	// If the above statement returns FALSE after trying to 
	// initialize the card, enter into this section and
	// hold in an infinite loop.
#if ECHO_TO_SERIAL
		Serial.println(F("SD initialization failed"));
#endif
		
		while(1){ // infinite loop due to SD card initialization error
			bitSet(PINB, 0); // Toggle error led on PINB0 (D8 Arduino)
			delay(100);
			bitSet(PINB, 0); // Toggle error led on PINB0 (D8 Arduino)
			bitSet(PINB, 1); // Toggle the LED on Arduino pin D9 (PINB1)
			delay(100);
			bitSet(PINB, 1); // Toggle the LED on Arduino pin D9 (PINB1)
		}
	}
#if ECHO_TO_SERIAL
	Serial.println(F("SD initialization done."));
#endif

#if ECHO_TO_SERIAL
	Serial.print(F("Start minute: "));
	Serial.print(STARTMINUTE);
	Serial.print(F(", endMinute: "));
	Serial.print(endMinute);
	Serial.print(F(", wakeMinute: "));
	Serial.println(wakeMinute);
	delay(40);
#endif
	
	// Start MS5803 pressure sensor
	sensor.initializeMS_5803();

	//--------------------------------------------------------
	// Check current time, branch the rest of the setup loop
	// depending on whether it's time to take data or 
	// go to sleep for a while
	newtime = RTC.now();

	if (newtime.minute() >= STARTMINUTE && newtime.minute() <= endMinute){
		// Create new file on SD card using initFileName() function
		// This creates a filename based on the year + month + day
		// with a number indicating which number file this is on
		// the given day. The format of the filename is
		// YYMMDDXX.CSV where XX is the counter (00-99). The
		// function also writes the column headers to the new file.
		initFileName(starttime);

		// Start 32.768kHz clock signal on TIMER2. Set argument to true
		// if using a 32.768 signal from the DS3231 Chronodot on XTAL1, 
		// or false if using a crystal on XTAL1/XTAL2. The 2nd argument
		// should be the current time
		newtime = startTIMER2(true, newtime);
		// Initialize oldtime and oldday values
		oldtime = newtime;
		oldday = oldtime.day();
		// set f_wdt flag to 1 so we start taking data in the main loop
		f_wdt = 1;
	} else if (newtime.minute() < STARTMINUTE | newtime.minute() > endMinute){
		// The current minute is earlier or later in the hour than the user has 
		// specified for taking data.
		oldtime = newtime;
		oldday = oldtime.day();

		cbi(TIMSK2,TOIE2); // disable the TIMER2 interrupt on overflow
		// Initialize oldtime and oldday values
		oldtime = newtime;
		oldday = oldtime.day();
		//  Enter the low power sleep cycle in the main loop
		// by setting f_wdt = 2.
		f_wdt = 2;	  
	}



}

//--------------------------------------------------------------
// Main loop
// We will enter the if() statement immediately when we get to the main loop for
// the first time, because f_wdt was set = 1 or 2 during the setup loop.
void loop() {
	//1111111111111111111111111111111111111111111111111111111111111111111111111
	//1111111111111111111111111111111111111111111111111111111111111111111111111
	if (f_wdt == 1) {
		f_wdt = 0; // reset interrupt flag
//		bitSet(PIND, 6); // used to visualize timing with LED or oscilloscope

		// Get a new time reading from the real time clock
		newtime = RTC.now();
		
		//*********************************************************************
		//*********************************************************************
		// If the current minute is >= than STARTMINUTE and <= endMinute, 
		// then take data
		if (newtime.minute() >= STARTMINUTE && newtime.minute() <= endMinute) {

			// Check to see if the current seconds value
			// is equal to oldtime.second(). If so, we
			// are still in the same second. If not,
			// the mscount value should be reset to 0
			// and oldtime updated to equal newtime.
			if (oldtime.second() != newtime.second()) {
				mscount = 0; // reset mscount
				oldtime = newtime; // update oldtime
				loopCount = 0; // reset loopCount
			}

			// Save current time to unixtimeArray
			unixtimeArray[loopCount] = newtime.unixtime();
			mscountArray[loopCount] = mscount;

			// bitSet(PINB,1); // used to visualize timing with LED or oscilloscope

			// Use readSensor() function to get pressure and temperature reading
			sensor.readSensor();
			pressureArray[loopCount] = sensor.pressure();
			tempCArray[loopCount] = sensor.temperature();

//			bitSet(PIND, 6); // used to visualize timing with LED or oscilloscope

			// Now if loopCount is equal to the value in SAMPLE_BUFFER
			// (minus 1 for zero-based counting), then write out the contents
			// of the sample data arrays to the SD card.
			if (loopCount >= (SAMPLE_BUFFER - 1)) {
				// Check to see if a new day has started. If so, open a new file
				// with the initFileName() function
				// TODO: this check may not be necessary any more if we're splitting
				// files up hourly
				if (oldtime.day() != oldday) {
					// Generate a new output filename based on the new date
					initFileName(oldtime);
					// Update oldday value to match the new day
					oldday = oldtime.day();
				}
				//      bitSet(PINB,1); // used to visualize timing with LED or oscilloscope

				// Call the writeToSD function to output the data array contents
				// to the SD card
				writeToSD();
#if ECHO_TO_SERIAL
				if (newtime.second() % 10 == 0){
					printTimeSerial(newtime, mscount);
					// Show pressure
					Serial.print(F("Pressure = "));
					Serial.print(sensor.pressure());
					Serial.print(F(" mbar, "));
					Serial.print(sensor.temperature());
					Serial.println(F(" C"));
					Serial.print(F("Free RAM: ")); Serial.println(freeRam());
					delay(40);
				}
#endif
				//      bitSet(PINB,1); // used to visualize timing with LED or oscilloscope
			} // end of if (loopCount >= (SAMPLE_BUFFER - 1))

			// increment loopCount after writing all the sample data to
			// the arrays
			loopCount = loopCount++;

			// Increment the fractional seconds count
			mscount = mscount + 25;

			//  bitSet(PINB,1); // toggle LED

			goToSleep(); // call the goToSleep function (below)
		
		//***************************************************************
		//***************************************************************	
		} else if (newtime.minute() < STARTMINUTE){
			// If it is less than STARTMINUTE time, check to see if 
			// it is wakeMinute time. If so, use goToSleep, else if not use
			// lowPowerSleep
			//================================================================
			//================================================================
			if (newtime.minute() == wakeMinute){
				
#if ECHO_TO_SERIAL
				if (newtime.second() % 10 == 0){
					printTimeSerial(newtime, mscount);
					Serial.println(F("Wake minute, goToSleep"));
					delay(40);
				}
#endif 
				// Since it is the wakeMinute, we'll idle in the 
				// goToSleep cycle (returning f_wdt = 1 on interrupt from
				// TIMER2) until the minute rolls over to start recording
				// data.
				goToSleep();
			//================================================================
			//================================================================
			} else if (newtime.minute() != wakeMinute){
				
#if ECHO_TO_SERIAL
				printTimeSerial(newtime, mscount);
				Serial.println(F("Going to deep sleep"));
				delay(40);
#endif 
				TIMSK2 = 0; // stop TIMER2 interrupts
				// Turn off the RTC's 32.768kHz clock signal
				RTC.enable32kHz(false);
				// Go into low power sleep mode with watchdog timer
				lowPowerSleep();
				// From here, f_wdt will be set to 2 on interrupt from the
				// watchdog timer, and the code in the (f_wdt == 2) statement
				// below will be executed
			}
		//***************************************************************
		//***************************************************************
		} else if (newtime.minute() > endMinute && newtime.minute() != wakeMinute){
			
#if ECHO_TO_SERIAL
			printTimeSerial(newtime, mscount);
			Serial.println(F("Going to deep sleep, outer f_wdt = 1"));
			delay(40);
#endif
			TIMSK2 = 0; // stop TIMER2 interrupts
			// If we are past endMinute, enter lowPowerSleep (shuts off TIMER2)
			// Turn off the RTC's 32.768kHz clock signal
			RTC.enable32kHz(false);
			// Go into low power sleep mode with watchdog timer
			// The watchdog interrupt will return f_wdt = 2 after this point
			lowPowerSleep();
		//******************************************************************
		//******************************************************************	
		} else if (newtime.minute() == wakeMinute && wakeMinute == 59  && endMinute != wakeMinute){
			// Handle the special case when logging should start on minute 0,
			// but wakeMinute will be 59, and therefore isn't less than
			// STARTMINUTE. Reenter goToSleep mode. The third test, with
			// endMinute != wakeMinute, takes care of the additional special case
			// where the user wants to record continuously from 0 to 59 minutes,
			// so the endMinute would be equal to wakeMinute (that case should be
			// dealt with by the first statement in the block above where data 
			// collection happens). 
#if ECHO_TO_SERIAL
				if (newtime.second() % 10 == 0){
					printTimeSerial(newtime, mscount);
					Serial.println(F("Wake minute, goToSleep"));
					delay(40);
				}
#endif 
			goToSleep();
			
		} // end of if(f_wdt == 1) statement
//22222222222222222222222222222222222222222222222222222222222222222222222222222
//22222222222222222222222222222222222222222222222222222222222222222222222222222		
// Below here we are coming back from lowPowerSleep mode, when f_wdt == 2	
// This simply checks to see if it's a wakeMinute or not, and then either
// enters lowPowerSleep (not wakeMinute) or restarts TIMER2 and goes to 
// the goToSleep mode (is wakeMinute).		
	} else if (f_wdt == 2) {
		// When f_wdt == 2, we are coming back from low-power sleep mode
		f_wdt = 0; // always reset f_wdt flag value
		
		// Get a new time reading from the real time clock
		newtime = RTC.now();
		//===================================================================
		//===================================================================
		if (newtime.minute() != wakeMinute){
			
#if ECHO_TO_SERIAL
			printTimeSerial(newtime, mscount);
			Serial.println(F("Still in deep sleep"));
			delay(40);
#endif
			// Flash LED to indicate that we've come back from low
			// power sleep mode (and are about to go back into it).
			digitalWrite(LED, HIGH);
			delay(3);
			digitalWrite(LED, LOW);
			// If it is not yet the wakeMinute, just go back to 
			// low power sleep mode
			lowPowerSleep();
		//===================================================================
		//===================================================================
		} else if (newtime.minute() == wakeMinute){
			
#if ECHO_TO_SERIAL
			printTimeSerial(newtime, mscount);
			Serial.println(F("Restarting TIMER2, return to goToSleep mode"));
			delay(40);
#endif
			// If it is the wakeMinute, restart TIMER2
			newtime = startTIMER2(true,newtime);
			// Start a new file 
			//initFileName(newtime); // LPM commented out so new files only start on new days
			// Go back to sleep with TIMER2 interrupts activated
			goToSleep();
			// From here, f_wdt will be set to 1 on each interrupt from TIMER2
			// and the data-taking code above will be used
		}

	}
	// Set interrupt on pin D2, called when D2 goes low. Causes program to
	// stop completely (see endRun function below).
	attachInterrupt (0, endRun, LOW);

}
// END OF MAIN LOOP
//--------------------------------------------------------------


//--------------------------------------------------------------
// FUNCTIONS

//-----------------------------------------------------------------------------
// This Interrupt Service Routine (ISR) is called every time the
// TIMER2_OVF_vect goes high (=1), which happens when TIMER2
// overflows. The ISR doesn't care if the AVR is awake or
// in SLEEP_MODE_PWR_SAVE, it will still roll over and run this
// routine. If the AVR is in SLEEP_MODE_PWR_SAVE, the TIMER2
// interrupt will also reawaken it. This is used for the goToSleep() function
ISR(TIMER2_OVF_vect) {
	if (f_wdt == 0) { // if flag is 0 when interrupt is called
		f_wdt = 1; // set the flag to 1
	} else {
#if ECHO_TO_SERIAL
		Serial.print(F("TIMER2 fired, but it's "));
		Serial.println(f_wdt);
		delay(40);
#endif
	}
}

//-----------------------------------------------------------------------------
// goToSleep function. When called, this puts the AVR to
// sleep until it is awakened by an interrupt (TIMER2 in this case)
// This is a higher power sleep mode than the lowPowerSleep function uses.
void goToSleep()
{
	byte adcsra, mcucr1, mcucr2;
	// bitSet(PINB,1); // flip pin PB1 (digital pin 9) to indicate start of sleep cycle

	// Cannot re-enter sleep mode within one TOSC cycle. 
	// This provides the needed delay.
	OCR2A = 0; // write to OCR2A, we're not using it, but no matter
	while (ASSR & _BV(OCR2AUB)) {} // wait for OCR2A to be updated

	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	adcsra = ADCSRA; // save the ADC Control and Status Register A
	ADCSRA = 0; // disable ADC
	sleep_enable();

	// Do not interrupt before we go to sleep, or the
	// ISR will detach interrupts and we won't wake.
	noInterrupts ();
	
	wdt_disable(); // turn off the watchdog timer
	
	//ATOMIC_FORCEON ensures interrupts are enabled so we can wake up again
	ATOMIC_BLOCK(ATOMIC_FORCEON) { 
		// Turn off the brown-out detector
		mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE); 
		mcucr2 = mcucr1 & ~_BV(BODSE);
		MCUCR = mcucr1; //timed sequence
		// BODS stays active for 3 cycles, sleep instruction must be executed 
		// while it's active
		MCUCR = mcucr2; 
	}
	// We are guaranteed that the sleep_cpu call will be done
	// as the processor executes the next instruction after
	// interrupts are turned on.
	interrupts ();  // one cycle, re-enables interrupts
	sleep_cpu(); //go to sleep
	//wake up here
	sleep_disable(); // upon wakeup (due to interrupt), AVR resumes here

	//    bitSet(PINB,1); // flip Pin PB1 (digital pin 9) to indicate wakeup
	//    ADCSRA = adcsra; //restore ADCSRA
}

//-----------------------------------------------------------------------------
// lowPowerSleep function
// This sleep version uses the watchdog timer to sleep for 8 seconds at a time
// Because the watchdog timer isn't super accurate, the main program will leave
// this sleep mode once you get within 1 minute of the start time for a new 
// sampling period, and start using the regular goToSleep() function that 
// uses the more accurate TIMER2 clock source. 
void lowPowerSleep(void){
	
#if ECHO_TO_SERIAL
	Serial.print("lowPowerSleep ");
	Serial.println(millis());
	delay(40);
#endif
	
	bitSet(PIND,6); // flip Pin PD6 (digital pin 6) to indicate sleep
	
	/* It seems to be necessary to zero out the Asynchronous clock status 
	 * register (ASSR) before enabling the watchdog timer interrupts in this
	 * process. 
	 */
	ASSR = 0;  
	TIMSK2 = 0; // stop timer 2 interrupts
	// Cannot re-enter sleep mode within one TOSC cycle. 
	// This provides the needed delay.
	OCR2A = 0; // write to OCR2A, we're not using it, but no matter
	while (ASSR & _BV(OCR2AUB)) {} // wait for OCR2A to be updated

	ADCSRA = 0;   // disable ADC
	set_sleep_mode (SLEEP_MODE_PWR_DOWN);  // specify sleep mode
	sleep_enable();
	// Do not interrupt before we go to sleep, or the
	// ISR will detach interrupts and we won't wake.
	noInterrupts ();
	//--------------------------------------------------------------------------
	// Set up Watchdog timer for long term sleep

	// Clear the reset flag first
	MCUSR &= ~(1 << WDRF);

	// In order to change WDE or the prescaler, we need to
	// set WDCE (This will allow updates for 4 clock cycles).
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	// Enable watchdog interrupt (WDIE), and set 8 second delay
	WDTCSR = bit(WDIE) | bit(WDP3) | bit(WDP0); 
	wdt_reset();

	// Turn off brown-out enable in software
	// BODS must be set to one and BODSE must be set to zero within four clock 
	// cycles
	MCUCR = bit (BODS) | bit (BODSE);
	// The BODS bit is automatically cleared after three clock cycles
	MCUCR = bit (BODS);
	// We are guaranteed that the sleep_cpu call will be done
	// as the processor executes the next instruction after
	// interrupts are turned on.
	interrupts ();  // one cycle, re-enables interrupts
	sleep_cpu ();   // one cycle, going to sleep now, wake on interrupt
	// The AVR is now asleep. In SLEEP_MODE_PWR_DOWN it will only wake
	// when the watchdog timer counter rolls over and creates an interrupt
	//-------------------------------------------------------------------
	// disable sleep as a precaution after waking
	sleep_disable();
	bitSet(PIND,6); // flip Pin PD6 (digital pin 6) to indicate wakeup
}

//--------------------------------------------------------------------------
// Interrupt service routine for when the watchdog timer counter rolls over
// This is used after the user has entered the lowPowerSleep function.
ISR(WDT_vect) {
	if (f_wdt == 0) { // if flag is 0 when interrupt is called
		f_wdt = 2; // set the flag to 2
	} else {
#if ECHO_TO_SERIAL
		Serial.print(F("WDT fired, but f_wdt = "));
		Serial.println(f_wdt);
		delay(40);
#endif
	}
}

//--------------------------------------------------------------
// startTIMER2 function
// Starts the 32.768kHz clock signal being fed into XTAL1/2 to drive the
// quarter-second interrupts used during data-collecting periods. 
// Set the 1st argument true if using the 32kHz signal from a DS3231 Chronodot
// real time clock, otherwise set the argument false if using a 32.768kHz 
// clock crystal on XTAL1/XTAL2. Also supply a current DateTime time value. 
// This function returns a DateTime value that can be used to show the 
// current time when returning from this function. 
DateTime startTIMER2(bool start32k, DateTime currTime){
	TIMSK2 = 0; // stop timer 2 interrupts
	if (start32k){
		RTC.enable32kHz(true);
		ASSR = _BV(EXCLK); // Set EXCLK external clock bit in ASSR register
		// The EXCLK bit should only be set if you're trying to feed the
		// 32.768 clock signal from the Chronodot into XTAL1. 
	}
	ASSR = ASSR | _BV(AS2); // Set the AS2 bit, using | (OR) to avoid
	// clobbering the EXCLK bit that might already be set. This tells 
	// TIMER2 to take its clock signal from XTAL1/2
	TCCR2A = 0; //override arduino settings, ensure WGM mode 0 (normal mode)
	// Set up TCCR2B register (Timer Counter Control Register 2 B) to use the 
	// desired prescaler on the external 32.768kHz clock signal. Depending on 
	// which bits you set high among CS22, CS21, and CS20, different 
	// prescalers will be used. See Table 18-9 on page 158 of the AVR 
	// datasheet.
	//  TCCR2B = 0;  // No clock source (Timer/Counter2 stopped)
	// no prescaler -- TCNT2 will overflow once every 0.007813 seconds (128Hz)
	//  TCCR2B = _BV(CS20) ; 
	// prescaler clk/8 -- TCNT2 will overflow once every 0.0625 seconds (16Hz)
	//  TCCR2B = _BV(CS21) ; 
#if SAMPLES_PER_SECOND == 4
	// prescaler clk/32 -- TCNT2 will overflow once every 0.25 seconds
	TCCR2B = _BV(CS21) | _BV(CS20); 
#endif
	// Pause briefly to let the RTC roll over a new second
	DateTime starttime = currTime;
	// Cycle in a while loop until the RTC's seconds value updates
	while (starttime.second() == currTime.second()) {
		delay(1);
		currTime = RTC.now(); // check time again
	}

	TCNT2 = 0; // start the timer at zero
	// wait for the registers to be updated
	while (ASSR & (_BV(TCN2UB) | _BV(TCR2AUB) | _BV(TCR2BUB))) {} 
	TIFR2 = _BV(OCF2B) | _BV(OCF2A) | _BV(TOV2); // clear the interrupt flags
	TIMSK2 = _BV(TOIE2); // enable the TIMER2 interrupt on overflow
	// TIMER2 will now create an interrupt every time it rolls over,
	// which should be every 0.25 seconds regardless of whether the
	// AVR is awake or asleep.
	f_wdt = 0;
	return currTime;
}

//--------------------------------------------------------------
// writeToSD function. This formats the available data in the
// data arrays and writes them to the SD card file in a
// comma-separated value format.
void writeToSD (void) {
	bitSet(PINB, 1); // Toggle LED for monitoring
	bitSet(PIND, 7); // Toggle Arduino pin 7 for oscilloscope monitoring
	
	// Reopen logfile. If opening fails, notify the user
	if (!logfile.isOpen()) {
		if (!logfile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
			bitSet(PINB, 0); // toggle ERRLED to show error
#if ECHO_TO_SERIAL
			sd.errorHalt("opening file for write failed");
#endif
		}
	}
	// Step through each element of the sample data arrays
	// and write them to the SD card
	for (int i = 0; i < SAMPLE_BUFFER; i++) {
		// Write the unixtime
		logfile.print(unixtimeArray[i], DEC);
		logfile.print(F(","));
		// Convert the stored unixtime back into a DateTime
		// variable so we can write out the human-readable
		// version of the time stamp. There is a function
		// DateTime() that takes a uint32_t value and converts
		// to a DateTime object.
		DateTime tempTime = DateTime(unixtimeArray[i]);
		logfile.print(tempTime.year(), DEC);
		logfile.print(F("-"));
		logfile.print(tempTime.month(), DEC);
		logfile.print(F("-"));
		logfile.print(tempTime.day(), DEC);
		logfile.print(F(" "));
		logfile.print(tempTime.hour(), DEC);
		logfile.print(F(":"));
		logfile.print(tempTime.minute(), DEC);
		logfile.print(F(":"));
		logfile.print(tempTime.second(), DEC);
		logfile.print(F(","));
		logfile.print(mscountArray[i], DEC);
		logfile.print(F(","));
		
		// Write out pressure in millibar
		// Begin by converting the floating point value of pressure to
		// a string, truncating at 2 digits of precision
		dtostrf(pressureArray[i], precision+3, precision, pressBuffer);
		// Then print the value to the logfile. 
		logfile.print(pressBuffer);
		// logfile.print(pressureArray[i], DEC);
		logfile.print(F(","));
		
		// Write out temperature in Celsius
		// Begin by converting the floating point value of temperature to
		// a string, truncating at 2 digits of precision
		dtostrf(tempCArray[i], precision+3, precision, tempBuffer);
		logfile.println(tempBuffer);
		// logfile.println(tempCArray[i], DEC);
		logfile.sync(); // flush all data to SD card
	}
	//   DateTime t1 = DateTime(unixtimeArray[3]);
	//   // If the seconds value is 30, update the file modified timestamp
	//   if (t1.second() % 30 == 0){
	//     logfile.timestamp(T_WRITE, t1.year(),t1.month(),t1.day(),t1.hour(),t1.minute(),t1.second());
	//   }
	logfile.close(); // close file again
	bitSet(PIND, 7); // Toggle Arduino pin 7 for oscilloscope monitoring
	bitSet(PINB, 1); // Toggle LED for monitoring
}

//------------------------------------------------------------------------------
// initFileName - a function to create a filename for the SD card based
// on the 2-digit year, month, and day. The character array 'filename'
// was defined as a global variable at the top of the sketch
void initFileName(DateTime time1) {
	// Insert 2-digit year, month, and date into filename[] array
	// decade (2014-2000) = 14/10 = 1
	filename[0] = ((time1.year() - 2000) / 10) + '0'; 
	// year  (2014-2000) = 14%10 = 4
	filename[1] = ((time1.year() - 2000) % 10) + '0'; 
	if (time1.month() < 10) {
		filename[2] = '0';
		filename[3] = time1.month() + '0';
	} else if (time1.month() >= 10) {
		filename[2] = (time1.month() / 10) + '0';
		filename[3] = (time1.month() % 10) + '0';
	}
	if (time1.day() < 10) {
		filename[4] = '0';
		filename[5] = time1.day() + '0';
	} else if (time1.day() >= 10) {
		filename[4] = (time1.day() / 10) + '0';
		filename[5] = (time1.day() % 10) + '0';
	}
	// Next change the counter on the end of the filename
	// (digits 6+7) to increment count for files generated on
	// the same day. This shouldn't come into play
	// during a normal data run, but can be useful when 
	// troubleshooting.
	for (uint8_t i = 0; i < 100; i++) {
		filename[6] = i / 10 + '0';
		filename[7] = i % 10 + '0';
		if (!sd.exists(filename)) {
			// when sd.exists() returns false, this block
			// of code will be executed to open the file
			if (!logfile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
				// If there is an error opening the file, notify the
				// user. Otherwise, the file is open and ready for writing
#if ECHO_TO_SERIAL
				sd.errorHalt("opening file for write failed");
#endif
				// Turn both inidicator LEDs on to indicate a failure
				// to create the log file
				bitSet(PINB, 0); // Toggle error led on PINB0 (D8 Arduino)
				bitSet(PINB, 1); // Toggle indicator led on PINB1 (D9 Arduino)
			}
			break; // Break out of the for loop when the
			// statement if(!logfile.exists())
			// is finally false (i.e. you found a new file name to use).
		} // end of if(!sd.exists())
	} // end of file-naming for loop
#if ECHO_TO_SERIAL
	Serial.print(F("Logging to: "));
	Serial.println(filename);
#endif
	// write a header line to the SD file
	logfile.println(F("POSIXt,DateTime,frac.seconds,Pressure.mbar,TempC"));
	// Sync and close the logfile for now.
	logfile.sync();
	// Update the file's creation date, modify date, and access date.
	logfile.timestamp(T_CREATE, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.timestamp(T_WRITE, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.timestamp(T_ACCESS, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.close();

} // end of initFileName function




//------------------------------------------------------------------------------
// endRun function.
// If user presses button to trigger interrupt 0 (physical pin 4, Arduino D2)
// then stop taking data and just flash the LED once in a while while sleeping
void endRun ()
{
#if ECHO_TO_SERIAL
	printTimeSerial(newtime, mscount);
	Serial.println(F("Ending program"));
#endif
	// cancel sleep as a precaution
	sleep_disable();
	// must do this as the pin will probably stay low for a while
	detachInterrupt (0);
	interrupts(); // reenable global interrupts (including TIMER0 for millis)

	// Turn off the RTC's 32.768kHz clock signal if it's not already off
	RTC.enable32kHz(false);
	TIMSK2 = 0; // stop timer 2 interrupts

	// Create a final time stamp for the file's modify date
	DateTime time1 = DateTime(unixtimeArray[0]);
	logfile.timestamp(T_WRITE, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.timestamp(T_ACCESS, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());

	logfile.close(); // close file again just to be sure
	digitalWrite(LED, LOW); // Turn off LED if it was on

	//--------------------------------------------------------------------------

	// At this point we'll enter into an endless loop
	// where the LED will flash briefly, then go back
	// to sleep over and over again. Only a full reset
	// will restart the logging process.
	while (1) {
		digitalWrite(LED, HIGH);
		delay(3);
		digitalWrite(LED, LOW);
		lowPowerSleep();  // go back to sleep
		f_wdt = 0; // reset flag each time through
	}
}  // end of endRun

//------------------------------------------------------------------------------
// printTimeSerial function
// Just a function to print a formatted date and time to the serial monitor
void printTimeSerial(DateTime newtime, byte mscount){
	Serial.print(newtime.year(), DEC);
	Serial.print(F("/"));
	Serial.print(newtime.month(), DEC);
	Serial.print(F("/"));
	Serial.print(newtime.day(), DEC);
	Serial.print(F(" "));
	Serial.print(newtime.hour(), DEC);
	Serial.print(F(":"));
	if (newtime.minute() < 10) {
		Serial.print(F("0"));
	}
	Serial.print(newtime.minute(), DEC);
	Serial.print(F(":"));
	if (newtime.second() < 10) {
		Serial.print(F("0"));
	}
	Serial.print(newtime.second(), DEC);
	Serial.print(F("."));
	Serial.println(mscount);
}



//---------------------------------------------------------------------------
// freeRam function. Used to report back on the current available RAM while
// the sketch is running. A 328P AVR has only 2048 bytes of RAM available
// so this should return a value between 0 and 2048. If RAM drops to
// zero you will start seeing weird behavior as bits of memory are accidentally
// overwritten, destroying normal functioning in the process.
int freeRam () {
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}




