/* Sleep_tests21a
  
  v21a Adding code to make the LED flash when writing to SD
  
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
  8MHz oscillator. LED on pin D9. Powered from a 3V3 regulator.
 
                                       +---\/---+                      10k ohm
    GND---BUTTON--------(RESET)  PC6  |1       28|  PC5  (ADC5 / SCL) --/\/\-- +3V3
                          (RXD)  PD0  |2       27|  PC4  (ADC4 / SDA) --/\/\-- +3V3
                          (TXD)  PD1  |3       26|  PC3  (ADC3)         10k ohm
    GND---BUTTON---------(INT0)  PD2  |4       25|  PC2  (ADC2)
                         (INT1)  PD3  |5       24|  PC1  (ADC1)
                     (XCK / T0)  PD4  |6       23|  PC0  (ADC0)
                                 VCC  |7       22|  GND
                                 GND  |8       21|  AREF
32k crystal ----(XTAL1 / TOSC1)  PB6  |9       20|  AVCC
32k crystal ----(XTAL2 / TOSC2)  PB7  |10      19|  PB5  (SCK) --- CLK on SD Card
                           (T1)  PD5  |11      18|  PB4  (MISO) --- D0 on SD Card
                         (AIN0)  PD6  |12      17|  PB3  (MOSI / OC2) --- DI on SD Card
                         (AIN1)  PD7  |13      16|  PB2  (SS / OC1B) --- CS on SD Card
 GND --/\/\--- LED ------(ICP1)  PB0  |14      15|  PB1  (OC1A)  --- LED --/\/\-- GND
       560ohm                          +--------+                          560ohm
  
  
  Programmed via AVR MKII ISP. 
  
  Communicating via FTDI Friend at 57600 bps.
  
  Chronodot hooked up via I2C to pins A4 + A5 (SDA + SCL), along
  with 10k ohm pull-up resistors to 3V3. 
  
  32.768kHz watch crystal hooked up to XTAL1/XTAL2 to run TIMER2 
  (physical pins 9 + 10 on 28-pin AVR DIP chip) 
  
  MS5803 pressure sensor hooked up via I2C:
               --------------
   SCL     ---|1   MS5803  8|--- NC
   GND     ---|2           7|--- SDA
   +3V3    ---|3           6|--- +3V3
   NC      ---|4           5|--- +3V3
              --------------- 
  (NC = no connection) 
  -------------------------------------------------------------------
  The sketch:
  During setup, we disable the Chronodot's 32.768kHz 
  output. A separate 32.768kHz crystal oscillator is hooked to XTAL1/XTAL2
  to provide the timed interrupt. This saves power compared to having
  the Chronodot 32.768 pin running (the SQW pin also pulls current and 
  so we make sure it is also disabled during setup). 
  
  Then we write to the Asynchronous Timer Counter register (ASSR)
  to use AS2. The AVR will then expect a clock signal on 
  pin XTAL1/XTAL2 (AVR pin 9+10 on the 328P DIP package) that will be 
  monitored by TIMER2, an 8-bit (0-255) counter. 
  
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
  
  */
#include <SPI.h>
#include <Wire.h>
#include <RTClib.h>
#include <RTC_DS3231.h>
#include <MS5803_14.h>
#include <SdFat.h>

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/atomic.h>

#define LED 9 // Arduino pin D9, physical pin 15 on AVR
#define ERRLED 8 // Arduino pin D8, physical pin 14 on AVR

#define ECHO_TO_SERIAL 1 // echo data to serial port, set to 0 to turn off

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

volatile int f_wdt = 1; // TIMER2 flag (watchdog or TIMER2 counter)
volatile int mscount = 0; // millisecond count for timestamp
DateTime oldtime; // used to track time in main loop
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
  // Set interrupt 2 pin as input, use internal pullup resistor 
  pinMode(2, INPUT_PULLUP); 
  
  #if ECHO_TO_SERIAL
  Serial.begin(57600);
  Serial.println();
  delay(500);  
  Serial.println(F("Starting up..."));
  #endif
  
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
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)){
   #if ECHO_TO_SERIAL 
   Serial.println(F("SD initialization failed"));
   #endif
   bitSet(PINB,0); // Toggle error led on PINB0 (D8 Arduino)
   return; // this should bring the program to a halt
  }
  #if ECHO_TO_SERIAL
  Serial.println(F("SD initialization done."));
  #endif
  
  // Create new file on SD card using initFileName() function
  // This creates a filename based on the year + month + day
  // with a number indicating which number file this is on 
  // the given day. The format of the filename is
  // YYMMDDXX.CSV where XX is the counter (00-99). The
  // function also writes the column headers to the new file. 
  initFileName(starttime);
  
  // Start MS5803 pressure sensor
  sensor.initializeMS_5803(); 
  
  TIMSK2 = 0; // stop timer 2 interrupts

//  ASSR = _BV(EXCLK); // Set EXCLK external clock bit in ASSR register
  // The EXCLK bit should only be set if you're trying to feed the 
  // 32.768 clock signal from the Chronodot into XTAL1. We're not doing 
  // that here. Instead we're hooking a 32.768 watch crystal to XTAL1 + XTAL2
  // so we only need to set the AS2 bit below. Those are two slightly different
  // use cases. If you don't have a separate 32kHz watch crystal, you can 
  // hook up the Chronodot 32K line to XTAL1, with a 10k ohm pull up resistor
  // and uncomment the ASSR = _BV(EXCLK); line above. You would then also
  // change the RTC 32k enable line above to read: RTC.enable32kHz(true); so
  // that the Chronodot 32k square wave runs. 

  ASSR = ASSR | _BV(AS2); // Set the AS2 bit, using | (OR) to avoid 
                          // clobbering the EXCLK bit that might already be set

  TCCR2A = 0; //override arduino settings, ensure WGM mode 0 (normal mode)
  // Set up TCCR2B register (Timer Counter Control Register 2 B) to use the desired 
  // prescaler on the external 32.768kHz clock signal. Depending on which bits you set 
  // high among CS22, CS21, and CS20, different prescalers will be used. See Table 18-9
  // on page 158 of the AVR datasheet.
  //  TCCR2B = 0;  // No clock source (Timer/Counter2 stopped)
  //  TCCR2B = _BV(CS20) ; // no prescaler -- TCNT2 will overflow once every 0.007813 seconds (128Hz)  
  //  TCCR2B = _BV(CS21) ; // prescaler clk/8 -- TCNT2 will overflow once every 0.0625 seconds (16Hz)
  #if SAMPLES_PER_SECOND == 4 
    TCCR2B = _BV(CS21) | _BV(CS20); // prescaler clk/32 -- TCNT2 will overflow once every 0.25 seconds
  #endif
  
//  #if SAMPLES_PER_SECOND == 2
//    TCCR2B = _BV(CS22) ; // prescaler clk/64 -- TCNT2 will overflow once every 0.5 seconds
//  #endif
  
  //  TCCR2B = _BV(CS22) | _BV(CS20); // prescaler clk/128 -- TCNT2 will overflow once every 1 seconds
//  TCCR2B = _BV(CS22) | _BV(CS21); // prescaler clk/256 -- TCNT2 will overflow once every 2 seconds
//   TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20) ; // prescaler clk/1024 -- TCNT2 will overflow once every 8 seconds
  

  
  // Pause briefly to let the RTC roll over a new second
  // Begin by defining two DateTime variables with the current time
  DateTime newtime = RTC.now();
  starttime = newtime; 
  // Cycle in a while loop until the RTC's seconds value updates
  while (starttime.second() == newtime.second()){
    delayMicroseconds(1);
    newtime = RTC.now(); // check time again
  }
  
  TCNT2 = 0; // start the timer at zero
  while (ASSR & (_BV(TCN2UB) | _BV(TCR2AUB) | _BV(TCR2BUB))) {} // wait for the registers to be updated
  TIFR2 = _BV(OCF2B) | _BV(OCF2A) | _BV(TOV2); // clear the interrupt flags
  TIMSK2 = _BV(TOIE2); // enable the TIMER2 interrupt on overflow
  // TIMER2 will now create an interrupt every time it rolls over,
  // which should be every 0.25 seconds regardless of whether the
  // AVR is awake or asleep.

  oldtime = RTC.now(); // get starting time
  oldday = oldtime.day(); // get starting day value
}

//--------------------------------------------------------------
// Main loop
// We will enter the if() statement 
// immediately when we get to the main loop for
// the first time, because f_wdt was set = 1 during
// the initial compile. 
void loop() {
  if(f_wdt == 1) {
    f_wdt = 0; // reset interrupt flag
//    bitSet(PINB,1); // used to visualize timing with LED or oscilloscope 

    // Get a new time reading from the real time clock
    DateTime newtime = RTC.now();
    // Check to see if the current second value
    // is equal to oldtime.second(). If so, we
    // are still in the same second. If not, 
    // the mscount value should be reset to 0
    // and oldtime updated to equal now.
    if (oldtime.second() != newtime.second()){
      mscount = 0; // reset mscount
      oldtime = newtime; // update oldtime
      loopCount = 0; // reset loopCount
    }
    
    // Save current time to unixtimeArray 
    unixtimeArray[loopCount] = newtime.unixtime();
    mscountArray[loopCount] = mscount;
    
//    bitSet(PINB,1); // used to visualize timing with LED or oscilloscope

    // Use readSensor() function to get pressure and temperature reading. 
    sensor.readSensor(); 
    pressureArray[loopCount] = sensor.pressure();
    tempCArray[loopCount] = sensor.temperature();
        
    // Now if loopCount is equal to the value in SAMPLE_BUFFER
    // (minus 1 for zero-based counting), then write out the contents
    // of the sample data arrays to the SD card.
    if (loopCount >= (SAMPLE_BUFFER - 1)){
      // Check to see if a new day has started. If so, open a new file
      // with the initFileName() function
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
//      bitSet(PINB,1); // used to visualize timing with LED or oscilloscope
    }    
    
    // increment loopCount after writing all the sample data to
    // the arrays
    loopCount = loopCount++;
    
//    bitSet(PINB,1); // used to visualize timing with LED or oscilloscope

  #if ECHO_TO_SERIAL
    Serial.print(newtime.year(), DEC);
    Serial.print('/');
    Serial.print(newtime.month(), DEC);
    Serial.print('/');
    Serial.print(newtime.day(), DEC);
    Serial.print(' ');
    Serial.print(newtime.hour(), DEC);
    Serial.print(':');
    Serial.print(newtime.minute(), DEC);
    Serial.print(':');
    Serial.print(newtime.second(), DEC);
    Serial.print(".");
  #endif  

  #if ECHO_TO_SERIAL  
    Serial.println(mscount);
  #endif    
  // Increment the fractional seconds count
  mscount = mscount + 25;
 #if ECHO_TO_SERIAL   
    // Show pressure
    Serial.print(F("Pressure = "));
    Serial.print(sensor.pressure());
    Serial.print(F(" mbar, "));
    Serial.print(sensor.temperature());
    Serial.println(F(" C")); 
    Serial.print(F("Free RAM: ")); Serial.println(freeRam());
    Serial.print(F("loopCount: "));Serial.println(loopCount);
//    Serial.print("secondCount: "); Serial.println(secondCount);
    delay(40); 
  #endif  
//  bitSet(PINB,1); // toggle LED
  
// Set interrupt on pin D2, called when D2 goes low
  attachInterrupt (0, endRun, LOW);
  
  goToSleep(); // call the goToSleep function (below)
  } // end of if(f_wdt == 1) statement
  
}
// END OF MAIN LOOP
//--------------------------------------------------------------


//--------------------------------------------------------------
// OTHER FUNCTIONS

//--------------------------------------------------------------
// writeToSD function. This formats the available data in the
// data arrays and writes them to the SD card file in a 
// comma-separated value format. 
void writeToSD (void) {
  digitalWrite(LED, HIGH);
  bitSet(PIND,7); // Toggle Arduino pin 7 for oscilloscope monitoring
    // Reopen logfile. If opening fails, notify the user
  if (!logfile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
      bitSet(PINB,0); // toggle ERRLED to show error
      #if ECHO_TO_SERIAL
      sd.errorHalt("opening file for write failed");
      #endif
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
      // Write out pressure in millibars
      logfile.print(pressureArray[i], DEC);
      logfile.print(F(","));
      // Write out temperature in Celsius
      logfile.println(tempCArray[i], DEC);
      logfile.sync(); // flush all data to SD card
   }   
  logfile.close(); // close file again
  bitSet(PIND,7); // Toggle Arduino pin 7 for oscilloscope monitoring
  digitalWrite(LED, LOW);
}

//------------------------------------------------------------------------
// initFileName - a function to create a filename for the SD card based 
// on the 2-digit year, month, and day. The character array filename
// was defined as a global variable at the top of the sketch
void initFileName(DateTime currenttime){
  // Insert 2-digit year, month, and date into filename[] array
  filename[0] = ((currenttime.year()-2000)/10) + '0'; // decade (2014-2000) = 14/10 = 1 
  filename[1] = ((currenttime.year()-2000)%10) + '0'; // year  (2014-2000) = 14%10 = 4
  if (currenttime.month() < 10) {
    filename[2] = '0';
    filename[3] = currenttime.month()+ '0';
  } else if (currenttime.month() >= 10) {
    filename[2] = (currenttime.month() / 10) + '0';
    filename[3] = (currenttime.month() % 10) + '0';
  }
  if (currenttime.day() < 10) {
    filename[4] = '0';
    filename[5] = currenttime.day() + '0';
  } else if (currenttime.day() >= 10) {
    filename[4] = (currenttime.day() / 10) + '0';
    filename[5] = (currenttime.day() % 10) + '0';
  }
  // Next change the counter on the end of the filename
  // (digits 6+7) to increment count for files generated on 
  // the same day
  for (uint8_t i = 0; i < 100; i++){
   filename[6] = i/10 + '0';
   filename[7] = i%10 + '0';
   if (!sd.exists(filename)){
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
        bitSet(PINB,0); // Toggle error led on PINB0 (D8 Arduino)
        bitSet(PINB,1); // Toggle indicator led on PINB1 (D9 Arduino)       
     }
     break; // Break out of the for loop when the 
            // statement if(!logfile.exists())
            // is finally false (i.e. you found a new 
            // file name to use). 
   } // end of if(!sd.exists())
  } // end of file-naming for loop
  #if ECHO_TO_SERIAL
  Serial.print(F("Logging to: "));
  Serial.println(filename);
  #endif
  // write a header line to the SD file
  logfile.println(F("POSIXt,DateTime,ms,Pressure.mbar,TempC"));
  // Sync and close the logfile for now. 
  logfile.sync();
  logfile.close();
  
} // end of initFileName function
//------------------------------------------------------------------

// This Interrupt Service Routine (ISR) is called every time the
// TIMER2_OVF_vect goes high (=1), which happens when Timer2
// overflows. The ISR doesn't care if the AVR is awake or
// in SLEEP_MODE_PWR_SAVE, it will still roll over and run this
// routine. If the AVR is in SLEEP_MODE_PWR_SAVE, the Timer2 
// interrupt will also reawaken it. 
ISR(TIMER2_OVF_vect){
  if (f_wdt == 0) { // if flag is 0 when interrupt is called
    f_wdt = 1; // set the flag to 1
  }
}

//---------------------------------------------------------------
// goToSleep function. When called, this puts the AVR to
// sleep until it is wakened by an interrupt (TIMER2 in our case)
void goToSleep()
{
    byte adcsra, mcucr1, mcucr2;
//    bitSet(PINB,1); // flip pin PB1 (digital pin 9) to indicate start of sleep cycle
    //Cannot re-enter sleep mode within one TOSC cycle. This provides the needed delay.
    OCR2A = 0; //write to OCR2A, we're not using it, but no matter
    while (ASSR & _BV(OCR2AUB)) {} //wait for OCR2A to be updated
    
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    adcsra = ADCSRA; //save the ADC Control and Status Register A
    ADCSRA = 0; //disable ADC
    sleep_enable();
    
    // Do not interrupt before we go to sleep, or the
    // ISR will detach interrupts and we won't wake.
    noInterrupts ();
    // Set interrupt on pin D2, called when D2 goes low
//    attachInterrupt (0, endRun, LOW);
    
    ATOMIC_BLOCK(ATOMIC_FORCEON) { //ATOMIC_FORCEON ensures interrupts are enabled so we can wake up again
        mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE); //turn off the brown-out detector
        mcucr2 = mcucr1 & ~_BV(BODSE);
        MCUCR = mcucr1; //timed sequence
        MCUCR = mcucr2; //BODS stays active for 3 cycles, sleep instruction must be executed while it's active
    }
    // We are guaranteed that the sleep_cpu call will be done
    // as the processor executes the next instruction after
    // interrupts are turned on.
    interrupts ();  // one cycle, re-enables interrupts
    sleep_cpu(); //go to sleep
                                   //wake up here
    sleep_disable(); // upon wakeup (due to interrupt), AVR resumes processing here
//    bitSet(PINB,1); // flip Pin PB1 (digital pin 9) to indicate wakeup
//    ADCSRA = adcsra; //restore ADCSRA
}


//----------------------------------------------------------------------------------- 
// endRun function.
// If user presses button to trigger interrupt 0 (physical pin 4)
// then stop taking data and just flash the LED once in a while while sleeping
void endRun ()
{
  // cancel sleep as a precaution
  sleep_disable();
  // must do this as the pin will probably stay low for a while
  detachInterrupt (0);
  interrupts(); // reenable global interrupts (including TIMER0 for millis)
  TIMSK2 = 0; // stop timer 2 interrupts
  
  logfile.close(); // close file again just to be sure
  digitalWrite(LED, LOW); // Turn off LED if it was on
  //--------------------------------------------------------------------------
  // Set up Watchdog timer for long term sleep
  
  // Clear the reset flag first
  MCUSR &= ~(1<<WDRF);
  
  // Change the WDE and WDCE in the Watchdog Timer Control Register (WDTSCSR)
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  
  // Set watchdog timeout prescaler to 0.25s
//  WDTCSR = 1<<WDP2;
  // Set watchdog timeout prescaler to 0.125s
//  WDTCSR = 1<<WDP1 | 1<<WDP0;
  // Set watchdog timeout prescaler to 8s
  WDTCSR = 1<<WDP0 | 1<<WDP3;
  
  //Enable the watchdog interrupt (does not reset AVR when interrupt trips)
  WDTCSR |= _BV(WDIE);
  //--------------------------------------------------------------------------
  
  // At this point we'll enter into an endless loop
  // where the LED will flash briefly, then go back
  // to sleep over and over again. Only a full reset
  // will restart the logging process. 
  while(1){
      digitalWrite(LED, HIGH); 
      delay(3);
      digitalWrite(LED, LOW);      
      goToSleepPermanently();  // go back to sleep      
      f_wdt = 0; // reset flag each time through
  }
}  // end of endRun

//-----------------------------------------------------
// Interrupt service routine for when the watchdog timer counter rolls over
// This is used after the user has entered the endRun function, which just
// loops continuously, sleeping and using the watchdog timer to wake up
// every once in a while.
ISR(WDT_vect){
  if (f_wdt == 0) { // if flag is 0 when interrupt is called
    f_wdt = 1; // set the flag to 1
  }
}

//----------------------------------------------------
// goToSleepPermanently function. This shuts down all the peripherals and 
// send the cpu to sleep. In this configuration it is only
// awakened when the watchdog timer interrupt fires, causing 
// it to wake again. 
void goToSleepPermanently (void){
  // Entering sleep mode section. Don't change much in here
  // without knowing what you're up to.
  ADCSRA = 0;   // disable ADC
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  // specify sleep mode
  sleep_enable();
  // Do not interrupt before we go to sleep, or the
  // ISR will detach interrupts and we won't wake.
  noInterrupts ();

  // Turn off brown-out enable in software
  // BODS must be set to one and BODSE must be set to zero within four clock cycles
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





