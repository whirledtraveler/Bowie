/* Sleep_tests17  

  This sketch implements a button on pin 2 (interrupt 0) to stop the 
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
32k crystal ----(XTAL2 / TOSC2)  PB7  |10      19|  PB5  (SCK)
                           (T1)  PD5  |11      18|  PB4  (MISO)
                         (AIN0)  PD6  |12      17|  PB3  (MOSI / OC2)
                         (AIN1)  PD7  |13      16|  PB2  (SS / OC1B)
                         (ICP1)  PB0  |14      15|  PB1  (OC1A)  --- LED --/\/\-- GND
                                       +--------+                          560ohm
  
  
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
  
  v17 adds the freeRam function so I can see what's going on with 
  memory usage. 
  
  
  Create numeric arrays to hold several readings...
  uint32_t unixtimes[4];
  uint8_t mscount[4];
  float pressArray[4];
  float tempCArray[4];
  When a second rolls over, write the contents of those arrays to 
  the SD card.
  */
#include <SPI.h>
#include <Wire.h>
#include <RTClib.h>
#include <RTC_DS3231.h>
#include <MS5803_14.h>
#include <SD.h>

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/atomic.h>

#define LED 9 // Arduino pin D9, physical pin 15 on AVR

#define ECHO_TO_SERIAL 0 // echo data to serial port, set to 0 to turn off

#define SAMPLES_PER_SECOND 4// number of samples taken per second

// If we were using the SQW output from the Chronodot, we'd
// need to define the desired frequency here
//#define SQW_FREQ DS3231_SQW_FREQ_1     //  1 Hz
//#define SQW_FREQ DS3231_SQW_FREQ_1024     //  1024Hz

const byte chipSelect = 10; // define the Chip Select pin for SD card
char filename[] = "LOGGER00.CSV";

// Declare data arrays
uint32_t unixtimeArray[SAMPLES_PER_SECOND]; // store unixtime values temporarily
int mscountArray[SAMPLES_PER_SECOND]; // store mscount values temporarily
float pressureArray[SAMPLES_PER_SECOND]; // store pressure readings temporarily
float tempCArray[SAMPLES_PER_SECOND]; // store temperature readings temporarily
byte loopCount = 0; // counter to keep track of data sampling loops

volatile int f_wdt = 1; // TIMER2 flag (watchdog or TIMER2 counter)
volatile int mscount = 0; // millisecond count for timestamp
DateTime oldtime; // used to track time in main loop


RTC_DS3231 RTC;
MS_5803 sensor = MS_5803(512);
File logfile;  // for sd card, this is the file object to be written to

// Define an error function to print any errors to 
// the serial monitor
void error(char *str){
 #if ECHO_TO_SERIAL 
 Serial.print("error: ");
 Serial.println(str);
 #endif 
  digitalWrite(LED, HIGH); // light permanently
 while(1); 
}

//---------------------------------------------------------
void setup() {
  // Set indicator LED as output
  pinMode(LED, OUTPUT);
  // Set interrupt 2 pin as input, use internal pullup resistor 
  pinMode(2, INPUT_PULLUP); 
//  digitalWrite(LED, HIGH); // Start with indicator LED on
  
  #if ECHO_TO_SERIAL
  Serial.begin(57600);
  Serial.println();
  delay(500);  
  Serial.println("Starting up...");
  #endif
  
  pinMode(chipSelect, OUTPUT);  // set chip select pin for SD card to output
  
  // Initialize the SD card object
  if (!SD.begin(chipSelect)){
   #if ECHO_TO_SERIAL 
   Serial.println("SD initialization failed");
   #endif
   return; 
  }
  #if ECHO_TO_SERIAL
  Serial.println("SD initialization done.");
  #endif
  
  // create new file on SD card
//  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++){
   filename[6] = i/10 + '0';
   filename[7] = i%10 + '0';
   if (!SD.exists(filename)){
     // only open new file if it doesn't exist
     logfile = SD.open(filename, FILE_WRITE);
     break;
   } 
  }
  if (! logfile){
   error("couldn't create file"); 
  }
  
  #if ECHO_TO_SERIAL
  Serial.print("Logging to: ");
  Serial.println(filename);
  #endif
  // write a header line to the SD file
  logfile.println("POSIXt,DateTime,ms,Pressure.mbar,TempC");
  logfile.close();
  
  //--------RTC SETUP ------------
  Wire.begin();
  RTC.begin(); // Start Chronodot
  RTC.enable32kHz(false); // Stop 32.768kHz output from Chronodot
  // The Chronodot can also put out several different square waves
  // on its SQW pin (1024, 4096, 8192 Hz), though I don't use them
  // in this sketch. The code below disables the SQW output.
  RTC.SQWEnable(false); // Stop the SQW output pin
//  RTC.SQWFrequency(SQW_FREQ); // set SQW frequency

  sensor.initializeMS_5803(); // Start MS5803 pressure sensor
  
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
  TCCR2B = _BV(CS21) | _BV(CS20); // prescaler clk/32 -- TCNT2 will overflow once every 0.25 seconds
//    TCCR2B = _BV(CS22) ; // prescaler clk/64 -- TCNT2 will overflow once every 0.5 seconds
  //  TCCR2B = _BV(CS22) | _BV(CS20); // prescaler clk/128 -- TCNT2 will overflow once every 1 seconds
//  TCCR2B = _BV(CS22) | _BV(CS21); // prescaler clk/256 -- TCNT2 will overflow once every 2 seconds
//   TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20) ; // prescaler clk/1024 -- TCNT2 will overflow once every 8 seconds
  TCNT2 = 0; // start the timer at zero
  while (ASSR & (_BV(TCN2UB) | _BV(TCR2AUB) | _BV(TCR2BUB))) {} // wait for the registers to be updated
  TIFR2 = _BV(OCF2B) | _BV(OCF2A) | _BV(TOV2); // clear the interrupt flags
  TIMSK2 = _BV(TOIE2); // enable the TIMER2 interrupt on overflow
  // TIMER2 will now create an interrupt every time it rolls over,
  // which should be every 0.25 seconds regardless of whether the
  // AVR is awake or asleep.
  
  // Pause briefly to let the RTC roll over a new second <-- I don't think this is working as I hoped
  // Begin by defining two DateTime variables with the current time
  DateTime starttime, newtime = RTC.now();
  // Cycle in the while loop until the seconds value updates
  while (starttime.second() == newtime.second()){
    delayMicroseconds(1);
    newtime = RTC.now(); // check time again
  }
  TCNT2 = 0; // reset the TIMER2 counter at zero again
  oldtime = RTC.now(); // get starting time
}

//----------------------------------------------
// Main loop
// We will enter the if() statement 
// immediately when we get to the main loop for
// the first time, because f_wdt was set = 1 during
// the initial compile. 
void loop() {
  if(f_wdt == 1) {
    f_wdt = 0; // reset interrupt flag
//    bitSet(PINB,1); // used to visualize timing with LED or oscilloscope 
    DateTime newtime = RTC.now();
    // Check to see if the current second value
    // is equal to oldtime.second(). If so, we
    // are still in the same second. If not, 
    // the mscount value should be reset to 0
    // and oldtime updated to equal now.
    if (oldtime.second() != newtime.second()){
      mscount = 0; // reset mscount
      oldtime = newtime; // update oldtime
      loopCount = 0;  // reset loopCount
    }
    // Save current time to unixtimeArray 
    unixtimeArray[loopCount] = newtime.unixtime();
    mscountArray[loopCount] = mscount;
    
//    bitSet(PINB,1); // used to visualize timing with LED or oscilloscope

    // Use readSensor() function to get pressure and temperature reading. 
    sensor.readSensor(); 
    pressureArray[loopCount] = sensor.pressure();
    tempCArray[loopCount] = sensor.temperature();
        
    // Now if loopCount is equal to the value in SAMPLES_PER_SECOND
    // (minus 1 for zero-based counting), then write out the contents
    // of the sample data arrays to the SD card.
    if (loopCount == (SAMPLES_PER_SECOND - 1)){
      // Call the writeToSD function to output the data array contents
      // to the SD card
      bitSet(PINB,1);
      writeToSD();
      bitSet(PINB,1);
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
//  bitSet(PINB, 1); // toggle LED 
//  logfile = SD.open(filename, FILE_WRITE); // reopen file
//  logfile.print(newtime.unixtime());
//  logfile.print(",");
//  logfile.print(newtime.year(), DEC);  
//  logfile.print("-");
//  logfile.print(newtime.month(), DEC);
//  logfile.print("-");
//  logfile.print(newtime.day(), DEC);
//  logfile.print(" ");
//  logfile.print(newtime.hour(), DEC);
//  logfile.print(":");
//  logfile.print(newtime.minute(), DEC);
//  logfile.print(":");
//  logfile.print(newtime.second(), DEC);
//  logfile.print(",");
//  logfile.print(mscount, DEC);
//  logfile.print(",");
//  logfile.print(sensor.pressure());
//  logfile.print(",");
//  logfile.println(sensor.temperature());
//  logfile.flush();  
//  logfile.close(); // close file again 
//  delay(5);  
  
  
  // Increment the milliseconds count (mscount)
  mscount = mscount + 250;
 #if ECHO_TO_SERIAL   
    // Show pressure
    Serial.print("Pressure = ");
    Serial.print(sensor.pressure());
    Serial.print(" mbar, ");
    Serial.print(sensor.temperature());
    Serial.println(" C"); 
    Serial.print("Free RAM: "); Serial.println(freeRam());
    delay(40); 
  #endif  
//  bitSet(PINB,1); // toggle LED
  
// Set interrupt on pin D2, called when D2 goes low
  attachInterrupt (0, endRun, LOW);
  
  goToSleep(); // call the goToSleep function (below)
  }
  
}

void writeToSD (void) {
  logfile = SD.open(filename, FILE_WRITE); // reopen file
  // Step through each element of the sample data arrays
  // and write them to the SD card
   for (int i = 0; i < SAMPLES_PER_SECOND; i++) {
      // Write the unixtime      
      logfile.print(unixtimeArray[i], DEC);
      logfile.print(",");
      // Convert the stored unixtime back into a DateTime
      // variable so we can write out the human-readable 
      // version of the time stamp. There is a function 
      // DateTime() that takes a uint32_t value and converts
      // to a DateTime object.
      DateTime tempTime = DateTime(unixtimeArray[i]);
      logfile.print(tempTime.year(), DEC);  
      logfile.print("-");
      logfile.print(tempTime.month(), DEC);
      logfile.print("-");
      logfile.print(tempTime.day(), DEC);
      logfile.print(" ");
      logfile.print(tempTime.hour(), DEC);
      logfile.print(":");
      logfile.print(tempTime.minute(), DEC);
      logfile.print(":");
      logfile.print(tempTime.second(), DEC);
      logfile.print(",");
      logfile.print(mscountArray[i], DEC);
      logfile.print(",");
      logfile.print(pressureArray[i], DEC);
      logfile.print(",");
      logfile.println(tempCArray[i], DEC);
      logfile.flush();
   }   
  logfile.close(); // close file again

}


// This Interrupt Service Routine (ISR)is called every time the
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

//--------------------------------------------
// Sleep function. When called, this puts the AVR to
// sleep until it is wakened by an interrupt (TIMER2 in our case)
void goToSleep()
{
    byte adcsra, mcucr1, mcucr2;
//    bitSet(PINB,1); // flip pin PB1 (digital //Cannot re-enter sleep mode within one TOSC cycle. This provides the needed delay.
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
      delay(1);
      digitalWrite(LED, LOW);      
      goToSleepPermanently();  // go back to sleep      
      f_wdt = 0; // reset flag each time through
  }
}  // end of endRun

//-----------------------------------------------------
// Interrupt service routine for when the watchdog timer counter rolls over
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
  //-----------------------------------------------------------
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

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
                 art : (int) __brkval); 
}
