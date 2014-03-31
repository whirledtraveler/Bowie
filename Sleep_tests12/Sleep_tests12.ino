/* Sleep_tests12

  Hardware:
  ATMEGA328P on a breadboard, with bootloader set to run on internal
  8MHz oscillator. LED on pin D9. Powered from a 3V battery (2xAA).
 
                                       +---\/---+                      10k ohm
                        (RESET)  PC6  |1       28|  PC5  (ADC5 / SCL) --/\/\-- +3V3
                          (RXD)  PD0  |2       27|  PC4  (ADC4 / SDA) --/\/\-- +3V3
                          (TXD)  PD1  |3       26|  PC3  (ADC3)         10k ohm
                         (INT0)  PD2  |4       25|  PC2  (ADC2)
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
  */
#include <SPI.h>
#include <Wire.h>
#include <RTClib.h>
#include <RTC_DS3231.h>
#include <MS_5803.h>
#include <SD.h>

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/atomic.h>

#define LED 9 // Arduino pin D9, physical pin 15 on AVR

#define ECHO_TO_SERIAL 1 // echo data to serial port, set to 0 to turn off

// If we were using the SQW output from the Chronodot, we'd
// need to define the desired frequency here
//#define SQW_FREQ DS3231_SQW_FREQ_1     //  1 Hz
//#define SQW_FREQ DS3231_SQW_FREQ_1024     //  1024Hz

const byte chipSelect = 10; // define the Chip Select pin for SD card

volatile int f_wdt = 1; // TIMER2 flag (watchdog or TIMER2 counter)
volatile int mscount = 0; // millisecond count for timestamp
DateTime oldtime; // used to track time in main loop


RTC_DS3231 RTC;
MS_5803 sensor = MS_5803(5);
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

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH); // Start with indicator LED on
  
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
   Serial.println("Initialization failed");
   #endif
   return; 
  }
  #if ECHO_TO_SERIAL
  Serial.println("SD initialization done.");
  #endif
  
  // create new file on SD card
  char filename[] = "LOGGER00.CSV";
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
  logfile.println("POSIXt,DateTime,ms,Pressurembar,TempC");

  
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
  //  TCCR2B = _BV(CS22) ; // prescaler clk/64 -- TCNT2 will overflow once every 0.5 seconds
  //  TCCR2B = _BV(CS22) | _BV(CS20); // prescaler clk/128 -- TCNT2 will overflow once every 1 seconds
  // TCCR2B = _BV(CS22) | _BV(CS21); // prescaler clk/256 -- TCNT2 will overflow once every 2 seconds
  //    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20) ; // prescaler clk/1024 -- TCNT2 will overflow once every 8 seconds
  TCNT2 = 0; // start the timer at zero
  while (ASSR & (_BV(TCN2UB) | _BV(TCR2AUB) | _BV(TCR2BUB))) {} // wait for the registers to be updated
  TIFR2 = _BV(OCF2B) | _BV(OCF2A) | _BV(TOV2); // clear the interrupt flags
  TIMSK2 = _BV(TOIE2); // enable the TIMER2 interrupt on overflow
  // TIMER2 will now create an interrupt every time it rolls over,
  // which should be every 0.25 seconds regardless of whether the
  // AVR is awake or asleep.
  
  // Pause briefly to let the RTC roll over a new second <-- I don't think this is working as I hoped
  // Begin by defining two DateTime variables with the current time
  DateTime starttime, now = RTC.now();
  // Cycle in the while loop until the seconds value updates
  while (starttime.second() == now.second()){
    delayMicroseconds(1);
    now = RTC.now(); // check time again
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
    DateTime now = RTC.now();
    
//    bitSet(PINB,1); // used to visualize timing with LED or oscilloscope

    // Use readSensor() function to get pressure and temperature reading. 
    sensor.readSensor(); 

//    bitSet(PINB,1); // used to visualize timing with LED or oscilloscope

  #if ECHO_TO_SERIAL
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print(".");
  #endif  
    // Check to see if the current second value
    // is equal to oldtime.second(). If so, we
    // are still in the same second. If not, 
    // the mscount value should be reset to 0
    // and oldtime updated to equal now.
    if (oldtime.second() != now.second()){
      mscount = 0;
      oldtime = now;
    }
  #if ECHO_TO_SERIAL  
    Serial.println(mscount);
  #endif  
    // Increment the milliseconds count (mscount)
    mscount = mscount + 250;
 #if ECHO_TO_SERIAL   
    // Show pressure
    Serial.print("Pressure = ");
    Serial.print(sensor.pressure());
    Serial.print(" mbar, ");
    Serial.print(sensor.temperature());
    Serial.println(" C"); 
    delay(40); 
  #endif  
  bitSet(PINB, 1); // toggle LED 
  logfile.print(now.unixtime());
  logfile.print(",");
  logfile.print(now.year(), DEC);  
  logfile.print("-");
  logfile.print(now.month(), DEC);
  logfile.print("-");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print(",");
  logfile.print(mscount, DEC);
  logfile.print(",");
  logfile.print(sensor.pressure());
  logfile.print(",");
  logfile.println(sensor.temperature());  
  delay(10);  
  bitSet(PINB,1); // toggle LED
  
    goToSleep(); // call the goToSleep function (below)
  }
  
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
    bitSet(PINB,1); // flip pin PB1 (digital pin 9) to indicate start of sleep cycle
    //Cannot re-enter sleep mode within one TOSC cycle. This provides the needed delay.
    OCR2A = 0; //write to OCR2A, we're not using it, but no matter
    while (ASSR & _BV(OCR2AUB)) {} //wait for OCR2A to be updated

    sleep_enable();
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    adcsra = ADCSRA; //save the ADC Control and Status Register A
    ADCSRA = 0; //disable ADC
    ATOMIC_BLOCK(ATOMIC_FORCEON) { //ATOMIC_FORCEON ensures interrupts are enabled so we can wake up again
        mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE); //turn off the brown-out detector
        mcucr2 = mcucr1 & ~_BV(BODSE);
        MCUCR = mcucr1; //timed sequence
        MCUCR = mcucr2; //BODS stays active for 3 cycles, sleep instruction must be executed while it's active
    }
    sleep_cpu(); //go to sleep
                                   //wake up here
    sleep_disable(); // upon wakeup (due to interrupt), AVR resumes processing here
    bitSet(PINB,1); // flip Pin PB1 (digital pin 9) to indicate wakeup
//    ADCSRA = adcsra; //restore ADCSRA
}
