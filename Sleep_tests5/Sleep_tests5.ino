/* Test code for reading clock, pressure sensor, and sleeping

  The watchdog timer is set to trip every
  1/4 second, wake the avr up, read the clock + pressure, and
  output to serial. Then it should go back to sleep until the 
  next watchdog timer interrupt. 

  At present this script outputs time + pressure to the serial 
  port, which takes a bit of time. It seems to miss the 4th 
  reading in a second every few seconds, so something is taking
  longer than desired.   
  
  Reading the RTC takes approx 1.2 milliseconds
  Reading the MS5803 on 4096 oversampling mode takes approx 9 ms.
  
  Tested on 3.3V 8MHz Pro Mini.  With the serial connection
  disconnected and running on battery power, it pulls around
  0.66mA to 1.0mA on average, with LED 13 flashing at each 
  WDT interrupt. I don't have an SD card hooked up yet. 
  
  2014-03-16
*/

#include <avr/sleep.h> // sleep library
#include <avr/wdt.h> // watchdog timer library
#include <Wire.h>
#include <SPI.h>
#include <RTClib.h>
#include <RTC_DS3231.h>
#include <MS_5803.h>

RTC_DS3231 RTC;
MS_5803 sensor = MS_5803(5);
volatile long c1;
volatile long c2;
volatile long c3;
volatile long c4;
const byte LED = 13; // Specify which pin the LED is attached to
volatile int f_wdt = 1; // watchdog timer flag

void setup(){
 Serial.begin(57600);
 Serial.println("Starting...");
 delay(100);
 
 Wire.begin();
 RTC.begin();
 DateTime now = RTC.now();
 sensor.initializeMS_5803();
 
 //--------------------------------------------------------------------------
  // Set up Watchdog timer
  
  // Clear the reset flag first
  MCUSR &= ~(1<<WDRF);
  
  // Change the WDE and WDCE in the Watchdog Timer Control Register (WDTSCSR)
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  
  // Set watchdog timeout prescaler to 0.25s
  WDTCSR = 1<<WDP2;
  // Set watchdog timeout prescaler to 0.125s
//  WDTCSR = 1<<WDP1 | 1<<WDP0;
  // Set watchdog timeout prescaler to 8s
//  WDTCSR = 1<<WDP0 | 1<<WDP3;
  
  //Enable the watchdog interrupt (does not reset AVR when interrupt trips)
  WDTCSR |= _BV(WDIE);
  //--------------------------------------------------------------------------
 
}

void loop() {
  
  // If the watchdog interrupt fires, do the stuff in this statement, 
  // including reading the clock and pressure sensor
  if (f_wdt == 1){
    f_wdt = 0; // reset watchdog timer flag
    // Flash LED before going to sleep
    pinMode (LED, OUTPUT);
    digitalWrite (LED, HIGH);
    delay (1);
    digitalWrite (LED, LOW);
    pinMode (LED, INPUT); // for power savings
    
    // Read real time clock
    
    DateTime now = RTC.now();
    
    // Read pressure sensor
    c3 = micros(); // record initial time
    // Use readSensor() function to get pressure and temperature reading. 
    sensor.readSensor();
    c4 = micros();  // record 2nd time
    
    c1 = micros();
    Serial.print("MS5803 read time (us): ");
    Serial.println(c4 - c3);
    // print hour:min:sec
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
    
    // Show pressure
    Serial.print("Pressure = ");
    Serial.print(sensor.pressure());
    Serial.println(" mbar");
    c2 = micros();
    
    
    // Print results to serial port
    Serial.print("Serial print time (us): ");
    Serial.println(c2 - c1);
    Serial.println();
    delay(20);
    
  }
  goToSleep(); // call the goToSleep function (below)
}


//-----------------------------------------------------
// Interrupt service routine for when the watchdog timer counter rolls over
ISR(WDT_vect){
  if (f_wdt == 0) { // if flag is 0 when interrupt is called
    f_wdt = 1; // set the flag to 1
  }
}

//----------------------------------------------------
// Sleep function. This shuts down all the peripherals and 
// send the cpu to sleep. In this configuration it is only
// awakened when the watchdog timer interrupt fires, causing 
// it to wake again. 
void goToSleep(void){
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

