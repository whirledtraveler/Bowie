/* Based on code from  http://gammon.com.au/power
   This script puts the Arduino to sleep, and then uses the SQW squarewave 
   output from a DS3231 Chronodot to wake it again. It lights the LED
   while awake, then goes back to sleep. The DS3231 SQW output is set to
   trigger once per second (1 Hz). Connect the SQW output from the clock
   to pin D2 on the Arduino. We use the internal AVR pull-up resistor on
   that line.
   LPM March 2014
*/
#include <avr/sleep.h>
#include <Wire.h>
//#include <SPI.h>
#include <RTClib.h>
#include <RTC_DS3231.h>

RTC_DS3231 RTC; // name the real time clock object RTC

#define SQW_FREQ DS3231_SQW_FREQ_1     //0b00001000   1Hz 
//#define RTC_SQW_IN 2     // input square wave from RTC into T0 pin (D5)

const byte LED = 9; // Specify which pin the LED is attached to

void setup () 
  {
  Wire.begin();
  RTC.begin();
  RTC.SQWEnable(true); // turn on the square wave
  RTC.BBSQWEnable(true); // enable the battery-backed SQW output
  RTC.SQWFrequency( SQW_FREQ ); // Set the SQW frequency
  
  digitalWrite (2, HIGH);  // enable pull-up on pin D2 (interrupt)
  }  // end of setup


void loop () 
{  
   // If pin D2 is currently low, turn on the LED
   // This will be true when coming back from sleep, since
   // the low signal triggers the wake up and remains low 
   // for however long the SQW interval is set to. 
   while (digitalRead(2) == LOW){
    pinMode (LED, OUTPUT);
    digitalWrite (LED, HIGH);
   }
   // Now that pin D2 is high again, turn off LED and
   // continue with loop
   digitalWrite (LED, LOW);
   pinMode (LED, INPUT); // for power savings
   
  //-----------------------------------------------------------
  // Entering sleep mode section. Don't change much in here
  // without knowing what you're up to.
  ADCSRA = 0;   // disable ADC analog digital convertor
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  // specify sleep mode
  sleep_enable();
  // Do not interrupt before we go to sleep, or the
  // ISR will detach interrupts and we won't wake.
  noInterrupts ();
  // Set interrupt on pin D2, called when D2 goes low
  attachInterrupt (0, wake, LOW);
  // turn off brown-out enable in software
  // BODS must be set to one and BODSE must be set to zero within four clock cycles
  MCUCR = bit (BODS) | bit (BODSE);
  // The BODS bit is automatically cleared after three clock cycles
  MCUCR = bit (BODS); 
  // We are guaranteed that the sleep_cpu call will be done
  // as the processor executes the next instruction after
  // interrupts are turned on.
  interrupts ();  // one cycle, re-enables interrupts
  sleep_cpu ();   // one cycle, going to sleep now, wake on interrupt
  // The AVR is now asleep. In SLEEP_MODE_PWR_DOWN it can only be woken
  // using the interrupt we set on pin D2 (interrupt 0) by pulling that
  // pin low (it should normally be pulled high). At that point, the
  // wake function will be called, sleep will be disabled, and the
  // interrupt on pin D2 will be disabled. You then re-enter the
  // program below this section
  //-------------------------------------------------------------------

} // end of loop

//--------------------------------- 
// The wake() function wakes the processor and removes the 
// interrupt on pin D2 (interrupt 0) so that the wake function
// isn't called more than once per sleep cycle. This also 
// allows pin D2 to do other stuff, including monitor other
// interrupts while the processor is awake. 
void wake ()
{
  // cancel sleep as a precaution
  sleep_disable();
  // must do this as the pin will probably stay low for a while
  detachInterrupt (0);
}  // end of wake
//----------------------------------
