#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Wire.h>
#include <SPI.h>
#include <RTClib.h>
#include <RTC_DS3231.h>


/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground
    
*/
   
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

RTC_DS3231 RTC;

// easier to reference here...see .h file for more options
//#define SQW_FREQ DS3231_SQW_FREQ_1      //  0b00000000  1Hz
#define SQW_FREQ DS3231_SQW_FREQ_1024     //0b00001000   1024Hz
//#define SQW_FREQ DS3231_SQW_FREQ_4096  // 0b00010000   4096Hz
//#define SQW_FREQ DS3231_SQW_FREQ_8192 //0b00011000      8192Hz

#define PWM_COUNT 1020   //determines how often the LED flips
#define LOOP_DELAY 5000 //ms delay time in loop

#define RTC_SQW_IN 5     // input square wave from RTC into T1 pin (D5)
                               //WE USE TIMER1 so that it does not interfere with Arduino delay() command
#define INT0_PIN   2     // INT0 pin for 32kHz testing?
#define LED_PIN    9     // random LED for testing...tie to ground through series resistor..
#define LED_ONBOARD 13   // Instead of hooking up an LED, use LED at pin 13.

volatile long TOGGLE_COUNT = 0;

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/

void displaySensorDetails(void)
{
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(2500);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
  Serial.begin(9600);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  pinMode(RTC_SQW_IN, INPUT);
  pinMode(INT0_PIN, INPUT);
  
    //--------RTC SETUP ------------
    Wire.begin();
    RTC.begin();

    if (! RTC.isrunning()) {
      Serial.println("RTC is NOT running!");
      // following line sets the RTC to the date & time this sketch was compiled
      RTC.adjust(DateTime(__DATE__, __TIME__));
    }
  
    DateTime now = RTC.now();
    DateTime compiled = DateTime(__DATE__, __TIME__);
    if (now.unixtime() < compiled.unixtime()) {
      //Serial.println("RTC is older than compile time!  Updating");
      RTC.adjust(DateTime(__DATE__, __TIME__));
    }
    
    RTC.enable32kHz(true);
    RTC.SQWEnable(true);
    RTC.BBSQWEnable(true);
    RTC.SQWFrequency( SQW_FREQ );
  
    char datastr[100];
    RTC.getControlRegisterData( datastr[0]  );
    Serial.print(  datastr );
 
  
  
    //--------INT 0---------------
    EICRA = 0;      //clear it
    EICRA |= (1 << ISC01);
    EICRA |= (1 << ISC00);   //ISC0[1:0] = 0b11  rising edge INT0 creates interrupt
    EIMSK |= (1 << INT0);    //enable INT0 interrupt
    
    
  Serial.println("Pressure Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) 
{
  
   Serial.print("Toggle Count over ");
    Serial.print(LOOP_DELAY, DEC);
    Serial.print("ms with PWM_COUNT of ");
    Serial.print(PWM_COUNT, DEC);
    Serial.print(":  ");
    Serial.print(TOGGLE_COUNT, DEC);
    Serial.println();
    TOGGLE_COUNT = 0;
  
    DateTime now = RTC.now();
    
    RTC.forceTempConv(true);  //DS3231 does this every 64 seconds, we are simply testing the function here
    float temp_float = RTC.getTempAsFloat();
    int16_t temp_word = RTC.getTempAsWord();
    int8_t temp_hbyte = temp_word >> 8;
    int8_t temp_lbyte = temp_word &= 0x00FF;
    
    
    
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
    Serial.println();
    Serial.print("Chronodot Temp: ");
    Serial.print(temp_hbyte, DEC);
    Serial.print(".");
    Serial.print(temp_lbyte, DEC);
    Serial.println(" C");
    Serial.println();
    
  /* Get a new sensor event */ 
  sensors_event_t event;
  bmp.getEvent(&event);
 
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    /* Display atmospheric pressue in hPa */
    Serial.print("Pressure:    ");
    Serial.print(event.pressure);
    Serial.println(" hPa");
    
    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted, as well as the ambient temperature in degress           *
     * celcius.  If you don't have these values, a 'generic' value of     *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
     * in sensors.h), but this isn't ideal and will give variable         *
     * results from one day to the next.                                  *
     *                                                                    *
     * You can usually find the current SLP value by looking at weather   *
     * websites or from environmental information centers near any major  *
     * airport.                                                           *
     *                                                                    *
     * For example, for Paris, France you can check the current mean      *
     * pressure and sea level at: http://bit.ly/16Au8ol                   */
     
    /* First we get the current temperature from the BMP085 */
    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");

    /* Then convert the atmospheric pressure, SLP and temp to altitude    */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = 1029.1;
    Serial.print("Altitude:    "); 
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure,
                                        temperature)); 
    Serial.println(" m");
    Serial.println("");
  }
  else
  {
    Serial.println("Sensor error");
  }
  delay(2500);
}
