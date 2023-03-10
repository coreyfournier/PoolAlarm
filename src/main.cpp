#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include <cmath>

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10

void loop4events();
void advancedRead();
void displaySensorDetails();
void configureSensor();
float getLux();

// software SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)


float lastX = 0;
float lastY = 0;
float lastZ = 0;
bool on = false;
gpio_num_t relayGPIO = GPIO_NUM_26;
const float potentiometerMax = 4095;
const int highestDifference = 2;
float lastPotentiometerValue = 0;
//It must be greater than this before it's armed
float minLux = 20;


void setup(void) {
  Serial.println("Setup");
  Serial.begin(115200);

  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Setting up GPIO");
  gpio_set_direction(relayGPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(relayGPIO, 0);
  
  Serial.println("LIS3DH test!");


  if (! lis.begin(0x18)) 
  {   // change this to 0x19 for alternative i2c address
      Serial.println("Couldnt start");
      while (1) yield();
  }
  Serial.println("LIS3DH found!");

  // lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");

  // lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  Serial.print("Data rate set to: ");
  switch (lis.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;

    case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
    case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
    case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
  }  

  Serial.println(F("Starting Adafruit TSL2591 Test!"));
  
  if (tsl.begin()) 
  {
    Serial.println(F("Found a TSL2591 sensor"));
  } 
  else 
  {
    Serial.println(F("No sensor found ... check your wiring?"));
    while (1);
  }
    
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Configure the sensor */
  configureSensor();

}

void loop(){
  loop4events();
}

void loop4events() {

  int potentiometerValue = analogRead(A5);  
  float potentiometerPercent = (float)potentiometerValue / potentiometerMax;
  float diffThreshold = (potentiometerPercent * highestDifference);
  float lux = getLux();

  if(lastPotentiometerValue != potentiometerValue)
  {
    Serial.print("P%=");
    Serial.print(potentiometerPercent);
    Serial.print(" threshold="); 
    Serial.println(diffThreshold);
    lastPotentiometerValue = potentiometerValue;
  }

  lis.read();      // get X Y and Z data at once

  /* Or....get a new sensor event, normalized */
  sensors_event_t event;
  lis.getEvent(&event);
  
  if(lux <= minLux){
    Serial.printf("It's too dark, exiting lux %f < min %f", lux, minLux);
    delay(3000);
    return;
  }
    

  if(lastX > 0 || lastY > 0 || lastZ > 0)
  {
    if(abs(lastX - event.acceleration.x) > diffThreshold || abs(lastY - event.acceleration.y) > diffThreshold)
    {
      //Don't trigger on the way up
      if(on)
      {
        on = false;
        Serial.println("Going back to steady state");
      }
      else 
      {        
        on = true;
        Serial.println("Event occured");
        Serial.print("\t\tX: "); Serial.print(event.acceleration.x); Serial.print(" chg:"); Serial.print(abs(lastX - event.acceleration.x));
        Serial.print(" \tY: "); Serial.print(event.acceleration.y); Serial.print(" chg:"); Serial.print(abs(lastY - event.acceleration.y));
        Serial.print(" \tZ: "); Serial.print(event.acceleration.z);
        Serial.println(" m/s^2 ");
        Serial.println();

        Serial.println("Setting high");
        gpio_set_level(relayGPIO, 1);
        delay(1000);
        gpio_set_level(relayGPIO, 0);
        Serial.println("Setting low");
      }
    }
    else
    {
      //Serial.println("Didn't move enough "); 
    }  
  }
  
  lastX = event.acceleration.x;
  lastY = event.acceleration.y;
  lastZ = event.acceleration.z;

  delay(200);
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" lux"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" lux"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution, 4); Serial.println(F(" lux"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
}

void configureSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Gain:         "));
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println(F("1x (Low)"));
      break;
    case TSL2591_GAIN_MED:
      Serial.println(F("25x (Medium)"));
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println(F("428x (High)"));
      break;
    case TSL2591_GAIN_MAX:
      Serial.println(F("9876x (Max)"));
      break;
  }
  Serial.print  (F("Timing:       "));
  Serial.print((tsl.getTiming() + 1) * 100, DEC); 
  Serial.println(F(" ms"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
}


/**************************************************************************/
/*
    Show how to read IR and Full Spectrum at once and convert to lux
*/
/**************************************************************************/
void advancedRead(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("IR: ")); Serial.print(ir);  Serial.print(F("  "));
  Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  "));
  Serial.print(F("Visible: ")); Serial.print(full - ir); Serial.print(F("  "));
  Serial.print(F("Lux: ")); Serial.println(tsl.calculateLux(full, ir), 6);
}

float getLux()
{
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  return tsl.calculateLux(full, ir);
}