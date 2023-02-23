#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <cmath>

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10

void loop4events();

// software SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

float lastX = 0;
float lastY = 0;
float lastZ = 0;
bool on = false;
gpio_num_t relayGPIO = GPIO_NUM_26;


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
  
}

void loop(){
  loop4events();
}

void loop4events() {
  lis.read();      // get X Y and Z data at once

  /* Or....get a new sensor event, normalized */
  sensors_event_t event;
  lis.getEvent(&event);

  float threshold = .5;

  if(lastX > 0 || lastY > 0 || lastZ > 0)
  {
    if(abs(lastX - event.acceleration.x) > threshold || abs(lastY - event.acceleration.y) > threshold)
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
        char outString[200];
        
        sprintf(outString,
        "\t\tX: %f to %f (%f) Y: %f to %f (%f) Z: %f to %f (%f)",
          lastX, event.acceleration.x, abs(lastX - event.acceleration.x),
          lastY, event.acceleration.y, abs(lastY - event.acceleration.y),
          lastZ, event.acceleration.z, abs(lastZ - event.acceleration.z));
        Serial.println(outString);
    
        Serial.println("Setting high");
        gpio_set_level(relayGPIO, 1);
        delay(1000);
        gpio_set_level(relayGPIO, 0);
        Serial.println("Setting low");
        //delay(1000);
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

  delay(100);
}
