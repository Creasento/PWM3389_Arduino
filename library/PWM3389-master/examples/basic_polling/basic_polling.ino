#include <PMW3389.h>

/* 

# PIN CONNECTION
  * MI = MISO
  * MO = MOSI
  * SS = Slave Select / Chip Select
  * SC = SPI Clock
  * MT = Motion (active low interrupt line)
  * RS = Reset
  * GD = Ground
  * VI = Voltage in up to +5.5V 

Module   Arduino
  RS --- (NONE)
  GD --- GND
  MT --- (NONE)
  SS --- Pin_10   (use this pin to initialize a PMW3389 instance)
  SC --- SCK 
  MO --- MOSI
  MI --- MISO
  VI --- 5V

# PMW3389_DATA struct format and description
  - PMW3389_DATA.isMotion      : bool, True if a motion is detected. 
  - PMW3389_DATA.isOnSurface   : bool, True when a chip is on a surface 
  - PMW3389_DATA.dx, data.dy   : integer, displacement on x/y directions.
  - PMW3389_DATA.SQUAL         : byte, Surface Quality register, max 0x80
                               * Number of features on the surface = SQUAL * 8
  - PMW3389_DATA.rawDataSum    : byte, It reports the upper byte of an 18‐bit counter 
                               which sums all 1296 raw data in the current frame;
                               * Avg value = Raw_Data_Sum * 1024 / 1296
  - PMW3389_DATA.maxRawData    : byte, Max/Min raw data value in current frame, max=127
    PMW3389_DATA.minRawData
  - PMW3389_DATA.shutter       : unsigned int, shutter is adjusted to keep the average
                               raw data values within normal operating ranges.
 */

#define SS  10   // Slave Select pin. Connect this to SS on the module.

PMW3389 sensor;

void setup() {
  Serial.begin(9600);  
  while(!Serial);
  
  //sensor.begin(10, 1600); // to set CPI (Count per Inch), pass it as the second parameter
  if(sensor.begin(SS))  // 10 is the pin connected to SS of the module.
    Serial.println("Sensor initialization successed");
  else
    Serial.println("Sensor initialization failed");
  
  //sensor.setCPI(1600);    // or, you can set CPI later by calling setCPI();
}

void loop() {
  PMW3389_DATA data = sensor.readBurst();
  
  if(data.isOnSurface && data.isMotion)
  {
    Serial.print(data.dx);
    Serial.print("\t");
    Serial.print(data.dy);
    Serial.println();
  }
  
  delay(10);
}
