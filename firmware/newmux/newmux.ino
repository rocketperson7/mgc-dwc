#include "Wire.h"
#include <Adafruit_VL53L1X.h>




int16_t distance = 0;

uint8_t port = 0;


Adafruit_VL53L1X vl53; // Create an instance of the VL53L1CXV0FY1 sensor object

#define TCAADDR 0x70

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}




void setup() {
  while (!Serial);
  delay(1000);
  Serial.begin(115200);
  Serial.println("\ndwc setup");
    
    
  // Wire.begin(5,4);
  Wire.begin(4,5);



  // lidar sensor section:
  for (port=0; port<5; port++) {
      if (port == 1 || port == 3) continue; //skip empty
      tcaselect(port);


      Serial.println(F("Adafruit VL53L1X sensor demo"));

      if (! vl53.begin(0x29, &Wire)) {
        Serial.print(F("Error on init of VL sensor: "));
        Serial.println(vl53.vl_status);
        continue;
      }
      Serial.println(F("VL53L1X sensor OK!"));

      Serial.print(F("Sensor ID: 0x"));
      Serial.println(vl53.sensorID(), HEX);

      if (! vl53.startRanging()) {
        Serial.print(F("Couldn't start ranging: "));
        Serial.println(vl53.vl_status);
        continue;
      }
      Serial.println(F("Ranging started"));

      // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
      vl53.setTimingBudget(50);
      Serial.print(F("Timing budget (ms): "));
      Serial.println(vl53.getTimingBudget());

      /*
      vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
      vl.VL53L1X_SetInterruptPolarity(0);
      */
    }

    Serial.println("\nsetup done");
}






void loop() {

  delay(50); // delay between updating all sensors


  for (port=0; port<5; port++) {
      if (port == 1 || port == 3) continue; //skip empty channels
      tcaselect(port);
      
      if (vl53.dataReady()) {
        // new measurement for the taking!
        distance = vl53.distance();
        if (distance == -1) {
          // something went wrong!
          Serial.print(F("Couldn't get distance on port: "));
          Serial.print(port);
          Serial.print(" status: ");
          Serial.println(vl53.vl_status);


          continue;
        }
        Serial.print("lidar number ");
        Serial.print(port);
        Serial.print(F(" distance: "));
        Serial.print(distance);
        Serial.println(" mm");

        // data is read out, time for another reading!
        vl53.clearInterrupt();




        // delay(50); //delay between each sensor update
      } 
      else {
        Serial.print("lidar not ready: ");
        Serial.println(port);
      }
    }
}
