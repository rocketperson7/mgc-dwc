#include "Wire.h"
#include "Adafruit_DRV2605.h"
#include <Adafruit_VL53L1X.h>

Adafruit_DRV2605 drv;

Adafruit_VL53L1X vl53; // Create an instance of the VL53L1CXV0FY1 sensor object


void i2c_select(uint8_t mux, uint8_t port) {
  if (port > 3) return;

  // Serial.print("I2C SELECT: ");
  // Serial.print("mux: ");
  // Serial.print(mux);
  // Serial.print(" port: ");
  // Serial.println(port);


  Wire.beginTransmission(0x71 - mux); //disable the other mux
  Wire.write(0);
  Wire.endTransmission();

  Wire.beginTransmission(0x70 + mux);
  Wire.write(4 + port);
  Wire.endTransmission();
  



}





void setup() {
  while (!Serial);
  delay(1000);
  Serial.begin(115200);
  Serial.println("\ndwc setup");
    
    
  // Wire.begin(5,4);
  Wire.begin(4,5);


  // for (uint8_t m=0; m<8; m++) {
  //   for (uint8_t t=0; t<4; t++) {
  //     i2c_select(m, t);
  //     Serial.print("mux at addr 0x7");
  //     Serial.print(m);
  //     Serial.print(" Port #"); 
  //     Serial.println(t);

  //     for (uint8_t addr = 0; addr<=127; addr++) {
  //       if (addr >= 0x70 && addr <= 0x78) continue;

  //       Wire.beginTransmission(addr);
  //       if (Wire.endTransmission() == 0) {
  //         Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
  //       }
  //     }
  //   }
  // }



  // motor driver section:
  for (uint8_t m=0; m<2; m++) {
    for (uint8_t p=0; p<3; p++) {
      if (m == 1 and p == 2) continue; //skip channel 6

      i2c_select(m, p); //motor driver

      Serial.println("Adafruit DRV2605 Basic test");
      if (! drv.begin()) {
        Serial.print("Could not find DRV2605 on mux ");
        Serial.print(m);
        Serial.print(" port ");
        Serial.println(p);
        // while (1) delay(10);
      }

      // drv.selectLibrary(6);
      // drv.useLRA();

      drv.useERM();
      // drv.writeRegister8(DRV2605_REG_CONTROL3, drv.readRegister8(DRV2605_REG_CONTROL3) | 0x00);

      Serial.println("control3 reg");
      Serial.print(drv.readRegister8(DRV2605_REG_CONTROL3),HEX);
      Serial.println(" ");

      // // I2C trigger by sending 'go' command 
      // // default, internal trigger when sending GO command
      // drv.setMode(DRV2605_MODE_INTTRIG); 
      drv.setMode(DRV2605_MODE_REALTIME);

      drv.setRealtimeValue(0);

    }
  }

  // while(1) delay(10000); // infinite loop without crashing



  //lidar sensor section:
  for (uint8_t m=0; m<2; m++) {
    for (uint8_t p=0; p<3; p++) {
      if (m == 1 and p == 2) continue; //skip channel 6
      i2c_select(m, p); 

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
  }




    Serial.println("\nsetup done");

}



uint8_t motor_out = 0;

void loop() {
   // while(1) delay(10000);

  int16_t distance;

  for (uint8_t m=0; m<2; m++) {
    for (uint8_t p=0; p<3; p++) {
      if (m == 1 and p == 2) continue; //skip channel 6

      i2c_select(m, p);
      
      if (vl53.dataReady()) {
        // new measurement for the taking!
        distance = vl53.distance();
        if (distance == -1) {
          // something went wrong!
          Serial.print(F("Couldn't get distance: "));
          Serial.println(vl53.vl_status);

          drv.setRealtimeValue(0);

          continue;
        }
        Serial.print("lidar number ");
        Serial.print(3*m + p);
        Serial.print(F(" distance: "));
        Serial.print(distance);
        Serial.println(" mm");

        // data is read out, time for another reading!
        vl53.clearInterrupt();



        // motor_out = map(distance, 0, 1000, 127,0);

      
        motor_out = 127 - 64* log(distance / 20);



        Serial.print("motor power:");
        Serial.println(motor_out);
        drv.setRealtimeValue(motor_out);

        delay(50);


        // i2c_select(m, 1); //lidar
      }
    }
  }
}
