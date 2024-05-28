#include "Wire.h"
#include "Adafruit_DRV2605.h"
#include <Adafruit_VL53L1X.h>



#define VOL_PIN A0
#define MODE_PIN 16


#define MOTOR_LIN 1
#define MOTOR_LOG 2
#define MOTOR_MIX 3

uint8_t motor_out_lin = 0;
uint8_t motor_out_log = 0;
uint8_t motor_out = 0;



uint8_t motor_mode = MOTOR_LIN;

uint8_t button = 1;
uint8_t button_last = 1;


uint16_t volume = 1024;
uint16_t volume_last = 1024;

int16_t distance = 0;


uint8_t mux = 0; 
uint8_t port = 0;


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


  //volume and mode pins
  pinMode(MODE_PIN, INPUT);




  // motor driver section:
  for (mux=0; mux<2; mux++) {
    for (port=0; port<3; port++) {
      if (mux == 1 and port == 2) continue; //skip channel 6
      i2c_select(mux, port);


      Serial.println("Adafruit DRV2605 Basic test");
      if (! drv.begin()) {
        Serial.print("Could not find DRV2605 on mux ");
        Serial.print(mux);
        Serial.print(" port ");
        Serial.println(port);
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



  // lidar sensor section:
  for (mux=0; mux<2; mux++) {
    for (port=0; port<3; port++) {
      if (mux == 1 and port == 2) continue; //skip channel 6
      i2c_select(mux, port); 


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






void loop() {
   // while(1) delay(10000);

  volume = analogRead(VOL_PIN);
  if (volume != volume_last){
    Serial.print(F("new volume: "));
    Serial.println(volume);
    volume_last = volume;

  }

  button = digitalRead(MODE_PIN);
  if(button != button_last){
    if(button == 0) {
      motor_mode++;
      if(motor_mode > MOTOR_MIX) motor_mode = MOTOR_LIN;
      Serial.print(F("new motor mode: "));
      Serial.println(motor_mode);

    } 
    button_last = button;

  }

  delay(50); // delay between updating all sensors




  for (mux=0; mux<2; mux++) {
    for (port=0; port<3; port++) {
      if (mux == 1 and port == 2) continue; //skip channel 6
      i2c_select(mux, port);
      
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
        Serial.print(3*mux + port);
        Serial.print(F(" distance: "));
        Serial.print(distance);
        Serial.println(" mm");

        // data is read out, time for another reading!
        vl53.clearInterrupt();



        // motor_out = map(distance, 0, 1000, 127,0);

      
        // motor_out = 127 - 64* log(distance / 20);
        motor_out_lin = 256*(2000-distance)/2000;

        motor_out_log = 256-77*log(distance);

        switch (motor_mode) {
          case MOTOR_LIN:
            motor_out = motor_out_lin;
            break;
          case MOTOR_LOG:
            motor_out = motor_out_log;
            break;
          case MOTOR_MIX:
            motor_out = 0.5*motor_out_lin + 0.5*motor_out_log;
            break;
          default:
            motor_out = 0;
        }
      
        
        motor_out = volume / 1023.0 * motor_out;



        Serial.print("motor power lin: ");
        Serial.print(motor_out_lin);
        Serial.print(", motor power log: ");
        Serial.print(motor_out_log);
        Serial.print(", motor out: ");
        Serial.println(motor_out);
        drv.setRealtimeValue(motor_out);

        // delay(50); //delay between each sensor update
      }
    }
  }
}
