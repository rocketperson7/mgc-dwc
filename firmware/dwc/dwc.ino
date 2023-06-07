#include "Wire.h"
#include "Adafruit_DRV2605.h"
#include <Adafruit_VL53L1X.h>

Adafruit_DRV2605 drv;

Adafruit_VL53L1X vl53; // Create an instance of the VL53L1CXV0FY1 sensor object


void i2c_select(uint8_t mux, uint8_t port) {
  if (port > 3) return;
 
  for (uint8_t m=0; m<7; m++) {
    // if(m==mux) continue; //don't need to disable the current mux 
    Wire.beginTransmission(0x70 + m); //disable the other muxes
    Wire.write(0);
    Wire.endTransmission();
  }

  Wire.beginTransmission(0x70 + mux);
  Wire.write(4 + port);
  Wire.endTransmission();
  



}

void LED_write_byte(uint8_t Reg_Add,uint8_t Reg_Dat)
{
  Wire.beginTransmission(0x68); // transmit to device IS31FL373x
  Wire.write(Reg_Add); // sends regaddress
  Wire.write(Reg_Dat); // sends regaddress
  Wire.endTransmission(); // stop transmitting
}

void IS31FL3193_Breath_mode(void)//
{
  LED_write_byte(0x00,0x20);//normal operation
  LED_write_byte(0x02,0x20);//one shot programming mode
  LED_write_byte(0x03,0x00);//Imax=42mA
  LED_write_byte(0x04,0xFF);//PWM
  LED_write_byte(0x05,0xFF);//PWM
  LED_write_byte(0x06,0xFF);//PWM
  LED_write_byte(0x07,0x00);//update
  LED_write_byte(0x0A,0x00);//OUT1 T0=0
  LED_write_byte(0x0B,0x10);//OUT2 T0=0.13s
  LED_write_byte(0x0C,0x20);//OUT3 T0=0.26s
  LED_write_byte(0x10,0x46);//OUT1 T1=1.04S
  LED_write_byte(0x11,0x46);//OUT2 T1=1.04S
  LED_write_byte(0x12,0x46);//OUT3 T1=1.04S
  LED_write_byte(0x16,0x46);//OUT1 T3=1.04S
  LED_write_byte(0x17,0x46);//OUT2 T3=1.04S
  LED_write_byte(0x18,0x46);//OUT3 T3=1.04S
  LED_write_byte(0x1D,0x07);    //on LED  
  LED_write_byte(0x1C,0x00);//update
}

// LED_set()


void setup() {
  while (!Serial);
    delay(1000);
    Serial.begin(115200);
    Serial.println("\ndwc setup");
    
    
    // Wire.begin(5,4);
    Wire.begin(4,5);


  for (uint8_t m=0; m<8; m++) {
    for (uint8_t t=0; t<4; t++) {
      i2c_select(m, t);
      Serial.print("mux at addr 0x7");
      Serial.print(m);
      Serial.print(" Port #"); 
      Serial.println(t);

      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr >= 0x70 && addr <= 0x78) continue;

        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
          Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }
      }
    }
  }



          // motor driver section:
    for (uint8_t m=0; m<8; m++) {
      if (m != 2 and m !=5) continue;

      i2c_select(m, 0); //motor driver

      Serial.println("Adafruit DRV2605 Basic test");
      if (! drv.begin()) {
        Serial.print("Could not find DRV2605 on mux ");
        Serial.print(m);
        Serial.println(" port 0");
        // while (1) delay(10);
      }

      // drv.selectLibrary(6);
      // drv.useLRA();

      drv.useERM();
      // drv.writeRegister8(DRV2605_REG_CONTROL3, drv.readRegister8(DRV2605_REG_CONTROL3) | 0x00);

      Serial.println("control3 reg");
      Serial.print(drv.readRegister8(DRV2605_REG_CONTROL3),HEX);

      // // I2C trigger by sending 'go' command 
      // // default, internal trigger when sending GO command
      // drv.setMode(DRV2605_MODE_INTTRIG); 
      drv.setMode(DRV2605_MODE_REALTIME);

      drv.setRealtimeValue(50);


    //   i2c_select(mux_num, 3); //motor driver

    //   Serial.println("Adafruit DRV2605 Basic test");
    //   if (! drv.begin()) {
    //     Serial.print("Could not find DRV2605 on mux ");
    //     Serial.print(m);
    //     Serial.println(" port 3");
    //     // while (1) delay(10);
    //   }

    //   // drv.selectLibrary(6);
    //   // drv.useLRA();

    //   drv.useERM();
    //   // drv.writeRegister8(DRV2605_REG_CONTROL3, drv.readRegister8(DRV2605_REG_CONTROL3) | 0x00);

    //   Serial.println("control3 reg");
    //   Serial.println(drv.readRegister8(DRV2605_REG_CONTROL3),HEX);

    //   // // I2C trigger by sending 'go' command 
    //   // // default, internal trigger when sending GO command
    //   // drv.setMode(DRV2605_MODE_INTTRIG); 
    //   drv.setMode(DRV2605_MODE_REALTIME);

    }

  // while(1) delay(10000); // infinite loop without crashing



    //lidar sensor section:
    for (uint8_t m=0; m<8; m++) {
      if (m != 2 and m != 5) continue;


      i2c_select(m, 1); 
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

    //led driver section
    // for (uint8_t m=0; m<8; m++) {
    //   i2c_select(m, 2); 
    //   // IS31FL3193_Breath_mode();
    // }




    Serial.println("\nsetup done");

}

// void loop() {
  // put your main code here, to run repeatedly: (for each i2c port?)

  // read lidars

  // do some math

  // send intensity to motors

  // put something on the LED?





uint8_t motor_out = 0;

void loop() {
//  Serial.print("motor_out #"); Serial.println(motor_out);


  // set the motor_out to play
  // drv.setWaveform(0, motor_out);  // play motor_out 
  // drv.setWaveform(1, 0);       // end waveform

  // i2c_select(3, 0); //motor driver
  // drv.setRealtimeValue(motor_out);

  // play the motor_out!
  // drv.go();

  // wait a bit
  // delay(50);

  // motor_out+=1;
  // if (motor_out > 127) motor_out = 1;

 

  int16_t distance;

  for (uint8_t m=0; m<8; m++) {
      if (m != 2 and m != 5) continue;
      i2c_select(m, 1);
      if (vl53.dataReady()) {
        // new measurement for the taking!
        distance = vl53.distance();
        if (distance == -1) {
          // something went wrong!
          // Serial.print(F("Couldn't get distance: "));
          // Serial.println(vl53.vl_status);
          i2c_select(m, 0); //motor driver
          drv.setRealtimeValue(0);
          i2c_select(m, 1); //lidar
          continue;
        }
      Serial.print("lidar number ");
      Serial.print(m);
      Serial.print(F(" distance: "));
      Serial.print(distance);
      Serial.println(" mm");

      // data is read out, time for another reading!
      vl53.clearInterrupt();


      i2c_select(m, 0); //motor driver
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
