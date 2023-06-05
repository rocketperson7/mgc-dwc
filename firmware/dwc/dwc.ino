#include "Wire.h"
#include "Adafruit_DRV2605.h"
#include <Adafruit_VL53L1X.h>

Adafruit_DRV2605 drv;

Adafruit_VL53L1X vl53; // Create an instance of the VL53L1CXV0FY1 sensor object

uint8_t mux_num = 6;


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
    
    
    Wire.begin(5,4);


  for (uint8_t m=0; m<8; m++) {
    for (uint8_t t=0; t<4; t++) {
      i2c_select(m, t);
      Serial.print("mux at addr 0x7");
      Serial.print(m);
      Serial.print(" Port #"); Serial.println(t);

      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr >= 0x70 && addr <= 0x78) continue;
        // if (addr == 0x73) continue;

        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
          Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }
      }
    }
  }
    // for (uint8_t t=0; t<4; t++) {
    //   i2c_select(3, t);
    //   Serial.print("PCA 3 Port #"); Serial.println(t);

    //   for (uint8_t addr = 0; addr<=127; addr++) {
    //     // if (addr == 0x70) continue;
    //     // if (addr == 0x73) continue;

    //     Wire.beginTransmission(addr);
    //     if (Wire.endTransmission() == 0) {
    //       Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
    //     }
    //   }
    // }

  while(1){
    delay(10000);
  };

    // motor driver section:
    i2c_select(mux_num, 0); //motor driver

    Serial.println("Adafruit DRV2605 Basic test");
    if (! drv.begin()) {
      Serial.println("Could not find DRV2605");
      while (1) delay(10);
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

    //lidar sensor section:
    i2c_select(mux_num, 1); 
    Serial.println(F("Adafruit VL53L1X sensor demo"));

    if (! vl53.begin(0x29, &Wire)) {
      Serial.print(F("Error on init of VL sensor: "));
      Serial.println(vl53.vl_status);
      while (1)       delay(10);
    }
    Serial.println(F("VL53L1X sensor OK!"));

    Serial.print(F("Sensor ID: 0x"));
    Serial.println(vl53.sensorID(), HEX);

    if (! vl53.startRanging()) {
      Serial.print(F("Couldn't start ranging: "));
      Serial.println(vl53.vl_status);
      while (1)       delay(10);
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


    //led driver section
    // i2c_select(3, 2); 
    // IS31FL3193_Breath_mode();
    // Wire.beginTransmission(0x68);
    // Wire.write(0); //set led driver register address
    // Wire.write(0); //disable software shutdown?
    // Wire.endTransmission();

    // Wire.beginTransmission(0x68);
    // Wire.write(0x1D); //set led driver register address
    // Wire.write(0x03); 
    // Wire.endTransmission();




    // Wire.write(0); //breathign control
    // Wire.write(1<<5); //led mode
    // Wire.write(0); //current setting

    // Wire.write(255); //red channel?
    // Wire.write(200); //green channel?
    // Wire.write(155); //blue channel?

    // Wire.write(0); //latch values to output

    // Wire.write(1<<5); // enable all channels?

    // if(Wire.endTransmission()){
    //   Serial.println("fail");
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

  if (vl53.dataReady()) {
    // new measurement for the taking!
    distance = vl53.distance();
    if (distance == -1) {
      // something went wrong!
      // Serial.print(F("Couldn't get distance: "));
      // Serial.println(vl53.vl_status);
      i2c_select(3, 0); //motor driver
      drv.setRealtimeValue(0);
      i2c_select(3, 1); //lidar

      return;
    }
    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(" mm");

    // data is read out, time for another reading!
    vl53.clearInterrupt();


    i2c_select(mux_num, 0); //motor driver
    // motor_out = map(distance, 0, 1000, 127,0);

    
    motor_out = 127 - 64* log(distance / 20);



    // if (distance > 0 and distance < 20){
    //   motor_out = 127;
    // } else if (distance >= 20 and distance < 200){
    //   motor_out = 96;
    // } else if (distance >= 200 and distance < 300){
    //   motor_out = 64;
    // } else if (distance >= 300 and distance < 400){
    //   motor_out = 32;
    // } else if (distance >= 400 and distance < 500){
    //   motor_out = 16;
    // } else if (distance >= 500){
    //   motor_out = 8;
    // }

    Serial.print("motor power:");
    Serial.println(motor_out);
    drv.setRealtimeValue(motor_out);




    i2c_select(mux_num, 1); //lidar

  }
}
