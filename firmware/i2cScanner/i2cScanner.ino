// SPDX-FileCopyrightText: 2023 Liz Clark for Adafruit Industries
//
// SPDX-License-Identifier: MIT
/**
 * PCA9546 I2CScanner.ino -- I2C bus scanner for Arduino
 *
 * Based on https://playground.arduino.cc/Main/I2cScanner/
 *
 */

#include "Wire.h"


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




// standard Arduino setup()
void setup()
{
    while (!Serial);
    delay(1000);
    Serial.begin(115200);
    Serial.println("\nPCAScanner ready!");
    
    
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
    Serial.println("\ndone");
}

void loop() 
{
}
