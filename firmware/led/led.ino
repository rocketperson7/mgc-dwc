#include <Wire.h>
#include <Adafruit_VL53L1X.h>

#define TCA_ADDR 0x73 // I2C address of TCA9544APWR
#define LED_ADDR 0x68 // I2C address of the IS31FL3193-DLS2-TR
ddd
s
void setup() {
  while (!Serial);
  delay(1000);
  Serial.begin(115200);
	
  Wire.begin(5,4); // Initialize I2C communication
	Wire.beginTransmission(TCA_ADDR);
	Wire.write(4 + 0x01); // Select bus 1 (connected to the IS31FL3193-DLS2-TR)
	Wire.endTransmission();
	// vl53l1.begin(); // Initialize the VL53L1CXV0FY1 sensor
}

void loop() {
	// uint16_t distance = vl53l1.read(); // Read the distance from the sensor
	// byte dutyCycle = map(distance, 0, 1000, 0, 255); // Map the distance to a duty cycle value (0-255)
	setLED(1, 255); // Set the duty cycle of the first LED channel to the mapped value
	delay(50); // Wait for 50ms before reading again
}

// Set the duty cycle of a single LED channel
void setLED(int channel, byte dutyCycle) {
	Wire.beginTransmission(LED_ADDR);
	Wire.write(channel);
	Wire.write(dutyCycle);
	Wire.endTransmission();
}