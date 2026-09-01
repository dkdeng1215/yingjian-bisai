#include <Wire.h>
#include <Adafruit_VL53L1X.h>

#include "config.h"

Adafruit_VL53L1X tof;

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial to initialize

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);

  Serial.println("Initializing VL53L1X...");
  if (!tof.begin(VL53L1X_I2C_ADDRESS, &Wire)) {
    Serial.println("Failed to find VL53L1X sensor!");
    while (1){
      delay(1000);
    };
  }
  Serial.println("VL53L1X is initialized successfully!");

  tof.VL53L1X_SetDistanceMode(2);
  tof.setTimingBudget(50);
  tof.startRanging();

  Serial.println("VL531X started");
}

void loop() {
  if(tof.dataReady()){
    uint16_t distance=tof.distance();
    tof.clearInterrupt();

  if (distance >= DISTANCE_MIN_MM && distance <= DISTANCE_MAX_MM) {
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" mm");
      } 
  else {
      Serial.println("Invalid distance");
    }
  }

}
