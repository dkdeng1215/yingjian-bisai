#include <Wire.h>
#include <Adafruit_VL53L1X.h>

#include "config.h"

bool selectTofChannel(uint8_t channel){
  if(channel>7){
    return false;
  }
  Wire.beginTransmission(TCA9548A_I2C_ADDRESS);
  Wire.write(1<<channel);
  return (Wire.endTransmission()==0);
}//检查是否超出多路复用器的通道数

bool checkI2CAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}//一个检查地址的小函数

void checkTofModules(){
  Serial.println(F("check I2C devices"));
  if(checkI2CAddress(TCA9548A_I2C_ADDRESS)){
    Serial.println(F("TCA9548A 0X70:OK"));
  }
  else {
  Serial.println(F("TCA9548A OX70:FAILED"));
  return;
  }
  uint8_t channels[3] = {
    TOF_LEFT_CHANNEL,
    TOF_CENTER_CHANNEL,
    TOF_RIGHT_CHANNEL
  };

  const char *names[3] = {
    "LEFT",
    "CENTER",
    "RIGHT"
  };

  uint8_t foundCount = 0;

  for (uint8_t i = 0; i < 3; i++) {
    if (!selectTofChannel(channels[i])) {
      Serial.print(names[i]);
      Serial.print(F(" CH"));
      Serial.print(channels[i]);
      Serial.println(F(": TCA select failed"));
      continue;
    }

    if (checkI2CAddress(VL53L1X_I2C_ADDRESS)) {
      Serial.print(names[i]);
      Serial.print(F(" CH"));
      Serial.print(channels[i]);
      Serial.println(F(" 0x29: OK"));
      foundCount++;
    } else {
      Serial.print(names[i]);
      Serial.print(F(" CH"));
      Serial.print(channels[i]);
      Serial.println(F(" 0x29: MISSING"));
    }
  }

  Serial.print(F("ToF modules found: "));
  Serial.print(foundCount);
  Serial.print('/');
  Serial.println(3);
}

Adafruit_VL53L1X tof;

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial to initialize

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);

  Serial.println("Checking TCA9548A and VL53L1X modules...");
  checkTofModules();

  tof.VL53L1X_SetDistanceMode(2);
  tof.setTimingBudget(50);
  tof.startRanging();

  Serial.println("VL531X started");
}

void loop() {
}
