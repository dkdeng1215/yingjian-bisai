#include <Wire.h>
#include <Adafruit_VL53L1X.h>

#include "config.h"
#include "common.h"
#include "decision.h"

bool selectTofChannel(uint8_t channel){
  if(channel>7){
    return false;
  }
  Wire.beginTransmission(TCA9548A_I2C_ADDRESS);
  Wire.write(1<<channel);
  return (Wire.endTransmission()==0);
}//通道切换函数，选取不同通道的传感器进行通信避免冲突

bool checkI2CAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}//一个检查能否开启通信的小函数

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
  Serial.println(3);//挨个检查三个激光传感器是否在线
}//检查TCA和三个激光传感器是否在线的函数

bool initTofOnChannel(uint8_t channel, Adafruit_VL53L1X &tof) {
  Serial.print("Initializing ToF on CH");
  Serial.println(channel);

  if (!selectTofChannel(channel)) {
    Serial.println("TCA9548A select failed");
    return false;
  }

  if (!tof.begin(VL53L1X_I2C_ADDRESS, &Wire)) {
    Serial.println("VL53L1X begin failed");
    return false;
  }

  if (tof.VL53L1X_SetDistanceMode(2) != VL53L1X_ERROR_NONE) {
    Serial.println("VL53L1X SetDistanceMode failed");
    return false;
  }

  if (!tof.setTimingBudget(50)) {
    Serial.println("VL53L1X setTimingBudget failed");
    return false;
  }

  if (!tof.startRanging()) {
    Serial.println("VL53L1X startRanging failed");
    return false;
  }

  Serial.print("ToF CH");
  Serial.print(channel);
  Serial.println(" Started");
  return true;
}//初始化三个激光传感器的函数

struct SensorSlot {
    Adafruit_VL53L1X tof;        // 这个方向的传感器对象
    uint16_t samples[FILTER_WINDOW_SIZE];          // 最近5个有效样本（中值滤波用）
    uint8_t  writeIndex;                     // 下一个写到哪个位置（0~4循环）
    uint8_t  sampleCount;         // 当前有几个有效样本
    uint32_t lastValidMs;         // 最后一次有效读数时间
};

SensorSlot slots[3];  // slots[LEFT] / slots[CENTER] / slots[RIGHT]

bool readRawDistance(Direction dir,uint16_t &out){
    if(!selectTofChannel(dir))return false;
    if(!slots[dir].tof.dataReady())return false;
    out=slots[dir].tof.distance();
    return true;
  }//读取传感器原始数据的函数

bool isDistanceValid(uint16_t distance) {
    return (distance >= DISTANCE_MIN_MM && distance <= DISTANCE_MAX_MM);
}//检查数据是否是有效值

void dataPush(Direction dir,uint16_t distance,uint32_t nowMs){
  SensorSlot &s=slots[dir];
  s.samples[s.writeIndex]=distance;
  s.writeIndex=(s.writeIndex+1)%FILTER_WINDOW_SIZE;
  if(s.sampleCount<FILTER_WINDOW_SIZE){
    s.sampleCount++;
  }
  s.lastValidMs=nowMs;
}//把原始数据推入对应对象的缓冲区

uint16_t mediateData(Direction dir){
  SensorSlot &s=slots[dir];
  if(s.sampleCount==0)return 0;

  uint16_t temp[FILTER_WINDOW_SIZE];
  for(int i=0;i<s.sampleCount;i++){
    // 环形缓冲区里找第i个有效样本
        uint8_t idx = (s.writeIndex + FILTER_WINDOW_SIZE - s.sampleCount + i) % FILTER_WINDOW_SIZE;
        temp[i] = s.samples[idx];
  }
  // 冒泡排序（只有5个元素，随便排）
    for (uint8_t i = 0; i < s.sampleCount - 1; i++) {
        for (uint8_t j = 0; j < s.sampleCount - 1 - i; j++) {
            if (temp[j] > temp[j + 1]) {
                uint16_t t = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = t;
            }
        }
    }

  return temp[s.sampleCount/2];
}//中值滤波，处理原始数据，输出处理后的数据

SystemState getSystemState(){
  SystemState state;
  for(int i=0;i<3;i++){
    state.zones[i].distanceMm=mediateData(i);
    state.zones[i].valid=(millis()-slots[i].lastValidMs<SENSOR_STALE_MS);
  }
  state.anySensorInvalid = !state.zones[0].valid || !state.zones[1].valid || !state.zones[2].valid;
  state.allSensorsInvalid = !state.zones[0].valid && !state.zones[1].valid && !state.zones[2].valid;

  return state;
}//这是决策部分的入口，拿这个返回的state做决策

void Schedule(uint32_t nowMs){
  static Direction dir=LEFT;
  static uint32_t lastPollMs=0;//设置静态变量，每次可以继续用

  if(nowMs-lastPollMs<TOF_POLL_INTERVAL_MS){
    return;
  }
  lastPollMs=nowMs;

  uint16_t rawdistance;
  if(readRawDistance(dir,rawdistance)){
    if(isDistanceValid(rawdistance)){
      dataPush(dir,rawdistance,nowMs);
    }
  }

  dir=(Direction)((dir+1)%3);
}//调度函数，在loop里面调用

void setup() {
  for (uint8_t i = 0; i < 3; i++) {
    slots[i].writeIndex = 0;
    slots[i].sampleCount = 0;
    slots[i].lastValidMs = 0;
  }//初始化结构体
  Serial.begin(115200);
  delay(1000); //等待串口初始化

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);//设置通信串口和频率

  Serial.println("Checking TCA9548A and VL53L1X modules...");
  checkTofModules();//检查设备是否在线

  for (uint8_t i = 0; i < 3; i++) {
    initTofOnChannel(i, slots[i].tof);
  }//初始化激光传感器
}

void loop() {

}
