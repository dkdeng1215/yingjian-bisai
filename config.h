#ifndef CONFIG_H
#define CONFIG_H

// 引脚定义
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

#define MOTOR_LEFT_PIN 4
#define MOTOR_CENTER_PIN 5
#define MOTOR_RIGHT_PIN 6

#define STATUS_LED_PIN 7
#define BUTTON_PIN 10

// I2C地址定义
#define TCA9548A_I2C_ADDRESS 0x70
#define VL53L1X_I2C_ADDRESS 0x29
#define I2C_CLOCK_HZ 100000UL
// TOF通道定义
#define TOF_LEFT_CHANNEL 0
#define TOF_CENTER_CHANNEL 1
#define TOF_RIGHT_CHANNEL 2
/* ---------- Scheduling ----------
#define SERIAL_BAUD_RATE            115200UL
#define SERIAL_STARTUP_DELAY_MS     1000

#define TOF_POLL_INTERVAL_MS        45
#define SENSOR_STALE_MS             500
#define LOG_INTERVAL_MS             250
#define FEEDBACK_WATCHDOG_TIMEOUT_MS 200 */
// 传感器测量距离
#define DISTANCE_MAX_MM 4000
#define DISTANCE_MIN_MM 40
// #define FILTER_WINDOW_SIZE          5
// TOF测量值过滤
// 危险距离等级
#define DANGER_THRESHOLD_MM 400
#define WARN_THRESHOLD_MM 800
#define NOTICE_THRESHOLD_MM 1200

#endif