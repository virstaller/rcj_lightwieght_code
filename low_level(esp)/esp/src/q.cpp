#include <Arduino.h>
#include <GyverOLED.h>
#include <MPU6050_light.h>

#include "Wire.h"
#include "actions.h"
#include "devices.h"
#include "driver/twai.h"

MPU6050 mpu(Wire);
GyverOLED<SSD1306_128x64, OLED_BUFFER> oled(0x3D);

#define DWR digitalWrite
#define DRD digitalRead
#define AWR analogWrite
#define ARD analogRead
#define PMD pinMode
#define str String

#define RX_PIN 5
#define TX_PIN 4

#define RXD2 41
#define TXD2 42

#define BTN1 17
#define BTN2 18
#define BTN3 48
#define BTN4 38
#define LED 47

#define SLD 6

#define S1 12
#define S2 11
#define S3 10
#define AIN1 14
#define AIN2 13

#define DEBUG 0

MoveActions robot((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, mpu);
DribblerActions dribbler();
KickActions kicker;

static union convUnion
{
  byte b[8];
  uint8_t u8[8];
  int8_t i8[8];
  uint16_t u16[4];
  int16_t i16[4];
  uint32_t u32[2];
  int32_t i32[2];
  uint64_t u64;
  int64_t i64;
  float f[2];
  double d;
} conv;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("I'M ALIVE!!!");
  Serial.println("I'M ALIVE!!!");
  Serial.println("I'M ALIVE!!!");
  robot.init();
  robot.MotorOn(1);
  robot.MotorOn(2);
  robot.MotorOn(3);
  robot.MotorOn(4);
  Wire.begin();
  PMD(S1, OUTPUT);
  PMD(S2, OUTPUT);
  PMD(S3, OUTPUT);
  PMD(AIN1, INPUT);
  PMD(AIN2, INPUT);
  PMD(BTN1, INPUT_PULLDOWN);
  PMD(BTN2, INPUT_PULLDOWN);
  PMD(BTN3, INPUT_PULLDOWN);
  PMD(BTN4, INPUT_PULLDOWN);
  PMD(LED, OUTPUT);
  PMD(SLD, OUTPUT);

  DWR(LED, 1);
  DWR(SLD, 0);

  while (!DRD(BTN2))
    ;
  DWR(LED, 0);
  delay(200);

  DWR(S1, 0);
  DWR(S2, 0);
  DWR(S3, 0);
}
void loop()
{
  robot.Drive(4000, 4000, 4000, 4000);
}