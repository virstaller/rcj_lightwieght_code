// #include <Arduino.h>
// #include <GyverOLED.h>
// #include <MPU6050_light.h>

// #include "Wire.h"
// #include "actions.h"
// #include "driver/twai.h"

// MPU6050 mpu(Wire);

// GyverOLED<SSD1306_128x64, OLED_BUFFER> oled(0x3D);

// #define DWR digitalWrite
// #define DRD digitalRead
// #define AWR analogWrite
// #define ARD analogRead
// #define PMD pinMode
// #define str String

// #define RX_PIN 5
// #define TX_PIN 4

// #define RXD2 41
// #define TXD2 42

// #define BTN1 17
// #define BTN2 18
// #define BTN3 48
// #define BTN4 38
// #define LED 47

// #define SLD 6

// #define S1 12
// #define S2 11
// #define S3 10
// #define AIN1 14
// #define AIN2 13

// #define DEBUG 0

// // Исполнитель: превращает ActionValues в CAN-команды моторам.
// MotorExecutor motors((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN);

// // Контекст, который нужен действиям (сейчас - только гироскоп).
// ActionDomain domain{mpu};

// static union convUnion {
//   byte b[8];
//   uint8_t u8[8];
//   int8_t i8[8];
//   uint16_t u16[4];
//   int16_t i16[4];
//   uint32_t u32[2];
//   int32_t i32[2];
//   uint64_t u64;
//   int64_t i64;
//   float f[2];
//   double d;
// } conv;

// uint16_t lineSensorsMin[12];
// uint16_t lineSensorsMax[12];
// int16_t lineSensors[12];

// void calibrateLine(bool upd = 0) {  // bool init = 0) {
//   if (upd) {
//     for (int n = 0; n < 12; n++) {
//       lineSensorsMin[n] = 0b1111111111111111;
//       lineSensorsMax[n] = 0;
//     }
//     DWR(LED, 1);
//     while (!DRD(BTN2)) {
//       for (int n = 0; n < 6; n++) {
//         DWR(S1, bitRead(n, 0));
//         DWR(S2, bitRead(n, 1));
//         DWR(S3, bitRead(n, 2));
//         delayMicroseconds(10);

//         uint16_t v1 = analogRead(AIN1);
//         uint16_t v2 = analogRead(AIN2);

//         if (v1 < lineSensorsMin[n]) lineSensorsMin[n] = v1;
//         if (v1 > lineSensorsMax[n]) lineSensorsMax[n] = v1;

//         if (v2 < lineSensorsMin[11 - n]) lineSensorsMin[11 - n] = v2;
//         if (v2 > lineSensorsMax[11 - n]) lineSensorsMax[11 - n] = v2;
//       }
//     }
//     DWR(LED, 0);
//     for (int n = 0; n < 12; n++) {
//       Serial.print("    lineSensorsMin[");
//       Serial.print(n);
//       Serial.print("] = ");
//       Serial.print(lineSensorsMin[n]);
//       Serial.println(";");
//     }
//     for (int n = 0; n < 12; n++) {
//       Serial.print("    lineSensorsMax[");
//       Serial.print(n);
//       Serial.print("] = ");
//       Serial.print(lineSensorsMax[n]);
//       Serial.println(";");
//     }
//     while (1);
//   } else {
//     lineSensorsMin[0] = 820;
//     lineSensorsMin[1] = 340;
//     lineSensorsMin[2] = 509;
//     lineSensorsMin[3] = 1207;
//     lineSensorsMin[4] = 1039;
//     lineSensorsMin[5] = 472;
//     lineSensorsMin[6] = 1894;
//     lineSensorsMin[7] = 921;
//     lineSensorsMin[8] = 668;
//     lineSensorsMin[9] = 664;
//     lineSensorsMin[10] = 668;
//     lineSensorsMin[11] = 846;

//     lineSensorsMax[0] = 1342;
//     lineSensorsMax[1] = 865;
//     lineSensorsMax[2] = 975;
//     lineSensorsMax[3] = 1629;
//     lineSensorsMax[4] = 1525;
//     lineSensorsMax[5] = 921;
//     lineSensorsMax[6] = 2443;
//     lineSensorsMax[7] = 1501;
//     lineSensorsMax[8] = 1293;
//     lineSensorsMax[9] = 1297;
//     lineSensorsMax[10] = 1272;
//     lineSensorsMax[11] = 1479;
//   }
// }

// float LineAngleLUT[12] = {};
// void countLineLUT() {
//   for (int n = 0; n < 12; n++) {
//     LineAngleLUT[n] = n * (360 / 12);
//   }
// }

// float anglemin;
// float anglemax;
// bool isline = 0;
// void updateLine() {
//   anglemin = 360;
//   anglemax = 0;
//   for (int n = 0; n < 6; n++) {
//     DWR(S1, bitRead(n, 0));
//     DWR(S2, bitRead(n, 1));
//     DWR(S3, bitRead(n, 2));
//     delay(1);
//     int delta1 = analogRead(AIN1) - lineSensors[n];
//     int delta2 = analogRead(AIN2) - lineSensors[11 - n];
//     lineSensors[n] = delta1 * 0.4;
//     lineSensors[11 - n] = delta2 * 0.4;
//     if (lineSensors[n] > 40) {
//       isline = 1;
//       anglemin = min(anglemin, LineAngleLUT[n]);
//       anglemax = max(anglemax, LineAngleLUT[n]);
//     }
//     if (lineSensors[11 - n] > 40) {
//       isline = 1;
//       anglemin = min(anglemin, LineAngleLUT[11 - n]);
//       anglemax = max(anglemax, LineAngleLUT[11 - n]);
//     }
//   }
// }

// #define ballSensor_CAN_ADDRESS 0x150

// int16_t getBallAngle() {
//   twai_message_t request;
//   request.identifier = ballSensor_CAN_ADDRESS;
//   request.extd = 0;              // стандартный ID
//   request.rtr = 1;               // Remote Frame
//   request.data_length_code = 8;  // запрашиваем 8 байт

//   esp_err_t ret = twai_transmit(&request, pdMS_TO_TICKS(800));
//   if (ret != ESP_OK) {
//     Serial.print("TRANSMIT CAN error: ");
//     Serial.println(ret);
//     return 444;
//   }

//   twai_message_t response;
//   ret = twai_receive(&response, pdMS_TO_TICKS(800));

//   if (ret == ESP_OK) {
//     if (response.identifier == ballSensor_CAN_ADDRESS && !response.rtr) {
//       for (int n = 0; n < 8; n++) {
//         conv.b[n] = response.data[n];
//       }
//       if (DEBUG) Serial.println("CAN OK.");
//       if (DEBUG) Serial.print("Angle = ");
//       if (DEBUG) Serial.println(conv.i16[0]);
//       return conv.i16[0];
//     }
//   } else if (ret == ESP_ERR_TIMEOUT) {
//     if (DEBUG) Serial.print("RECIEVE CAN timeout.");
//     return 444;
//   } else {
//     if (DEBUG) Serial.print("RECIEVE CAN not OK. Adress: ");
//     if (DEBUG) Serial.println(response.identifier);
//     return 444;
//   }
//   return 444;
// }

// void printOneCircle(float a_ball) {
//   Wire.setClock(800000L);
//   oled.clear();
//   oled.home();
//   oled.circle(64, 32, 30, OLED_STROKE);
//   int lx = 64 + 30 * sin((a_ball - 90) * 1000.0 / 57296.0);
//   int ly = 32 + 30 * cos((a_ball - 90) * 1000.0 / 57296.0);
//   oled.line(64, 32, lx, ly);
//   oled.circle(lx, ly, 5, OLED_STROKE);
//   oled.update();
//   Wire.setClock(100000);
// }

// void printCircle(float a_ball, float a_goal, float a_robot) {
//   Wire.setClock(800000L);
//   oled.clear();
//   oled.home();
//   oled.print(a_ball);
//   oled.circle(64, 32, 30, OLED_STROKE);
//   int lx = 64 + 30 * sin((a_ball - 90) * 1000.0 / 57296.0);
//   int ly = 32 + 30 * cos((a_ball - 90) * 1000.0 / 57296.0);
//   oled.line(64, 32, lx, ly);
//   oled.circle(lx, ly, 5, OLED_STROKE);

//   lx = 64 + 30 * sin((a_goal - 90) * 1000.0 / 57296.0);
//   ly = 32 + 30 * cos((a_goal - 90) * 1000.0 / 57296.0);
//   oled.line(64, 32, lx, ly);
//   oled.circle(lx, ly, 15, OLED_STROKE);

//   lx = 64 + 30 * sin((a_robot - 90) * 1000.0 / 57296.0);
//   ly = 32 + 30 * cos((a_robot - 90) * 1000.0 / 57296.0);
//   oled.line(64, 32, lx, ly);
//   oled.update();
//   Wire.setClock(100000);
// }

// union CameraAngle {
//   byte b[2];
//   int16_t i16;
// } CameraB, CameraGB, CameraGY, distB, distGB, distGY;

// bool cameraOK = 1;

// void updateCamera() {
//   if (cameraOK)
//     Serial2.print("?");
//   cameraOK = 0;
//   static uint32_t t_s = millis();
//   DWR(LED, 0);
//   while ((Serial2.available() < (3 * 6 + 1)));
//   DWR(LED, 1);
//   if (0) {
//     Serial.println("CAMERA TIMEOUT");
//   } else {
//     byte buf[3 * 6 + 1];
//     Serial2.readBytes(buf, 3 * 6 + 1);
//     Serial.println();
//     if (buf[0] == 0x00) {
//       for (int n = 0; n < 3 * 6; n++) {
//         buf[n] = buf[n + 1];
//       }
//     }
//     buf[3 * 6] = Serial2.read();
//     bool h1Bad = buf[0] != 0xAA;
//     bool h2Bad = buf[3] != 0xBB;
//     bool h3Bad = buf[6] != 0xCC;

//     bool h4Bad = buf[9] != 0xDD;
//     bool h5Bad = buf[12] != 0xEE;
//     bool h6Bad = buf[15] != 0x11;
//     bool endBad = buf[18] != 0xFF;

//     if (h1Bad | h2Bad | h3Bad | h4Bad | h5Bad | h6Bad | endBad) {
//       Serial.print("CAMERA READ ERROR: ");
//       Serial.println();
//     } else {
//       cameraOK = 1;
//       CameraB.b[0] = buf[1];
//       CameraB.b[1] = buf[2];
//       CameraGB.b[0] = buf[4];
//       CameraGB.b[1] = buf[5];
//       CameraGY.b[0] = buf[7];
//       CameraGY.b[1] = buf[8];

//       distB.b[0] = buf[10];
//       distB.b[1] = buf[11];
//       distGB.b[0] = buf[13];
//       distGB.b[1] = buf[14];
//       distGY.b[0] = buf[16];
//       distGY.b[1] = buf[17];
//     }
//   }
// }

// float KpG = 680;
// float KdG = 7750;
// int lastErrG = 0;
// int GatesAdj(int Gates) {
//   int err = Gates;
//   int uv = err * KpG + (err - lastErrG) * KdG;
//   lastErrG = err;
//   return uv;
// }

// void goAroundBall(int Ball, int Gates, int V, bool prinCirc = 1) {
//   int gAdj = GatesAdj(Gates);
//   int bangle = Ball;
//   if (bangle < 0) {
//     bangle += 360;
//   }
//   int RobotAngle = (bangle > 180) ? (bangle - 60) : (bangle + 60);

//   ActionValues out;
//   Actions::DriveAngle(RobotAngle, V, gAdj).process(domain, out);
//   motors.apply(out);

//   if (prinCirc)
//     printCircle(bangle, Gates, RobotAngle);
// }

// void goTowardsBall(int Ball, int Gates, int V, bool prinCirc = 1) {
//   int gAdj = GatesAdj(Gates);

//   ActionValues out;
//   Actions::DriveAngle(Gates, V, gAdj).process(domain, out);
//   motors.apply(out);
// }

// void setup() {
//   Serial.begin(115200);
//   Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
//   countLineLUT();
//   Serial.println("I'M ALIVE!!!");
//   Serial.println("I'M ALIVE!!!");
//   Serial.println("I'M ALIVE!!!");
//   motors.begin();
//   motors.motorOn(1);
//   motors.motorOn(2);
//   motors.motorOn(3);
//   motors.motorOn(4);
//   Wire.begin();
//   PMD(S1, OUTPUT);
//   PMD(S2, OUTPUT);
//   PMD(S3, OUTPUT);
//   PMD(AIN1, INPUT);
//   PMD(AIN2, INPUT);
//   PMD(BTN1, INPUT_PULLDOWN);
//   PMD(BTN2, INPUT_PULLDOWN);
//   PMD(BTN3, INPUT_PULLDOWN);
//   PMD(BTN4, INPUT_PULLDOWN);
//   PMD(LED, OUTPUT);
//   PMD(SLD, OUTPUT);

//   DWR(LED, 1);
//   DWR(SLD, 0);

//   while (!DRD(BTN2));
//   DWR(LED, 0);
//   delay(200);

//   DWR(S1, 0);
//   DWR(S2, 0);
//   DWR(S3, 0);
// }

// #define fwdErr 20
// void striker(int32_t V) {
//   delay(20);
//   int IRballAngle = getBallAngle();
//   updateCamera();
//   int ballAngle = CameraB.i16;
//   if ((ballAngle == 444) or (ballAngle == 0) or (cameraOK != 1)) {
//     Serial.print("!");
//     ballAngle = 0 - IRballAngle;
//   }
//   if ((CameraGB.i16 == 444) or (cameraOK != 1)) {
//     CameraGB.i16 = 5;
//   }
//   Serial.println(ballAngle);

//   if (((abs(ballAngle) < fwdErr) or (abs(360 - ballAngle) < fwdErr)) and (abs(CameraGB.i16) < fwdErr * 2)) {
//     goTowardsBall(ballAngle, CameraGB.i16, V);
//   } else {
//     goAroundBall(ballAngle, CameraGB.i16, V * 0.5);
//   }
// }

// float kp_goalkeeper = 300;
// float kd_goalkeeper = 600;
// int lastErr_goalkeeper = 0;
// void goalkeeper(int V) {
//   updateCamera();
//   int IRballAngle = getBallAngle();
//   int ballAngle = CameraB.i16;
//   if ((CameraB.i16 == 444) or (cameraOK != 1)) {
//     ballAngle = IRballAngle;
//   }
//   if ((CameraGB.i16 == 444) or (cameraOK != 1)) {
//     CameraGB.i16 = 0;
//   }
//   if ((CameraGY.i16 == 444) or (cameraOK != 1)) {
//     CameraGY.i16 = 0;
//   }

//   int gAdj = GatesAdj(CameraGB.i16);
//   int err = ballAngle;
//   lastErr_goalkeeper = err;
// }

// int mode = 0;
// void loop() {
//   striker(200000);
// }
