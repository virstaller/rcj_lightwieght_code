#pragma once

#include <Arduino.h>
#include <MPU6050_light.h>
#include "driver/twai.h"


class MoveActions {
   public:
    // txPin/rxPin - пины TWAI-контроллера ESP32.
    // mpu - ссылка на гироскоп, нужна только для driveAngleFor() (курсовая стабилизация).
    MoveActions(gpio_num_t txPin, gpio_num_t rxPin, MPU6050 &mpu);

    // Инициализирует драйвер TWAI. Вызвать один раз в setup().
    void init();
    
    bool MotorOn(int id);
    bool MotorOff(int id);
    bool MotorStop(int id);

    // Задать скорость мотору
    bool SetVelocity(int id, int32_t velocity);

    // Задать скорости всем четырём моторам сразу.
    void Drive(int32_t v1, int32_t v2, int32_t v3, int32_t v4);
    // Езда в точку с направлением angle
    void GoToPoint(int16_t x, int16_t y, int8_t angle);

   private:
    bool SendMessage(int id);

    gpio_num_t _txPin;
    gpio_num_t _rxPin;
    MPU6050 &_mpu;

    volatile byte _sendData[8];

    static constexpr uint16_t kBaseCanId = 0x140;
    static constexpr bool kDebug = false;
};


class DribblerActions{
    public:
        void SetSpeed(int16_t speed);
};

class KickActions {
public:
    bool is_ball_in;
    void Kick();
private:
    unsigned long time;
    unsigned long last_kick_time;
};