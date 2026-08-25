#include "actions.h"

MoveActions::MoveActions(gpio_num_t txPin, gpio_num_t rxPin, MPU6050 &mpu)
    : _txPin(txPin), _rxPin(rxPin), _mpu(mpu)
{
}

void MoveActions::init()
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(_txPin, _rxPin, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        Serial.println("Driver installed");
    }
    else
    {
        Serial.println("Failed to install driver");
        return;
    }
    if (twai_start() == ESP_OK)
    {
        Serial.println("Driver started");
    }
    else
    {
        Serial.println("Failed to start driver");
        return;
    }
}

bool MoveActions::SendMessage(int id)
{
    twai_message_t message;

    message.extd = 0;
    message.rtr = 0;
    message.dlc_non_comp = 0;
    message.identifier = kBaseCanId + id;
    message.data_length_code = 8;
    for (int i = 0; i < 8; i++)
    {
        message.data[i] = _sendData[i];
    }

    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK)
    {
        if (kDebug)
            Serial.println("Message queued for transmission\n");
        return true;
    }
    else
    {
        if (kDebug)
            Serial.println("Failed to queue message for transmission\n");
        return false;
    }
}

bool MoveActions::motorOff(int id)
{
    _sendData[0] = 0x80;
    for (int i = 1; i < 8; i++)
        _sendData[i] = 0x00;
    return SendMessage(id);
}

bool MoveActions::motorOn(int id)
{
    _sendData[0] = 0x88;
    for (int i = 1; i < 8; i++)
        _sendData[i] = 0x00;
    return SendMessage(id);
}

bool MoveActions::motorStop(int id)
{
    _sendData[0] = 0x81;
    for (int i = 1; i < 8; i++)
        _sendData[i] = 0x00;
    return SendMessage(id);
}

bool MoveActions::SetVelocity(int id, int32_t velocity)
{
    _sendData[0] = 0xA2;
    _sendData[1] = 0x00;
    _sendData[2] = 0x00;
    _sendData[3] = 0x00;
    _sendData[4] = (velocity & 0x000000FF);
    _sendData[5] = (velocity & 0x0000FF00) >> 8;
    _sendData[6] = (velocity & 0x00FF0000) >> 8 * 2;
    _sendData[7] = (velocity & 0xFF000000) >> 8 * 3;
    return SendMessage(id);
}

void MoveActions::Drive(int32_t v1, int32_t v2, int32_t v3, int32_t v4)
{
    SetVelocity(1, v2);
    SetVelocity(2, v3);
    SetVelocity(3, v4);
    SetVelocity(4, v1);
}



void KickActions::Kick()
{
    if (millis() - last_kick_time > 3000)
    {
        time = millis();
        digitalWrite(26, 1);
        while (millis() - time < 100);
        digitalWrite(26, 0);
        last_kick_time = millis();
    }
}
