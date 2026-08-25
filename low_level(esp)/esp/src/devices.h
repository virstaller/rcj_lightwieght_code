#pragma once
#include <Arduino.h>
#include <string>
#include <memory>
#include <unordered_map>
#include <functional>
#include <stdexcept>

std::string toLower(std::string str);

class Device
{
public:
    virtual ~Device() = default;
    virtual void ReadData() = 0;
};

class Camera : public Device
{
public:
    void ReadData();
};

class LineSensor : public Device
{
public:
    void ReadData();
};

class BallSensor : public Device
{
public:
    void ReadData();
};

class DeviceFactory
{
public:
    using Creator = std::function<std::unique_ptr<Device>()>;

    DeviceFactory();

    std::unique_ptr<Device> create(const std::string &deviceName);
    void registerDevice(const std::string &name, Creator creator);

private:
    std::unordered_map<std::string, Creator> registry;
};
