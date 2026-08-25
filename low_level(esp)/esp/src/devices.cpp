#include "devices.h"

std::string toLower(std::string str)
{
    std::transform(str.begin(), str.end(), str.begin(),
                   [](unsigned char c)
                   { return std::tolower(c); });
    return str;
}

void Camera::ReadData()
{
    
}
void LineSensor::ReadData()
{
}
void BallSensor::ReadData()
{
}

DeviceFactory::DeviceFactory()
{
    registry["camera"] = []()
    { return std::make_unique<Camera>(); };
    registry["line"] = []()
    { return std::make_unique<LineSensor>(); };
    registry["ball"] = []()
    { return std::make_unique<BallSensor>(); };
}

std::unique_ptr<Device> DeviceFactory::create(const std::string &deviceName)
{
    std::string lowerName = toLower(deviceName);

    auto it = registry.find(lowerName);
    if (it != registry.end())
    {
        return it->second();
    }

    throw std::invalid_argument("Unknown device: " + deviceName);
}

void DeviceFactory::registerDevice(const std::string &name, Creator creator)
{
    registry[toLower(name)] = std::move(creator);
}
