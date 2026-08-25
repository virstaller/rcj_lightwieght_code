#include "devices.h"

std::string toLower(const std::string& str) {
    std::string lowerStr = str;
    std::transform(lowerStr.begin(), lowerStr.end(), lowerStr.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return lowerStr;
}


std::any Camera::GetData() const {
    return Coordinates{1.5, 2.3};
}


std::any LineSensor::GetData() const {
    return true;
}


std::any BallSensor::GetData() const {
    return 42.5f;
}


DeviceFactory::DeviceFactory() {
    registry["camera"] = []() { return std::make_unique<Camera>(); };
    registry["line"]   = []() { return std::make_unique<LineSensor>(); };
    registry["ball"]   = []() { return std::make_unique<BallSensor>(); };
}

std::unique_ptr<Device> DeviceFactory::create(const std::string &deviceName) {
    std::string lowerName = toLower(deviceName);
    auto it = registry.find(lowerName);
    if (it != registry.end()) {
        return it->second();
    }
    throw std::invalid_argument("Unknown device: " + deviceName);
}

void DeviceFactory::registerDevice(const std::string &name, Creator creator) {
    registry[toLower(name)] = std::move(creator);
}