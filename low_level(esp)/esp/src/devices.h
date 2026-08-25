#pragma once
#include <Arduino.h>
#include <string>
#include <memory>
#include <unordered_map>
#include <functional>
#include <stdexcept>
#include <algorithm>
#include <cctype>
#include <any>

struct Coordinates {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

std::string toLower(const std::string& str);

class Device {
public:
    virtual ~Device() = default;
    virtual std::any GetData() const = 0;
};

class Camera : public Device {
public:
    std::any GetData() const override;
private:
    static constexpr int RXD2 = 41;
    static constexpr int TXD2 = 42;
    static constexpr int LED  = 47;
};

class LineSensor : public Device {
public:
    std::any GetData() const override;
};

class BallSensor : public Device {
public:
    std::any GetData() const override;
};

class DeviceFactory {
public:
    using Creator = std::function<std::unique_ptr<Device>()>;
    DeviceFactory();
    std::unique_ptr<Device> create(const std::string &deviceName);
    void registerDevice(const std::string &name, Creator creator);
private:
    std::unordered_map<std::string, Creator> registry;
};

class DeviceManager {
    DeviceFactory factory;
    std::unordered_map<std::string, std::unique_ptr<Device>> devices;

public:
    void addDevice(const std::string& name) {
        devices[toLower(name)] = factory.create(name);
    }

    std::any GetData(const std::string& name) {
        auto it = devices.find(toLower(name));
        if (it == devices.end()) {
            throw std::invalid_argument("Device not found: " + name);
        }
        return it->second->GetData();
    }
};