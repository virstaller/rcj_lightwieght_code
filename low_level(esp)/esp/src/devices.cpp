#include "devices.h"

std::string toLower(const std::string &str)
{
    std::string lowerStr = str;
    std::transform(lowerStr.begin(), lowerStr.end(), lowerStr.begin(),
                   [](unsigned char c)
                   { return std::tolower(c); });
    return lowerStr;
}

// Формат пакета (7 байт): [0xAA][lo][hi][0xBB][lo][hi][0xFF]
void Camera::updateCamera() const
{
    if (cameraOK)
        Serial2.print("?");
    cameraOK = false;

    const int PACKET_SIZE = 2 * 3 + 1; // 7 байт

    digitalWrite(LED, 0);

    while (Serial2.available() < PACKET_SIZE)
        ;

    digitalWrite(LED, 1);

    byte buf[PACKET_SIZE];
    Serial2.readBytes(buf, PACKET_SIZE);

    if (buf[0] == 0x00)
    {
        for (int n = 0; n < PACKET_SIZE - 1; n++)
        {
            buf[n] = buf[n + 1];
        }
        buf[PACKET_SIZE - 1] = Serial2.read();
    }

    bool h1Bad = buf[0] != 0xAA;
    bool h2Bad = buf[3] != 0xBB;
    bool endBad = buf[6] != 0xFF;

    if (h1Bad | h2Bad | endBad)
    {
        Serial.print("CAMERA READ ERROR: ");
        Serial.println();
        return;
    }

    cameraOK = true;
    val1.b[0] = buf[1];
    val1.b[1] = buf[2];
    val2.b[0] = buf[4];
    val2.b[1] = buf[5];
}

std::any Camera::GetData() const
{
    updateCamera();
    return Coordinates{
        (double)val1.i16,
        (double)val2.i16};
}

std::any LineSensor::GetData() const
{
    return 0;
}

std::any BallSensor::GetData() const
{
    return 0;
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