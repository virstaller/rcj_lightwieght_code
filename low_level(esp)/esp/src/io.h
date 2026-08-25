#include <Arduino.h>
#include <Wire.h>

class Uart{
    public:
        void init();
    private:
        bool cameraOK = 1;
        
        union CameraAngle {
            byte b[2];
            int16_t i16;
        } CameraB, CameraGB, CameraGY, distB, distGB, distGY;
};

class Can{
    void init();
};

class I2C{
    void init();
};