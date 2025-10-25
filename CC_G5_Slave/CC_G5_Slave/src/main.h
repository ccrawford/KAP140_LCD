#include <Wire.h>


class G5_CONTROL
{
private:
    int _powerButtonPin;
    int _powerLEDPin;
    int _encoderAPin;
    int _encoderBPin;
    int _encoderButtonPin;
    int _interruptPin;
    
    TwoWire * i2cChannel;
public:
    G5_CONTROL(/* args */);
    ~G5_CONTROL();
};

G5_CONTROL::G5_CONTROL(/* args */)
{
}

G5_CONTROL::~G5_CONTROL()
{
}
