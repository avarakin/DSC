#include <Adafruit_BNO055.h>

class SensorBNO055 : public Sensor
{
public:
    virtual void init();
    virtual Coordinates getCoordinates();

protected:
    Adafruit_BNO055 bno;
};