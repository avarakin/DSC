#include <Adafruit_BNO055.h>

class SensorBNO055 : public Sensor
{
    public:
        virtual void init();
        virtual Coordinates getCoordinates();
    
    protected:
        void displaySensorDetails(void);
        bool fullyCalibrated_imu(void);

    Adafruit_BNO055 bno;
};