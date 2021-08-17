#include "sensor.h"
#include "sensor_bno055.h"



void SensorBNO055::init()
{

}


Coordinates SensorBNO055::getCoordinates()
{

/*
    ORI_DATA data = getori_bno055();
    Serial.print("Orientation: ");
    Serial.print(data.yaw);
    Serial.print(" ");
    Serial.print(data.pitch);
    Serial.print(" ");
    Serial.println(data.roll);

    int iAzimuthReading = -data.roll*100;
    int iAltitudeReading = data.yaw*100;

*/


    return Coordinates(0,0);
}

