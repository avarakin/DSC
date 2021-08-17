#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "sensor.h"
#include "sensor_bno055.h"


/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

const adafruit_bno055_offsets_t oldCalib = {-68, -14, -48, -201, 275, 100, 1, -1, -1, 1000, 480};

void SensorBNO055::init()
{
   bno = Adafruit_BNO055(55);

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor as relative mode (mag is not used) */
  if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  Serial.println("\n\nRestoring Calibration data to the BNO055...");
  bno.setSensorOffsets(oldCalib);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Calibrate Again*/
  while (!fullyCalibrated_imu()) {
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }


}


Coordinates SensorBNO055::getCoordinates()
{
    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> ori = quat.toEuler();

    Serial.printf("X:%f Y:%f Z:%f",  ori.x(), ori.y(), ori.z());

    return Coordinates(ori.z()*180/PI*100,  -ori.x()*180/PI*100);
}

/* Check whether it is fullycalibrated for IMU mode (mag is not used)*/
bool SensorBNO055::fullyCalibrated_imu(void) {
    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    return !(gyro < 3 || accel < 3);
}

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void SensorBNO055::displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


