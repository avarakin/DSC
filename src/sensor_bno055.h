class SensorBNO055 : public Sensor
{
    public:
        virtual void init();
        virtual Coordinates getCoordinates();
};