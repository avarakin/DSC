class Coordinates {
    private:
        int alt;
        int az;
    public:
        Coordinates(int _alt, int _az) {
            alt = _alt;
            az = _az;
        }
        int getAlt() { return alt;}
        int getAz() { return az;}
};

class Sensor 
{
    public:
        virtual void init() = 0;
        virtual Coordinates getCoordinates() = 0;
};