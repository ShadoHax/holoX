#include "main.h"
// The entire class is a PID object with unique kP and kD, allowing different motor groups to have different tuning.
class lsd_koyori
{
    public:
        double kP;
        double kD;
        double spdFloor;
        int count;
        // PID presets, defined in code
        double lasterror;
        int lasttime;
        double derivative;
        int counter_reset;
        // PID calculation variables
        lsd_koyori(double setP, double setD, double min=0, int countLive=100);
        double get_spd(double error);
        void reset();
        // defining the constructor for the class, 

};