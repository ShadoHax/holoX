#include "main.h"
#include "lsd-koyori.hpp"

using namespace pros;

// The PID control is to travel more accurately. Normally, if we were to run at a constant speed and then stop, our robot would likely overshoot the target stopping point, or make a jerky stop that destabilizes it. PID control allows us to slow down the robot as it approaches the target, so that it can stop more accurately, allowing more consistency during the auton phase.

lsd_koyori::lsd_koyori(double setP, double setD, double min, int countLive)
{
    kP = setP;
    kD = setD;
    spdFloor = min;
    count = countLive;
    // PID presets, defined in our Robot program, for easier tuning
}
double lsd_koyori::get_spd(double error)
{
    int time = millis();
    int timeChange = time - lasttime;
    // according to other teams the delay is inaccurate, so this is a more precise time calculation
    double errorDX = (error - lasterror) / timeChange;
    // calculating derivative of error to compensate 
    lasterror = error;
    lasttime = time;
    count++;
    double spd = (kP * error) + (kD * errorDX);
    double coefficient = (std::min(100, count))/100;
    return (coefficient * (abs(spd) > spdFloor ? spd : /* janky abs? */(spd > 0) ? spdFloor : -spdFloor));
    // calculates the updated target speed and returns it

}