#include "main.h"
#include "lsd-koyori.hpp"
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <deque>

using namespace pros;

struct Robot 
{
    // Declaring controller, motors, and sensors
    static Controller master;
    static Motor L1;
    static Motor L2;
    static Motor L3;
    static Motor R1;
    static Motor R2;
    static Motor R3;
    static Motor IntakeRoller;
    static Motor Flywheel;
    static Imu IMU;
    static ADIEncoder xEncoder;
    static ADIEncoder yEncoder;
    static ADIDigitalOut EXPANSION;
    // Declaring PID instances
    static lsd_koyori rotationPID;
    static lsd_koyori travelPID;
    static lsd_koyori flywheelPID;
    
    static void ckEXPAND();
    static bool driver;
    static void Driver();
    static bool flywheelSpinning;
    static bool intakeEating;

    // Intialize some default values for motors and shit
    static void motorInit();

    // Set robot braking type, defaulting to brake
    static void brake(std::string braketype);
    static void recorder(int Lstick, int Rstick, bool rt2, bool rt1, bool lt2, bool lt1, bool x, bool b, bool up, bool down); //shhhhhh
    static void ghostdriver(int Lstick, int Rstick, bool rt2, bool rt1, bool lt2, bool lt1, bool x, bool b, bool up, bool down);
    static void ghostEXPAND(bool down, bool b, bool up, bool x);
    static void doRoller();
    static void findHeading(std::vector<double> target);
    static void reset();
};