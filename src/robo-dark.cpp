#include "robo-dark.hpp"
#include <cmath>
#include <string>
#include <atomic>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <deque>
#include <bits/stdc++.h>

using namespace pros;
// Define ports for controller, motors, encoders, and IMU
Controller Robot::master(CONTROLLER_MASTER);
Motor Robot::L1(13);
Motor Robot::L2(14);
Motor Robot::L3(11);
Motor Robot::R1(20);
Motor Robot::R2(17);
Motor Robot::R3(16);
Motor Robot::IntakeRoller(1);
Motor Robot::Flywheel(7);
ADIDigitalOut Robot::EXPANSION(1);
ADIDigitalOut Robot::EXPANSION2(2);
// Imu Robot::IMU(9);
// ADIEncoder Robot::xEncoder(2, 3, false);
// ADIEncoder Robot::yEncoder(4, 5, false);
// Create PID instances for each motor grouping
lsd_koyori Robot::rotationPID(0.3, 0.5, 0, 100);
lsd_koyori Robot::travelPID(0.2, 0.5, 0, 100);
lsd_koyori Robot::flywheelPID(0.5, 0.8, 0.9, 100);

// Sets some stuff that needs to be set for motors
void Robot::motorInit()
{
    IntakeRoller.set_brake_mode(MOTOR_BRAKE_BRAKE);
    Flywheel.set_brake_mode(MOTOR_BRAKE_COAST);
    L1.set_reversed(true);
    L2.set_reversed(true);
    L3.set_reversed(true);
    Flywheel.set_reversed(true);
    Flywheel.set_gearing(E_MOTOR_GEARSET_06);
    EXPANSION.set_value(0);
    EXPANSION2.set_value(0);
};

// Checks if both release buttons are pressed to launch EXPANSION, and allows for retraction as well
void Robot::ckEXPAND()
{
    if (master.get_digital(DIGITAL_DOWN) and master.get_digital(DIGITAL_B) and master.get_digital(DIGITAL_RIGHT) and master.get_digital(DIGITAL_Y)) 
    {
        EXPANSION.set_value(1);
        EXPANSION2.set_value(1);
    }
    if (master.get_digital(DIGITAL_UP) and master.get_digital(DIGITAL_X)) 
    {
        EXPANSION.set_value(0);
        EXPANSION2.set_value(0);
    }
}

void Robot::ghostEXPAND(bool down, bool b, bool up, bool x)
{
    if (down and b) 
    {
        EXPANSION.set_value(1);
        EXPANSION2.set_value(1);
    }
    if (up and x)
    {
        EXPANSION.set_value(0);
        EXPANSION2.set_value(0);
    }
};
// Drivercode, feeds driver control directly into the motors

void Robot::recorder(int Lstick, int Rstick, bool rt2, bool rt1, bool lt2, bool lt1, bool x, bool b, bool up, bool down, bool a, bool left)
{
    FILE *inputs = fopen("/usd/inputs.txt", "a");
    std::string sls = std::to_string(Lstick);
    std::string srs = std::to_string(Rstick);
    std::string srt2 = std::to_string(rt2);
    std::string srt1 = std::to_string(rt1);
    std::string slt2 = std::to_string(lt2);
    std::string slt1 = std::to_string(lt1);
    std::string sx = std::to_string(x);
    std::string sb = std::to_string(b);
    std::string sup = std::to_string(up);
    std::string sdown = std::to_string(down);
    std::string sa = std::to_string(a);
    std::string sleft = std::to_string(left);
    std::string out = "(" + sls + ", " + srs + ", " + srt2 + ", " + srt1 + ", " + slt2 + ", " + slt1 + ", " + sx + ", " + sb + ", " + sup + ", " + sdown + sa + sleft + "),\n";
    fprintf(inputs, out.c_str());
    fclose(inputs);
};

// FILE *inputs = fopen("/usd/inputs.txt", "a");
void Robot::throttlelog(bool throttle, double Lpi, double Rpi)
{
    FILE *inputs = fopen("/usd/inputs.txt", "a");
    std::string s = std::to_string(throttle);
    std::string sl = std::to_string(Lpi);
    std::string sr = std::to_string(Rpi);
    std::string out = "(" + s + ", " + sl + ", " + sr + "),\n";
    fprintf(inputs, out.c_str());
    fclose(inputs);
};

void Robot::Driver()
{
    // give the driver time to react lol
    // delay(500); hehe nvm
    // preset the values to false so we don't start off running lol
    // FILE *inputs = fopen("/usd/inputs.txt", "a");
    // std::string uhidk = "Begin Session\n";
    // fprintf(inputs, uhidk.c_str());
    // fclose(inputs);
    bool throttled = false;
    bool flywheelSpinning = false;
    bool intakeEating = false;
    int time = 0;
    while (true) {
        time ++;
        // THIS PART CALLS THE MOVE FUNC WHICH WE DON'T HAVE IMU FOR YET
        // FIX THIS ONCE WE HAVE A WORKING IMU
        // For tank drive, we just get the values of the left and right joysticks vertical lol;        
        double Lp = master.get_analog(ANALOG_LEFT_Y);
        double Rp = master.get_analog(ANALOG_RIGHT_Y);
        // Intake and Flywheel button check
        bool intakeIn = master.get_digital(DIGITAL_R2);
        bool intakeOut = master.get_digital(DIGITAL_R1);
        bool flywheelShoot = master.get_digital(DIGITAL_L2);
        bool flywheelSuck = master.get_digital(DIGITAL_L1);
        // recorder(Lp, Rp, intakeIn, intakeOut, flywheelShoot, flywheelSuck, master.get_digital(DIGITAL_X), master.get_digital(DIGITAL_B), master.get_digital(DIGITAL_UP), master.get_digital(DIGITAL_DOWN), master.get_digital(DIGITAL_A), master.get_digital(DIGITAL_LEFT));
        if (master.get_digital(DIGITAL_LEFT)) throttled = true;
        else if (master.get_digital(DIGITAL_A)) throttled = false;
        if (throttled)
        {
            double Lp = Lp * 0.6;
            double Rp = Rp * 0.6;
        }
        // throttlelog(throttled, Lp, Rp);
        L1.move(Lp);
        L2.move(Lp);
        L3.move(Lp);
        R1.move(Rp);
        R2.move(Rp);
        R3.move(Rp);
        // Intake control is simple, no PID, we want constant rotation rate to have stable roller control
        if (intakeIn) IntakeRoller = 125;
        else if (intakeOut) IntakeRoller = -125;
        else IntakeRoller = 0;
        // Flywheel control is determined entirely by PROS builtin PID, so we just set the target velocity and it does the rest
        if (flywheelShoot) Flywheel = 126;
        else if (flywheelSuck) Flywheel = 0;
        lcd::set_text(3, std::to_string(Flywheel.get_actual_velocity()));
        // else
        // {Flywheel = 0;};
        ckEXPAND();
        delay(2);
    }
};

void Robot::ghostdriver(int Lstick, int Rstick, bool rt2, bool rt1, bool lt2, bool lt1, bool x, bool b, bool up, bool down, bool a, bool left)
{
    // give the driver time to react lol
    // preset the values to false so we don't start off running lol
    bool throttled = false;
    bool flywheelSpinning = false;
    bool intakeEating = false;
    int time = 0;
    while (true) {
        time ++;
        // THIS PART CALLS THE MOVE FUNC WHICH WE DON'T HAVE IMU FOR YET
        // FIX THIS ONCE WE HAVE A WORKING IMU
        // For tank drive, we just get the values of the left and right joysticks vertical lol;        
        int Lp = Lstick;
        int Rp = Rstick;
        if (left and a) throttled = !throttled;
        if (throttled)
        {
            int Lp = Lstick * 0.7;
            int Rp = Rstick * 0.7;
        }

        // Intake and Flywheel button check
        bool intakeIn = rt2;
        bool intakeOut = rt1;
        bool flywheelShoot = lt2;
        bool flywheelSuck = lt1;
        
        L1.move(Lp);
        L2.move(Lp);
        L3.move(Lp);
        R1.move(Rp);
        R2.move(Rp);
        R3.move(Rp);
        // Intake control is simple, no PID, we want constant rotation rate to have stable roller control
        if (intakeIn) 
        {IntakeRoller = 122;}
        else if (intakeOut) 
        {IntakeRoller = -122;}
        else 
        {IntakeRoller = 0;};
        // Flywheel control is determined entirely by PROS builtin PID, so we just set the target velocity and it does the rest
        if (flywheelShoot) 
        {Flywheel.move_velocity(36000);}
        else if (flywheelSuck)
        // {Flywheel.move_velocity(36000);}
            {Flywheel = 0;};
        // else
        // {Flywheel = 0;};
        ghostEXPAND(down, b, up, x);
        delay(2);
    }
};

// Set brake type for opcontrol and autonomous
void Robot::brake(std::string braketype)
{
    if (braketype == "hold")
    {
        L1.set_brake_mode(MOTOR_BRAKE_HOLD);
        L2.set_brake_mode(MOTOR_BRAKE_HOLD);
        L3.set_brake_mode(MOTOR_BRAKE_HOLD);
        R1.set_brake_mode(MOTOR_BRAKE_HOLD);
        R2.set_brake_mode(MOTOR_BRAKE_HOLD);
        R3.set_brake_mode(MOTOR_BRAKE_HOLD);
    }
    else if (braketype == "coast")
    {
        L1.set_brake_mode(MOTOR_BRAKE_COAST);
        L2.set_brake_mode(MOTOR_BRAKE_COAST);
        L3.set_brake_mode(MOTOR_BRAKE_COAST);
        R1.set_brake_mode(MOTOR_BRAKE_COAST);
        R2.set_brake_mode(MOTOR_BRAKE_COAST);
        R3.set_brake_mode(MOTOR_BRAKE_COAST);
    }
    else
    {
        L1.set_brake_mode(MOTOR_BRAKE_BRAKE);
        L2.set_brake_mode(MOTOR_BRAKE_BRAKE);
        L3.set_brake_mode(MOTOR_BRAKE_BRAKE);
        R1.set_brake_mode(MOTOR_BRAKE_BRAKE);
        R2.set_brake_mode(MOTOR_BRAKE_BRAKE);
        R3.set_brake_mode(MOTOR_BRAKE_BRAKE);
    }
};

// For autonomous, roller function, adjustable based on roller motor gearing
void Robot::doRoller()
{
    // L1 = -127;
    // L2 = -127;
    // L3 = -127;
    // R1 = -127;
    // R2 = -127;
    // R3 = -127;
    // IntakeRoller = 127;

    L1.move_relative(-100, 100);
    L2.move_relative(-100, 100);
    L3.move_relative(-100, 100);
    R1.move_relative(-100, 100);
    R2.move_relative(-100, 100);
    R3.move_relative(-100, 100);
    delay(1000);
    IntakeRoller.move_relative(1400, 50); // Change the 100 to change distance for roller
}



void Robot::reset()
{
    IMU.reset();
    L1.tare_position();
    R1.tare_position();
    xEncoder.reset();
    yEncoder.reset();
}