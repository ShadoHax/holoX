#include "robo-dark.hpp"
#include <cmath>
#include <atomic>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <deque>
#include <bits/stdc++.h>

using namespace pros;

Controller Robot::dom(CONTROLLER_MASTER);
Motor Robot::L1(13);
Motor Robot::L2(14);
Motor Robot::L3(11);
Motor Robot::R1(20);
Motor Robot::R2(17);
Motor Robot::R3(16);
Motor Robot::IntakeRoller(1);
Motor Robot::Flywheel(7);
ADIDigitalOut Robot::EXPANSION(1);

// Imu Robot::IMU(9);
// ADIEncoder Robot::xEncoder(1, 2, false);
// ADIEncoder Robot::yEncoder(3, 4, false);
// Above definitions define controller, motors, encoders, and IMU
lsd_koyori Robot::rotationPID(0.3, 0.5, 0, 100);
lsd_koyori Robot::travelPID(0.2, 0.5, 0, 100);
lsd_koyori Robot::flywheelPID(0.5, 0.8, 0.9, 100);
// lsd_koyori Robot::strafePID(0.5, 0.1, 0, 100);

// flywheel piss
// void Robot::pisswheel() {
//   double kp = 0.87;
//   int curVel = Flywheel.get_actual_velocity();
//   int vel = Flywheel.get_target_velocity();
//   int error = vel - curVel;

// //   Flywheel.spin(fwd, error * kp, volt);
// // }
// }


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
};
void Robot::ckEXPAND()
{
    if (dom.get_digital(DIGITAL_DOWN) and dom.get_digital(DIGITAL_B))
    {
        EXPANSION.set_value(1);
    }
    if (dom.get_digital(DIGITAL_UP) and dom.get_digital(DIGITAL_X))
    {
        EXPANSION.set_value(0);
    }
}

void Robot::Driver()
{
    // give the driver time to react lol
    delay(500);
    // preset the values to false so we don't start off running lol
    bool flywheelSpinning = false;
    bool intakeEating = false;
    int time = 0;
    while (true) {
        time ++;
        // THIS PART CALLS THE MOVE FUNC WHICH WE DON'T HAVE IMU FOR YET
        // FIX THIS ONCE WE HAVE A WORKING IMU
        // For tank drive, we just get the values of the left and right joysticks vertical lol;
        int Lp = dom.get_analog(ANALOG_LEFT_Y);
        int Rp = dom.get_analog(ANALOG_RIGHT_Y);
        // Intake and Flywheel button check
        bool intakeIn = dom.get_digital(DIGITAL_R2);
        bool intakeOut = dom.get_digital(DIGITAL_R1);
        bool flywheelShoot = dom.get_digital(DIGITAL_L2);
        bool flywheelSuck = dom.get_digital(DIGITAL_L1);
        bool EXPAND = (dom.get_digital(DIGITAL_DOWN) and dom.get_digital(DIGITAL_B));
        
        L1.move(Lp);
        L2.move(Lp);
        L3.move(Lp);
        R1.move(Rp);
        R2.move(Rp);
        R3.move(Rp);
        if (intakeIn) 
        {
            IntakeRoller = 122;
        }
        else if (intakeOut) 
        {
            IntakeRoller = -122;
        }
        else 
        {
            IntakeRoller = 0;
        };
        if (flywheelShoot) 
        {
            Flywheel.move_velocity(36000);
            // Flywheel = flywheelPID.get_spd();
        }
        else if (flywheelSuck)
        {
            Flywheel.move_velocity(36000);
        }
        else
        {
            Flywheel = 0;
        };
        ckEXPAND();
        delay(5);



    }
};


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
