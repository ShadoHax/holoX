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
Motor Robot::L1(1);
Motor Robot::L2(2);
Motor Robot::L3(3);
Motor Robot::R1(4);
Motor Robot::R2(5);
Motor Robot::R3(6);
Motor Robot::IntakeRoller(7);
Motor Robot::Flywheel(8);
Imu Robot::IMU(9);
// ADIEncoder Robot::LeftEncoder(1, 2, false);
// ADIEncoder Robot::RightEncoder(3, 4, false);
// ADIEncoder Robot::BackEncoder(5, 6, false);
// Above definitions define controller, motors, encoders, and IMU
lsd_koyori Robot::rotationPID(0.3, 0.5, 0, 100);
lsd_koyori Robot::travelPID(0.2, 0.5, 0, 100);
// lsd_koyori Robot::strafePID(0.5, 0.1, 0, 100);


