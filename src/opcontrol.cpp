#include "main.h"
#include "robo-dark.hpp"

using namespace pros;

void opcontrol() {
    Robot::motorInit();
    Robot::brake("hold");
    Robot::Driver();
};