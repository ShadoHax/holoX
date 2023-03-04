#include "main.h"
#include "robo-dark.hpp"

using namespace pros;

void intialize()
{
    lcd::initialize();
    Robot::motorInit();
    Robot::reset();
}