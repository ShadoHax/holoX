#include "main.h"
#include "robo-dark.hpp"

using namespace pros;

void intialize()
{
    lcd::initialize();
    lcd::set_background_color(255, 255, 255);
    lcd::set_text_color(0, 0, 0);
    lcd::set_text(1, "POST");
    Robot::motorInit();
    Robot::reset();
}