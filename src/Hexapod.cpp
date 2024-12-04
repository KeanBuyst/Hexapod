#include "Components.h"
#include <iostream>
#include <cmath>

using namespace Kinematics;

Components::Hexapod::Hexapod(K3* R1,K3* R2,K3* R3) : R1(R1), R2(R2), R3(R3)
{
    // default positions
    R1->toPoint(origin.x,origin.y,origin.z);
    R2->toPoint(origin.x,origin.y + height,origin.z);
    R3->toPoint(origin.x,origin.y,origin.z);
    gpioSleep(PI_TIME_RELATIVE,0,20000);
    
    std::cout << "Hexapod ready" << std::endl;
}

constexpr int Components::Hexapod::time(){
    int time = static_cast<int>((max_speed - speed) * 10000);
    if (time < 0)
    {
        std::cerr << "Speed exceeds maximum speed\nsetting speed to max_speed" << std::endl;
        time = 10000;
    }
    return time;
}

void Components::Hexapod::move()
{
    static constexpr float l_shift = 55 * Deg2Rad;
    static constexpr float r_shift = (M_PI * 2) - (55 * Deg2Rad);

    float x,z;

    x = radius * cosf(rotation);
    z = radius * sinf(rotation);

    R2->setGoal(Vector(x,0,z));
    // TODO set goals for L1 and L3
    while (R2->next())
    {
        gpioSleep(PI_TIME_RELATIVE,0,time());
    }

    R2->setGoal(Vector(-x,0,-z));

    x = radius * cos(l_shift);
    z = radius * sin(l_shift);

    R1->setGoal(Vector(x,height,z));

    x = radius * cos(r_shift);
    z = radius * sin(r_shift);

    R3->setGoal(Vector(x,height,z));

    while (R1->next() || R2->next() || R3->next())
    {
        gpioSleep(PI_TIME_RELATIVE,0,time());
    }

    x = radius * cos(rotation + l_shift);
    z = radius * sin(rotation + l_shift);

    R1->setGoal(Vector(x,0,z));

    x = radius * cos(rotation + r_shift);
    z = radius * sin(rotation + r_shift);

    R3->setGoal(Vector(x,0,z));

    while (R1->next() || R3->next())
    {
        gpioSleep(PI_TIME_RELATIVE,0,time());
    }

    x = radius * cos(rotation + l_shift);
    z = radius * sin(rotation + l_shift);

    R1->setGoal(Vector(-x,0,-z));

    x = radius * cos(rotation + r_shift);
    z = radius * sin(rotation + r_shift);

    R3->setGoal(Vector(-x,0,-z));

    R2->setGoal(Vector(0,height,0));

    while (R2->next() || R1->next() || R3->next())
    {
        gpioSleep(PI_TIME_RELATIVE,0,time());
    }
}