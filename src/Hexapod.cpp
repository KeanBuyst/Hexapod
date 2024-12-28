#include "Components.h"
#include <iostream>
#include <cmath>

using namespace Kinematics;

Components::Hexapod::Hexapod(K3* R1,K3* R2,K3* R3,K3* L1,K3* L2,K3* L3) : R1(R1), R2(R2), R3(R3), L1(L1), L2(L2), L3(L3)
{    
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

void Components::Hexapod::setRotation(float radians) 
{ 
    rotation = radians;
}
void Components::Hexapod::setSpeed(float speed) 
{
    this->speed = speed;
}

void Components::Hexapod::interupts()
{
    if (standing){
        _stand();
        return;
    }
    if (sitting){
        _sit();
        return;
    }
}

void Components::Hexapod::move()
{
    static constexpr float shift = 55 * Deg2Rad;
    //static constexpr float r_shift = (M_PI * 2) - (55 * Deg2Rad);

    float x,z;
    // up
    R2->setGoal(Vector(0,height,0));

    x = radius * cosf(-shift);
    z = radius * sinf(-shift);

    L1->setGoal(Vector(-x,height,-z)); // negative due to left legs being mirrored of the right legs.

    x = radius * cosf(shift);
    z = radius * sinf(shift);

    L3->setGoal(Vector(-x,height,-z));
    // up

    // pull
    x = radius * cosf(rotation);
    z = radius * sinf(rotation);

    L2->setGoal(Vector(x,0,z));

    x = radius * cosf(rotation + shift);
    z = radius * sinf(rotation + shift);

    R1->setGoal(Vector(-x,0,-z));

    x = radius * cosf(rotation - shift);
    z = radius * sinf(rotation - shift);

    R3->setGoal(Vector(-x,0,-z));
    // pull

    compute(); // Action one (move)

    x = radius * cosf(rotation);
    z = radius * sinf(rotation);

    R2->setGoal(Vector(x,0,z));

    x = radius * cosf(rotation - shift);
    z = radius * sinf(rotation - shift);

    L1->setGoal(Vector(-x,0,-z));

    x = radius * cosf(rotation + shift);
    z = radius * sinf(rotation + shift);

    L3->setGoal(Vector(-x,0,-z));

    compute(); // Action two (all legs on ground)
    // move up
    x = radius * cosf(shift);
    z = radius * sinf(shift);

    R1->setGoal(Vector(x,height,z));

    x = radius * cosf(-shift);
    z = radius * sinf(-shift);

    R3->setGoal(Vector(x,height,z));

    L2->setGoal(Vector(0,height,0));
    // move up

    // pull
    x = radius * cosf(rotation);
    z = radius * sinf(rotation);

    R2->setGoal(Vector(-x,0,-z));

    x = radius * cosf(rotation - shift);
    z = radius * sinf(rotation - shift);

    L1->setGoal(Vector(x,0,z));

    x = radius * cosf(rotation + shift);
    z = radius * sinf(rotation + shift);

    L3->setGoal(Vector(x,0,z));
    // pull

    compute(); // Action three (move)

    // move down
    x = radius * cosf(rotation + shift);
    z = radius * sinf(rotation + shift);

    R1->setGoal(Vector(x,0,z));

    x = radius * cosf(rotation - shift);
    z = radius * sinf(rotation - shift);

    R3->setGoal(Vector(x,0,z));

    x = radius * cosf(rotation);
    z = radius * sinf(rotation);

    L2->setGoal(Vector(-x,0,-z));
    // move down

    compute(); // Action four (all legs on ground)
}

void Components::Hexapod::compute()
{
    bool condition;
    do {
        bool
        r1 = R1->next(), l1 = L1->next(),
        r2 = R2->next(), l2 = L2->next(),
        r3 = R3->next(), l3 = L3->next();

        condition = r1 || r2 || r3 || l1 || l2 || l3;

        gpioSleep(PI_TIME_RELATIVE,0,time());
    } while (condition);
}

void Components::Hexapod::stand()
{
    standing = true;
}

inline void Components::Hexapod::_stand()
{
    _sit();
    Vector lift(0,0,-50);
    R1->setGoal(lift); L1->setGoal(lift);
    R2->setGoal(lift); L2->setGoal(lift);
    R3->setGoal(lift); L3->setGoal(lift);
    
    bool condition;
    do {
        bool
        r1 = R1->next(), l1 = L1->next(),
        r2 = R2->next(), l2 = L2->next(),
        r3 = R3->next(), l3 = L3->next();

        condition = r1 || r2 || r3 || l1 || l2 || l3;

        gpioSleep(PI_TIME_RELATIVE,0,20000);
    } while (condition);

    standing = false;
}

void Components::Hexapod::sit()
{
    sitting = true;
}

inline void Components::Hexapod::_sit()
{
    Vector position(0,100,-50);
    R1->setGoal(position); L1->setGoal(position);
    R2->setGoal(position); L2->setGoal(position);
    R3->setGoal(position); L3->setGoal(position);

    bool condition;
    do {
        bool
        r1 = R1->next(), l1 = L1->next(),
        r2 = R2->next(), l2 = L2->next(),
        r3 = R3->next(), l3 = L3->next();

        condition = r1 || r2 || r3 || l1 || l2 || l3;

        gpioSleep(PI_TIME_RELATIVE,0,15000);
    } while (condition);

    sitting = false;
}