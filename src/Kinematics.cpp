#include "Kinematics.h"
#include <pigpio.h>

#include <math.h>

Kinematics::Vector Kinematics::origin = Kinematics::Vector(0,-80,100);

Kinematics::Vector::Vector() : x(0), y(0), z(0)
{}

Kinematics::Vector::Vector(float x,float y, float z) : x(x),y(y),z(z)
{}

Kinematics::Vector Kinematics::Vector::operator-(const Vector& other) const
{
    return Vector(x - other.x,y - other.y,z - other.z);
}
Kinematics::Vector Kinematics::Vector::operator/(float value) const
{
    return Vector(x / value,y / value,z / value);
}
Kinematics::Vector Kinematics::Vector::operator+(const Vector& other) const
{
    return Vector(x + other.x,y + other.y,z + other.z);
}
void Kinematics::Vector::operator+=(const Vector& other)
{
    x += other.x;
    y += other.y;
    z += other.z;
}
bool Kinematics::Vector::operator<(const Vector &other)
{
    return x < other.x && y < other.y && z < other.z;
}
bool Kinematics::Vector::operator!=(const Vector &other)
{
    return !this->operator==(other);
}
bool Kinematics::Vector::operator==(const Vector &other)
{
    return x == other.x && y == other.y && z == other.z;
}

float Kinematics::Vector::distanceTo(const Kinematics::Vector &other)
{
    return sqrt( pow((x - other.x), 2) + pow((y - other.y),2) + pow((z - other.z), 2) );
}

Kinematics::Vector Kinematics::Vector::round()
{
    return Vector(roundf(x),roundf(y),roundf(z));
}