#pragma once

namespace Kinematics 
{
    class Vector 
    {
        public:
        Vector();
        Vector(float x,float y, float z);

        Vector operator-(const Vector& other) const;
        Vector operator/(float value) const;
        Vector operator+(const Vector& other) const;
        void operator+=(const Vector& other);
        bool operator<(const Vector& other);
        bool operator!=(const Vector& other);
        bool operator==(const Vector& other);

        float distanceTo(const Kinematics::Vector& other);
        Vector round();

        float x,y,z;
    };
    extern Vector origin;
}