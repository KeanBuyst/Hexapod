#pragma once

#include "Kinematics.h"
#include <pigpio.h>
#include <math.h>
#include <thread>
#include <queue>
#include <unordered_map>
#include <functional>

typedef unsigned int uint;
constexpr float Rad2Deg = 180 / M_PI;
constexpr float Deg2Rad = M_PI / 180;

namespace Components {

    template<typename Function>
    extern std::queue<Function> sync;

    class Servo
    {
        private:
        uint max;
        uint pin;
        float shift;
        public:
        Servo(uint pin,float shift = 0.0f,uint max = 180);
        ~Servo();
        void setAngle(float angle) const;
    };

    class K3
    {
        public:
        K3(const Servo* s1,const Servo* s2,const Servo* s3);

        void toPoint(float x, float y, float z);

        void setGoal(Kinematics::Vector position);
        bool next();

        void debug();

        const Servo* base;
        const Servo* femor;
        const Servo* tibia;
        private:
        Kinematics::Vector current,end,step;
    };

    enum AXIES : uint8_t
    {
        LEFT_JOYSTICK = 0, // both 0 and 1 (2D axis)
        RIGHT_JOYSTICK = 2, // both 2 and 3 (2D axis)
        LEFT_TRIGGER = 5,
        RIGHT_TRIGGER = 4,
        // Realise = 0 (same for all axies in D-pad) Axis 6 (Left = 32767 Right = -32767) Axis 7 (Up = -32767 Down 32767)
        D_PAD = 6, // both 6 and 7 (2D axis)
    };

    enum BUTTONS : uint8_t
    {
        BUTTON_X = 3,
        BUTTON_Y = 4,
        BUTTON_B = 1,
        BUTTON_A = 0,

        BUTTON_LJ = 14, // left joystick button
        BUTTON_RJ = 13,

        BUTTON_RT = 9, // tigger button. Gets called when trigger is fully pulled in
        BUTTON_LT = 8,

        LB = 6,
        RB = 7,
        L4 = 2,
        R4 = 5,
        BACK = 10,
        START = 11,
        HOME = 12
    };

    class Input 
    {
        public:
        bool released = false;
    };

    class AxisInput : public Input
    {
        public:

        double 
        radians = 0,
        precentage = 0; // from 0 to 1

        short
        raw_x = 0,
        raw_z = 0;

        bool 
        LEFT = false,
        RIGHT = false,
        UP = false,
        DOWN = false;

    };

    struct InputContainer
    {
        uint8_t type;
        bool axis;
        std::function<void(Input&)> lambda;
    };

    class Controller 
    {
        public:
        Controller(uint index);

        void event(AXIES type,std::function<void(AxisInput&)> lambda);
        void event(BUTTONS type,std::function<void(Input&)> lambda);

        int getJS() { return js_device; }
        private:
        std::vector<InputContainer> containers;

        int js_device; // socket
        
        void thread();
    };

    enum Action
    {
        ROTATION_ACTION,
        MOVEMENT_ACTION
    };

    class Hexapod 
    {
        public:
        static constexpr float height = 40;
        static constexpr int radius = 50; // in mm
        static constexpr float max_speed = 1.1f;

        Hexapod(K3* R1,K3* R2,K3* R3);
        void move();
        void rotate();

        void setRotation(float radians) { rotation = radians; }
        void setSpeed(float speed) { this->speed = speed; }

        private:
        K3
        *L1, *R1,
        *L2, *R2,
        *L3, *R3;

        float 
        rotation, // in radians
        speed;

        constexpr int time();
    };
}