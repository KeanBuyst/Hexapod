#include <Components.h>

#include <iostream>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <csignal>

using namespace Components;
using namespace Kinematics;

bool moving = false;

int main()
{
    bool running = true;

    gpioInitialise();
    {
        // (Left/Right) (1-3) (Base/Tibia/Femor)
        Servo R2B(25);
        Servo R2F(8);
        Servo R2T(7);

        K3 R2(&R2B,&R2F,&R2T);

        Servo R1B(18);
        Servo R1F(23);
        Servo R1T(24);

        K3 R1(&R1B,&R1F,&R1T);

        Servo R3B(12,-10);
        Servo R3F(16);
        Servo R3T(20);

        K3 R3(&R3B,&R3F,&R3T);
        
        Controller controller(0);

        Hexapod hexapod(&R1,&R2,&R3);

        controller.event(LEFT_JOYSTICK, [&hexapod](const AxisInput& event) {
            hexapod.setRotation(event.radians < 0 ? event.radians + (M_PI * 2) : event.radians);
        });

        controller.event(RIGHT_TRIGGER, [&hexapod](const AxisInput& event){
            moving = event.precentage != 0;
            hexapod.setSpeed(event.precentage);
        });
        
        while (running)
        {
            while (moving)
            {
                hexapod.move();
            }
            gpioSleep(PI_TIME_RELATIVE,0,100); // decrease CPU load
        }
    }
    gpioTerminate();
}