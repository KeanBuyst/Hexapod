#include <Components.h>

#include <iostream>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <csignal>

using namespace Components;
using namespace Kinematics;

bool moving = false;
bool event = false;

int main()
{
    bool running = true;

    gpioInitialise();
    {
        // (Left/Right) (1-3) (Base/Tibia/Femor)
        Servo R2B(25,-10);
        Servo R2F(8);
        Servo R2T(7);

        K3 R2(&R2B,&R2F,&R2T);

        Servo R1B(18);
        Servo R1F(23);
        Servo R1T(24);

        K3 R1(&R1B,&R1F,&R1T);

        Servo R3B(12);
        Servo R3F(16);
        Servo R3T(20); // Yep

        K3 R3(&R3B,&R3F,&R3T);

        Servo L1B(4);
        Servo L1F(17);
        Servo L1T(27);

        K3 L1(&L1B,&L1F,&L1T);
        
        Servo L2B(22);
        Servo L2F(26);
        Servo L2T(9);

        K3 L2(&L2B,&L2F,&L2T);
        
        Servo L3B(5);
        Servo L3F(6);
        Servo L3T(13);

        K3 L3(&L3B,&L3F,&L3T);

        Controller controller(0);

        Hexapod hexapod(&R1,&R2,&R3,&L1,&L2,&L3);

        controller.event(LEFT_JOYSTICK, [&hexapod](const AxisInput& event) {
            hexapod.setRotation(event.radians < 0 ? event.radians + (M_PI * 2) : event.radians);
        });

        controller.event(RIGHT_TRIGGER, [&hexapod](const AxisInput& event){
            moving = event.precentage != 0;
            hexapod.setSpeed(event.precentage);
        });

        controller.event(START, [&hexapod](const Input& event){
            // startup procedure (Stand up)
            if (event.released) return;
            hexapod.stand();
            std::cout << "Starting standing procedure" << std::endl;
        });

        controller.event(BACK,  [&hexapod](const Input& event){
            // stop procedure (sit)
            if (event.released) return;
            hexapod.sit();
            std::cout << "Starting sitting procedure" << std::endl;
        });
        
        while (running)
        {
            hexapod.interupts();
            while (moving)
            {
                hexapod.move();
            }
            gpioSleep(PI_TIME_RELATIVE,0,1); // decrease CPU load
        }
    }
    gpioTerminate();
}