#include "Components.h"

#include <linux/joystick.h>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>
#include <iostream>
#include <climits>

const float lenFemor = 78;
const float lenTibia = 100;

Components::Servo::Servo(uint pin,float shift,uint max) : pin(pin),shift(shift),max(max)
{};

Components::Servo::~Servo()
{
    gpioServo(pin,0);
};

void Components::Servo::setAngle(float angle) const
{
    // servo 100% at 12. 0% at 2. While at 50 hertz
    angle += shift;
    if (angle > max || angle < 0)
    {
        std::cerr << "PI-GPIO(" << pin << "): provided angle out of bounds -> Servo::setAngle(" << angle << ")" << std::endl;
        return;
    }
    float dc = (angle/max) * 1900 + 500;
    gpioServo(pin,dc);
}

Components::K3::K3(const Servo* s1,const Servo* s2,const Servo* s3) : current(Kinematics::origin), end(Kinematics::origin)
{
    base = s1;
    femor = s2;
    tibia = s3;
}

void Components::K3::toPoint(float x, float y, float z)
{
    // z is depth (never negative)
    // x is left and right shift
    // y is up and down

    // x and z flipped because servo is upside down
    float pivot = atan2(z,x) * Rad2Deg;
    base->setAngle(pivot);

    float R = sqrt(x * x + z * z); // get point R

    float diagnol = sqrt(R * R + y * y);
    // rule of cosine = ( c^2 = a^2 + b^2 - 2abcos(*) ) c is oppersite side
    float tibiaRot = acos((lenTibia * lenTibia + lenFemor * lenFemor - diagnol * diagnol) / ( 2 * lenTibia * lenFemor )) * Rad2Deg;
    tibia->setAngle(180 - tibiaRot);
    float femorRot = (acos((lenFemor * lenFemor + diagnol * diagnol - lenTibia * lenTibia) / (2 * lenFemor * diagnol)) + atan2(R,-y)) * Rad2Deg;
    femor->setAngle(180 - femorRot);
}

void Components::K3::setGoal(Kinematics::Vector position)
{
    Kinematics::Vector pos = (position + Kinematics::origin).round();
    step = (pos - current) / 50.f;
    end = pos;
}

bool Components::K3::next()
{
    if (current == end) return false;
    if (current.distanceTo(end) < 1.f)
    {
        current = end;
        return false;
    }

    current += step;
    toPoint(current.x,current.y,current.z);

    return true;
}

// sets all servos to 90 degress
void Components::K3::debug()
{
    base->setAngle(90);
    femor->setAngle(90);
    tibia->setAngle(90);
    gpioSleep(PI_TIME_RELATIVE,1,0);
}

void Components::Controller::event(AXIES type,std::function<void(AxisInput&)> lambda)
{
    containers.push_back({
        type,
        true,
        [lambda](Input& input) {
            lambda(static_cast<AxisInput&>(input));
        }
    });
}
void Components::Controller::event(BUTTONS type,std::function<void(Input&)> lambda)
{
    containers.push_back({
        type,
        false,
        lambda
    });
}

void Components::Controller::thread()
{
    AxisInput l_js;
    AxisInput r_js;

    js_event event;

    uint count = 0;
    bool ignore_start = true;

    while (true)
    {
        ssize_t bytes = read(js_device, &event, sizeof(js_event));
        if (bytes < 0)
        {
            std::cerr << "Unable to read from js device" << std::endl;
            exit(-1);
        }
        if (ignore_start) // ingnore start of all buttons/axies resseting
        {
            if (count == 23) ignore_start = false;
            count++;
            continue;
        }
        if (bytes == sizeof(js_event)) {
            // Event type
            switch (event.type & ~JS_EVENT_INIT) {
                case JS_EVENT_BUTTON:
                    if (event.number == HOME) 
                    {
                        std::cout << "controller thread closed" << std::endl;
                        exit(0);
                    }
                    for (InputContainer& container : containers)
                    {
                        if (!container.axis && container.type == event.number)
                        {
                            Input input;
                            input.released = static_cast<bool>(event.value);
                            container.lambda(input);
                        }
                    }
                    break;
                case JS_EVENT_AXIS:
                    for (InputContainer& container : containers)
                    {
                        if (container.axis)
                        {
                            if (container.type == D_PAD)
                            {
                                if (event.number == 6)
                                {
                                    AxisInput input;
                                    if (event.value == 0)
                                    {
                                        input.released == true;
                                    }
                                    else if (event.value == SHRT_MAX)
                                    {
                                        input.LEFT = true;
                                    }
                                    else if (event.value == SHRT_MIN)
                                    {
                                        input.RIGHT = true;
                                    }
                                    container.lambda(input);
                                }
                                else if (event.number == 7)
                                {
                                    AxisInput input;
                                    if (event.value == 0)
                                    {
                                        input.released = true;
                                    }
                                    else if (event.value == SHRT_MAX)
                                    {
                                        input.DOWN = true;
                                    }
                                    else if (event.value == SHRT_MIN)
                                    {
                                        input.UP = true;
                                    }
                                    container.lambda(input);
                                }
                            }
                            else if (container.type == LEFT_JOYSTICK)
                            {
                                if (event.number == 0)
                                {
                                    // x-axies
                                    if (event.value == 0)
                                    {
                                        r_js = AxisInput();
                                        container.lambda(r_js);
                                    }
                                    else 
                                    {
                                        l_js.raw_x = event.value;
                                        l_js.radians = atan2(l_js.raw_x,l_js.raw_z);
                                        container.lambda(l_js);
                                    }
                                }
                                else if (event.number == 1)
                                {
                                    // z-axies
                                    if (event.value == 0)
                                    {
                                        r_js = AxisInput();
                                        container.lambda(r_js);
                                    }
                                    else 
                                    {
                                        l_js.raw_z = -event.value; // flip z
                                        l_js.radians = atan2(l_js.raw_x,l_js.raw_z);
                                        container.lambda(l_js);
                                    }
                                }
                            }
                            else if (container.type == RIGHT_JOYSTICK)
                            {
                                if (event.number == 2)
                                {
                                    // x-axies
                                    if (event.value == 0)
                                    {
                                        r_js = AxisInput();
                                        container.lambda(r_js);
                                    }
                                    else 
                                    {
                                        r_js.raw_x = event.value;
                                        r_js.radians = atan2(r_js.raw_x,r_js.raw_z);
                                        container.lambda(r_js);
                                    }
                                }
                                else if (event.number == 3)
                                {
                                    // z-axies
                                    if (event.value == 0)
                                    {
                                        r_js = AxisInput();
                                        container.lambda(r_js);
                                    }
                                    else 
                                    {
                                        r_js.raw_z = -event.value; // flip z
                                        r_js.radians = atan2(r_js.raw_x,r_js.raw_z);
                                        container.lambda(r_js);
                                    }
                                }
                            }
                            else if (container.type == event.number) // The triggers
                            {
                                AxisInput in;
                                double num = event.value + SHRT_MAX;
                                in.precentage = num / (SHRT_MAX * 2);
                                container.lambda(in);
                            }
                        }
                    }
                    break;
            }
        }
    }

    close(js_device);
}

Components::Controller::Controller(uint index)
{
    std::string device = "/dev/input/js" + std::to_string(index);
    js_device = open(device.c_str(), O_RDONLY);

    if (js_device < 0) 
    {
        std::cerr << "Failed to connect to controller" << std::endl;
        exit(-1);
    }

    std::thread(&Components::Controller::thread,this).detach();
}