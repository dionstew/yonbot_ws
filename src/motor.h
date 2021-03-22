#ifndef __MOTOR
#define __MOTOR
#include "wiringPi.h"
class Motor
{
    public:
        int front;
        int back;
        int en;
        int duty_cycle;

        Motor(int front, int back, int en);
        void init(const int front, const int back);
        void backward();
        void forward();
        void stop();
};

// Constructor
Motor::Motor(const int en, const int front, const int back)
{
    Motor::en = en;
    Motor::front = front;
    Motor::back = back; 
}

void init()
{
    wiringPiSetupGpio();
    pinMode(Motor::front, OUTPUT);
    pinMode(Motor::back, OUTPUT);
    pinMode(Motor::en, OUTPUT);
    softPwmCreate(Motor::en, 1, 100);
}

void stop()
{
    digitalWrite(Motor::front, LOW);
    digitalWrite(Motor::back, LOW);
    softPwmWrite(Motor::en, 0);
}

void forward()
{
    digitalWrite(Motor::front, HIGH);
    digitalWrite(Motor::back, LOW);
}

void backward()
{
    digitalWrite(Motor::front, LOW);
    digitalWrite(Motor::back, HIGH);
}

#endif