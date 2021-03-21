#ifndef __MOTOR
#define __MOTOR
#include "wiringPi.h"
#include "softPwm.h"

class Motor
{
    public:
        const int front;
        const int back;
        const int en;
        int duty_cycle;

        Motor(const int front, const int back, const int en);
        int calcDutyCycle(double x_val);
        void init(const int front, const int back);
        void move(bool is_F, int dutyCycle);
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

void move(bool is_F, int dutyCycle)
{
    if (is_F){
        digitalWrite(Motor::front, HIGH);
        digitalWrite(Motor::back, LOW);
    }
    else {
        digitalWrite(Motor::front, LOW);
        digitalWrite(Motor::back, HIGH);
    }
    softPwmWrite (Motor::en, dutyCycle);
}

void stop(){
    digitalWrite(Motor::front, LOW);
    digitalWrite(Motor::back, LOW);
    softPwmWrite(Motor::en, 0);
}

#endif