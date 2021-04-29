#include <DynamixelSerial.h>
#define M_PI_2      1.57079632679489661923
float ang_vel_des = 0;
float lin_vel_des = 0;
float vel_left_front = 0;
float vel_right_front = 0;
float vel_left_rear = 0;
float vel_right_rear = 0;
float front_left_steering = 0;
float front_right_steering = 0;
float rear_left_steering = 0;
float rear_right_steering = 0;

double track_ = 0;  //buscar la medida
double radius_ = 0; //buscar la medida
double base_ = 0;   //buscar la medida

// fabs() - abs()
// copysign() - copysign() preguntar!

void setup()
{
    Dynamixel.setSerial(&Serial1); // &Serial - Arduino UNO/NANO/MICRO, &Serial1, &Serial2, &Serial3 - Arduino Mega
    Dynamixel.begin(1000000, 3);   // Inicialize the servo at 1 Mbps and Pin Control 2
    delay(1000);
}

void loop()
{
    if(abs(2*lin_vel_des) > abs(ang_vel_des*track_)){
        computeVel();
    }
    if(abs(2*lin_vel_des) > abs(ang_vel_des*track_)){
        computeAng();
    }
    else if(abs(lin_vel_des) > 0.001){
        preguntar();
    }

    Dynamixel.move(3, random(200, 800)); // Move the Servo radomly from 200 to 800
    delay(1000);
    Dynamixel.moveSpeed(3, random(200, 800), random(200, 800));
    delay(2000);
    Dynamixel.setEndless(3, ON);
    Dynamixel.turn(3, RIGTH, 1000);
    delay(3000);
    Dynamixel.turn(3, LEFT, 1000);
    delay(3000);
    Dynamixel.setEndless(3, OFF);
    Dynamixel.ledStatus(3, ON);
    Dynamixel.moveRW(3, 512);
    delay(1000);
    Dynamixel.action();
    Dynamixel.ledStatus(3, OFF);
    delay(1000);
}

void computeVel()
{
    vel_left_front = copysign(1.0, lin_vel_des) * sqrt((pow(lin_vel_des - ang_vel_des * track_ / 2, 2) + pow(base_ * ang_vel_des / 2.0, 2))) / radius_;
    vel_right_front = copysign(1.0, lin_vel_des) * sqrt((pow(lin_vel_des + ang_vel_des * track_ / 2, 2) + pow(base_ * ang_vel_des / 2.0, 2))) / radius_;
    vel_left_rear = copysign(1.0, lin_vel_des) * sqrt((pow(lin_vel_des - ang_vel_des * track_ / 2, 2) + pow(base_ * ang_vel_des / 2.0, 2))) / radius_;
    vel_right_rear = copysign(1.0, lin_vel_des) * sqrt((pow(lin_vel_des + ang_vel_des * track_ / 2, 2) + pow(base_ * ang_vel_des / 2.0, 2))) / radius_;
}

void computeAng()
{
    front_left_steering = atan(ang_vel_des*base_ / 2.0*lin_vel_des - ang_vel_des*track_));
    front_right_steering = atan(ang_vel_des * base_ / (2.0 * lin_vel_des + ang_vel_des * track_));
    rear_left_steering = -atan(ang_vel_des * base_ / (2.0 * lin_vel_des - ang_vel_des * track_));
    rear_right_steering = -atan(ang_vel_des * base_ / (2.0 * lin_vel_des + ang_vel_des * track_));
}

void preguntar()
{
    front_left_steering = copysign(M_PI_2, ang_vel_des);
    front_right_steering = copysign(M_PI_2, ang_vel_des);
    rear_left_steering = copysign(M_PI_2, -ang_vel_des);
    rear_right_steering = copysign(M_PI_2, -ang_vel_des);
}