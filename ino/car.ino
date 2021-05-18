#include <DynamixelSerial.h>
#define M_PI 3.14159265358979323846
#define M_PI_2 1.57079632679489661923

int CAMBIO_VEL_LIN = 30;
int CAMBIO_VEL_ANG = 3;
int OFFSET_L_FRONT = 650;
int OFFSET_R_FRONT = 0; //encontrar experimentalmente
int OFFSET_L_REAR = 450;
int OFFSET_R_REAR = 0; //encontrar experimentalmente

/**
 * Definición de los motores de dirección
*/
int dir_left_front = 11;
int dir_right_front = 17;
int dir_left_rear = 13;
int dir_right_rear = 14;

/**
 * Definición de los motores de las ruedas
*/
int wheel_left_front = 1;
int wheel_right_front = 2;
int wheel_left_rear = 10;
int wheel_right_rear = 3;

/**
 * Velocidades angular y lineal deseadas
*/
float ang_vel_des = 0;
float lin_vel_des = 0;
/**
 * Velocidad que debe tomar cada motor de rueda
*/
float vel_left_front = 0;
float vel_right_front = 0;
float vel_left_rear = 0;
float vel_right_rear = 0;

/**
 * Velocidad que debe tomar cada motor de dirección
*/
float front_left_steering = 0;
float front_right_steering = 0;
float rear_left_steering = 0;
float rear_right_steering = 0;

/**
 * Medidas tomadas del vehículo
*/
double track_ = 27;
double radius_ = 3;
double base_ = 10;

void setup()
{
    Dynamixel.setSerial(&Serial1);
    Dynamixel.begin(1000000, 2);
    Serial.begin(115200);
    Serial3.begin(9600);
    delay(1000);
}
/**
 * Calcula la velocidad que debe tener cada rueda
*/
void computeVel()
{
    vel_left_front = copysign(1.0, lin_vel_des) * sqrt((pow(lin_vel_des - ang_vel_des * track_ / 2, 2) + pow(base_ * ang_vel_des / 2.0, 2))) / radius_;
    vel_right_front = copysign(1.0, lin_vel_des) * sqrt((pow(lin_vel_des + ang_vel_des * track_ / 2, 2) + pow(base_ * ang_vel_des / 2.0, 2))) / radius_;
    vel_left_rear = copysign(1.0, lin_vel_des) * sqrt((pow(lin_vel_des - ang_vel_des * track_ / 2, 2) + pow(base_ * ang_vel_des / 2.0, 2))) / radius_;
    vel_right_rear = copysign(1.0, lin_vel_des) * sqrt((pow(lin_vel_des + ang_vel_des * track_ / 2, 2) + pow(base_ * ang_vel_des / 2.0, 2))) / radius_;
}

/**
 * Calcula el ángulo de cada joint
*/
void computeAng()
{
    front_left_steering = atan2(ang_vel_des * base_, (2.0 * lin_vel_des - ang_vel_des * track_));
    front_right_steering = atan2(ang_vel_des * base_, (2.0 * lin_vel_des + ang_vel_des * track_));
    rear_left_steering = -atan2(ang_vel_des * base_, (2.0 * lin_vel_des - ang_vel_des * track_));
    rear_right_steering = -atan2(ang_vel_des * base_, (2.0 * lin_vel_des + ang_vel_des * track_));
}

/**
 * Calculan el ángulo de cada joint
*/
void computeAngAux()
{
    front_left_steering = copysign(M_PI_2, ang_vel_des);
    front_right_steering = copysign(M_PI_2, ang_vel_des);
    rear_left_steering = copysign(M_PI_2, -ang_vel_des);
    rear_right_steering = copysign(M_PI_2, -ang_vel_des);
}

/**
 * Retorna el sentido de giro de la rueda
 * Toma de parámetro la velocidad
*/
int selectSide(float vel)
{
    return vel < 0;
}

/**
 * Pone en movimiento cada una de las ruedas
*/
void runCar()
{
    Dynamixel.setEndless(wheel_left_front, ON);
    Dynamixel.setEndless(wheel_right_front, ON);
    Dynamixel.setEndless(wheel_left_rear, ON);
    Dynamixel.setEndless(wheel_right_rear, ON);

    int temp_vel_1 = map(abs(vel_left_front), 0, 50, 0, 1023);
    int temp_vel_2 = map(abs(vel_right_front), 0, 50, 0, 1023);
    int temp_vel_3 = map(abs(vel_left_rear), 0, 50, 0, 1023);
    int temp_vel_4 = map(abs(vel_right_rear), 0, 50, 0, 1023);

    Dynamixel.turn(wheel_left_front, selectSide(vel_left_front), temp_vel_1);
    Dynamixel.turn(wheel_right_front, selectSide(vel_right_front), temp_vel_2);
    Dynamixel.turn(wheel_left_rear, selectSide(vel_left_rear), temp_vel_3);
    Dynamixel.turn(wheel_right_rear, selectSide(vel_right_rear), temp_vel_4);

}

/**
 * Gira cada joint a la posición deseada
*/
void turn(){

    int temp_ang_1 = map(front_left_steering * RAD_TO_DEG, -90, 90, 0, 306);
    int temp_ang_2 = map(front_right_steering * RAD_TO_DEG, -90, 90, 0, 306);
    int temp_ang_3 = map(rear_left_steering * RAD_TO_DEG, -90, 90, 0, 306);
    int temp_ang_4 = map(rear_right_steering * RAD_TO_DEG, -90, 90, 0, 306);

    if (front_left_steering  < 0) temp_ang_1 *= -1;
    if (front_right_steering < 0) temp_ang_2 *= -1;
    if (rear_left_steering   < 0) temp_ang_3 *= -1;
    if (rear_right_steering  < 0) temp_ang_4 *= -1;

    /*
    *Comentar esta líne para hacer pruebas del sentido de giro*
    */
    Dynamixel.move(dir_left_front, temp_ang_1 + OFFSET_L_FRONT - 153); 
    Dynamixel.move(dir_right_front, temp_ang_2 + OFFSET_R_FRONT - 153); 
    Dynamixel.move(dir_left_rear, temp_ang_3 + OFFSET_L_REAR - 153); 
    Dynamixel.move(dir_right_rear, temp_ang_4 + OFFSET_R_REAR - 153); 
    
    /*
    *Para observar mejor en las peruabs de sentido de giro*
    */
    // Dynamixel.move(dir_left_front, 0);
    // Dynamixel.move(dir_right_front, 0);
    // Dynamixel.move(dir_left_rear, 1023);
    // Dynamixel.move(dir_right_rear, 0);

}


/**
 * Deja los joint en posiciión 0
 * Detiene todos los motores
*/
void stopAll()
{
    Dynamixel.setEndless(wheel_left_front, OFF);
    Dynamixel.setEndless(wheel_right_front, OFF);
    Dynamixel.setEndless(wheel_left_rear, OFF);
    Dynamixel.setEndless(wheel_right_rear, OFF);

    Dynamixel.move(dir_left_front, OFFSET_L_FRONT);
    Dynamixel.move(dir_right_front, OFFSET_R_FRONT);
    Dynamixel.move(dir_left_rear, OFFSET_L_REAR);
    Dynamixel.move(dir_right_rear, OFFSET_R_REAR);
    Dynamixel.move(wheel_left_front, 0); 
    Dynamixel.move(wheel_right_front, 0);
    Dynamixel.move(wheel_left_rear, 0);
    Dynamixel.move(wheel_right_rear, 0);
}

void loop()
{
    unsigned char comando = 0;
    if (Serial.available())
    {
        comando = Serial.read();
        if (comando == 'a')
            lin_vel_des += CAMBIO_VEL_LIN;
        else if (comando == 'A')
            lin_vel_des -= CAMBIO_VEL_LIN;
        else if (comando == 'b')
            ang_vel_des += CAMBIO_VEL_ANG;
        else if (comando == 'B')
            ang_vel_des -= CAMBIO_VEL_ANG;
        else if (comando == 's' || comando == 'S')
        {
            lin_vel_des = 0;
            ang_vel_des = 0;
        }
    }

    if (abs(2 * lin_vel_des) > abs(ang_vel_des * track_))
    {
        computeVel();
        runCar();
    }
    if (abs(2 * lin_vel_des) > abs(ang_vel_des * track_))
    {
        computeAng();
        turn();
    }
    else if (abs(lin_vel_des) > 0.001)
    {
        computeAngAux();
        turn();
    }
    else if (lin_vel_des == 0 && ang_vel_des == 0) stopAll();
    delay(500);
}
