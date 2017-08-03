/*
Car4W library

2017 Talos Electronics
Autor: César Vargas
*/

#include "MotorCD.h"
#include "Car4W.h"

Car4W::~Car4W()
{

}

void Car4W::Setup()
{
	MotorFrontLeft.Setup();
	MotorFrontRight.Setup(); 
	MotorRearLeft.Setup(); 	
	MotorRearRight.Setup(); 	
}

void Car4W::Forward(int SpeedMotorRearLeft, int SpeedMotorRearRight, int SpeedMotorFrontLeft, int SpeedMotorFrontRight)
{
	MotorFrontLeft.Forward(SpeedMotorFrontLeft);
	MotorFrontRight.Forward(SpeedMotorFrontRight);
	MotorRearLeft.Forward(SpeedMotorRearLeft);
	MotorRearRight.Forward(SpeedMotorRearRight);
}

void Car4W::Backward(int SpeedMotorRearLeft, int SpeedMotorRearRight, int SpeedMotorFrontLeft, int SpeedMotorFrontRight)
{
	MotorFrontLeft.Backward(SpeedMotorFrontLeft);
	MotorFrontRight.Backward(SpeedMotorFrontRight);
	MotorRearLeft.Backward(SpeedMotorRearLeft);
	MotorRearRight.Backward(SpeedMotorRearRight);
}

void Car4W::Left(int SpeedMotorRearLeft, int SpeedMotorRearRight, int SpeedMotorFrontLeft, int SpeedMotorFrontRight)
{
	MotorFrontLeft.Backward(SpeedMotorFrontLeft);
	MotorFrontRight.Forward(SpeedMotorFrontRight);
	MotorRearLeft.Backward(SpeedMotorRearLeft);
	MotorRearRight.Forward(SpeedMotorRearRight);
}

void Car4W::Right(int SpeedMotorRearLeft, int SpeedMotorRearRight, int SpeedMotorFrontLeft, int SpeedMotorFrontRight)
{
	MotorFrontLeft.Forward(SpeedMotorFrontLeft);
	MotorFrontRight.Backward(SpeedMotorFrontRight);
	MotorRearLeft.Forward(SpeedMotorRearLeft);
	MotorRearRight.Backward(SpeedMotorRearRight);
}

void Car4W::Stop()
{
	MotorFrontLeft.Stop();
	MotorFrontRight.Stop();
	MotorRearLeft.Stop();
	MotorRearRight.Stop();
}

