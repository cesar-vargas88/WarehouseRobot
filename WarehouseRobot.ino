
#include "Car4W.h"
#include "TCRT5000.h"
                    
Car4W Car4WWarehouse( 4 , 3 , 2 ,       // Motor Front Left
                      6 , 5 , 7 ,       // Motor Front Right
                      10, 9 , 8 ,       // Motor Rear Left 
                      12, 11, 13);      // Motor Rear Right

TCRT5000 Sensor1(31);
TCRT5000 Sensor2(29);
TCRT5000 Sensor3(27);
TCRT5000 Sensor4(25);
TCRT5000 Sensor5(23);                      

int nSpeedMotorFrontLeft  = 0;
int nSpeedMotorRearLeft   = 0;
int nSpeedMotorFrontRight = 0;
int nSpeedMotorRearRight  = 0;

bool  bSensor[5]  = {false, false, false, false, false};
float fError      = 0;
float fLast_Error = 0;
float fLast_I     = 0;
float fPID        = 0;
float fP          = 0;
float fI          = 0;
float fD          = 0;

float fKP         = 12;
float fKI         = 0.25;
float fKD         = 10;
int   nSpeed      = 127;

void setup() 
{
  Car4WWarehouse.Setup();

  Sensor1.Setup();
  Sensor2.Setup();
  Sensor3.Setup();
  Sensor4.Setup();
  Sensor5.Setup();
  
  Serial.begin(9600);
}

void loop() 
{
  bSensor[0] = Sensor1.getDigitalValue();
  bSensor[1] = Sensor2.getDigitalValue();
  bSensor[2] = Sensor3.getDigitalValue();
  bSensor[3] = Sensor4.getDigitalValue();
  bSensor[4] = Sensor5.getDigitalValue();

  if( (bSensor[0] == 0) && (bSensor[1] == 0) && (bSensor[2] == 0) && (bSensor[3] == 0) && (bSensor[4] == 0) )
  {
    fError = fError;
  }
  else
  {
    if     ( (bSensor[0] == 0) && (bSensor[1] == 0) && (bSensor[2] == 0) && (bSensor[3] == 0) && (bSensor[4] == 1) )
    {
      fError = 4;
    }
    else if( (bSensor[0] == 0) && (bSensor[1] == 0) && (bSensor[2] == 0) && (bSensor[3] == 1) && (bSensor[4] == 1) )
    {
      fError = 3;
    }
    else if( (bSensor[0] == 0) && (bSensor[1] == 0) && (bSensor[2] == 0) && (bSensor[3] == 1) && (bSensor[4] == 0) )
    {
      fError = 2;
    }
    else if( (bSensor[0] == 0) && (bSensor[1] == 0) && (bSensor[2] == 1) && (bSensor[3] == 1) && (bSensor[4] == 0) )
    {
      fError = 1;
    }
    else if( (bSensor[0] == 0) && (bSensor[1] == 0) && (bSensor[2] == 1) && (bSensor[3] == 0) && (bSensor[4] == 0) )
    {
      fError = 0;
    }
    else if( (bSensor[0] == 0) && (bSensor[1] == 1) && (bSensor[2] == 1) && (bSensor[3] == 0) && (bSensor[4] == 0) )
    {
      fError = -1;
    }
    else if( (bSensor[0] == 0) && (bSensor[1] == 1) && (bSensor[2] == 0) && (bSensor[3] == 0) && (bSensor[4] == 0) )
    {
      fError = -2;
    }
    else if( (bSensor[0] == 1) && (bSensor[1] == 1) && (bSensor[2] == 0) && (bSensor[3] == 0) && (bSensor[4] == 0) )
    {
      fError = -3;
    }
    else if( (bSensor[0] == 1) && (bSensor[1] == 0) && (bSensor[2] == 0) && (bSensor[3] == 0) && (bSensor[4] == 0) )
    {
      fError = -4;
    }
  }

  fP = fError;
  fI = fI + fLast_I;
  fD = fError - fLast_Error;

  fPID = (fKP * fP) + (fKI * fI) + (fKD * fD);

  fLast_I = fI;
  fLast_Error = fError;

  nSpeedMotorFrontLeft   = nSpeed - fPID;
  nSpeedMotorRearLeft    = nSpeed - fPID;
  nSpeedMotorFrontRight  = nSpeed + fPID;
  nSpeedMotorRearRight   = nSpeed + fPID;

  constrain(nSpeedMotorFrontLeft   , 0, 255);
  constrain(nSpeedMotorRearLeft    , 0, 255);
  constrain(nSpeedMotorFrontRight  , 0, 255);
  constrain(nSpeedMotorRearRight   , 0, 255);

  Car4WWarehouse.Forward(nSpeedMotorFrontLeft, nSpeedMotorFrontRight, nSpeedMotorRearLeft, nSpeedMotorRearRight);
  
  Serial.print(bSensor[0]);
  Serial.print(bSensor[1]);
  Serial.print(bSensor[2]);
  Serial.print(bSensor[3]);
  Serial.print(bSensor[4]);
  Serial.print("  Error:");
  Serial.print(fError);
  Serial.print("  LastError:");
  Serial.print(fLast_Error);
  Serial.print("  P:");
  Serial.print(fP);
  Serial.print("  I:");
  Serial.print(fI);
  Serial.print("  D:");
  Serial.print(fD);
  Serial.print("  PID:");
  Serial.print(fPID);
  Serial.print("  Last_I:");
  Serial.print(fLast_I);
  Serial.print("  KP:");
  Serial.print(fKP);
  Serial.print("  P:");
  Serial.print(fP);
  Serial.print("  KI:");
  Serial.print(fKI);
  Serial.print("  fI:");
  Serial.print(fI);
  Serial.print("  KD:");
  Serial.print(fKD);
  Serial.print("  D:");
  Serial.println(fD);
  
  delay(500);
}

  /*Serial.print("fP: ");
  Serial.print(fP);
  Serial.print("\tfI: ");
  Serial.print(fI);
  Serial.print("\tfD: ");
  Serial.print(fD);
  Serial.print("\tfPID: ");
  Serial.print(fPID);
  Serial.print("\tfLast_I: ");
  Serial.print(fLast_I);
  Serial.print("\tfError: ");
  Serial.print(fError);
  Serial.print("\tfLast_Error: ");
  Serial.print(fLast_Error);
  Serial.print("\tKP: ");
  Serial.print(nKP);
  Serial.print("\tfP: ");
  Serial.print(fP);
  Serial.print("\tKI: ");
  Serial.print(nKI);
  Serial.print("\tfI: ");
  Serial.print(fI);
  Serial.print("\tKD: ");
  Serial.print(nKD);
  Serial.println("\tfD: ");
  Serial.println(fD);*/

  /*
  Car4WWarehouse.Forward(nSpeedFrontLeft, nSpeedFrontRight, nSpeedRearLeft, nSpeedRearRight);
  delay(2000);
  Car4WWarehouse.Stop();
  delay(2000);
  
  Car4WWarehouse.Backward(nSpeedFrontLeft, nSpeedFrontRight, nSpeedRearLeft, nSpeedRearRight);
  delay(2000);
  Car4WWarehouse.Stop();
  delay(2000);
  
  Car4WWarehouse.Left(nSpeedFrontLeft, nSpeedFrontRight, nSpeedRearLeft, nSpeedRearRight);
  delay(2000);
  Car4WWarehouse.Stop();
  delay(2000);
  
  Car4WWarehouse.Right(nSpeedFrontLeft, nSpeedFrontRight, nSpeedRearLeft, nSpeedRearRight);
  delay(2000);
  Car4WWarehouse.Stop();
  delay(2000);

  if(Sensor1.getDigitalValue())
    Serial.print("1");
  else
    Serial.print("0");
  
  if(Sensor2.getDigitalValue())
    Serial.print("1");
  else
    Serial.print("0");

  if(Sensor3.getDigitalValue())
    Serial.print("1");
  else
    Serial.print("0");

  if(Sensor4.getDigitalValue())
    Serial.print("1");
  else
    Serial.print("0");

  if(Sensor5.getDigitalValue())
    Serial.println("1");
  else
    Serial.println("0");

  delay(100);
  */
