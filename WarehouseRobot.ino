
#include <Arduino_FreeRTOS.h>
#include "Car4W.h"
#include "TCRT5000.h"
#include "MotorCD.h"

void TaskLineFollower(void *pvParameters);

TCRT5000 Sensor1(31);
TCRT5000 Sensor2(29);
TCRT5000 Sensor3(27);
TCRT5000 Sensor4(25);
TCRT5000 Sensor5(23);               

MotorCD MotorFrontLeft  ( 4 ,  3 ,  2);   // Object for Motor Front Left
MotorCD MotorFrontRight ( 6 ,  5 ,  7);   // Object for Motor Front Right 
MotorCD MotorRearLeft   (10 ,  9 ,  8);   // Object for Motor Rear Left
MotorCD MotorRearRight  (12 , 11 , 13);   // Object for Motor Rear Right

int nSpeedMotorFrontLeft  = 0;
int nSpeedMotorRearLeft   = 0;
int nSpeedMotorFrontRight = 0;
int nSpeedMotorRearRight  = 0;

bool  bSensor[5]  = {false, false, false, false, false};
int nSpeed = 255;

//////////////////////
//  Variables PID   //
//////////////////////

int SamplingTime = 1;        // tiempo de muestreo Se encuentra en milisegundos
unsigned long Past = 0;       // tiempo Past (Se hace para asegurar tiempo de muestreo)
unsigned long Now;
float Ref = 0;                  // referencia 
double Y = 0;                   // Salida
double Error;                   // Error
double SumError = 0;           // Suma del Error para la parte integral
double LastError = 0;            // Error anterior para la derivada
double DerivateError = 0;

double U = 0;                   // Señal control

int TimeChange = 0;

float P = 0;                    // control proporcional
float I = 0;                    // control Integral
float D = 0;
      
// constantes del controlador 
float Kp = 40;
float Ki = 4;
float Kd = 40;

void setup() 
{  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  Sensor1.Setup();
  Sensor2.Setup();
  Sensor3.Setup();
  Sensor4.Setup();
  Sensor5.Setup();

  MotorFrontLeft.Setup();     
  MotorFrontRight.Setup();    
  MotorRearLeft.Setup();      
  MotorRearRight.Setup();     

  xTaskCreate(
    TaskLineFollower
    ,  (const portCHAR *) "LineFollower"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

void TaskLineFollower(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    bSensor[0] = Sensor1.getDigitalValue();
    bSensor[1] = Sensor2.getDigitalValue();
    bSensor[2] = Sensor3.getDigitalValue();
    bSensor[3] = Sensor4.getDigitalValue();
    bSensor[4] = Sensor5.getDigitalValue();
    
    if     ( (bSensor[0] == 0) && (bSensor[1] == 0) && (bSensor[2] == 0) && (bSensor[3] == 0) && (bSensor[4] == 1) )    Error =  4;
    else if( (bSensor[0] == 0) && (bSensor[1] == 0) && (bSensor[2] == 0) && (bSensor[3] == 1) && (bSensor[4] == 1) )    Error =  3;
    else if( (bSensor[0] == 0) && (bSensor[1] == 0) && (bSensor[2] == 0) && (bSensor[3] == 1) && (bSensor[4] == 0) )    Error =  2;
    else if( (bSensor[0] == 0) && (bSensor[1] == 0) && (bSensor[2] == 1) && (bSensor[3] == 1) && (bSensor[4] == 0) )    Error =  1;
    else if( (bSensor[0] == 0) && (bSensor[1] == 0) && (bSensor[2] == 1) && (bSensor[3] == 0) && (bSensor[4] == 0) )    Error =  0;
    else if( (bSensor[0] == 0) && (bSensor[1] == 1) && (bSensor[2] == 1) && (bSensor[3] == 0) && (bSensor[4] == 0) )    Error = -1;
    else if( (bSensor[0] == 0) && (bSensor[1] == 1) && (bSensor[2] == 0) && (bSensor[3] == 0) && (bSensor[4] == 0) )    Error = -2;
    else if( (bSensor[0] == 1) && (bSensor[1] == 1) && (bSensor[2] == 0) && (bSensor[3] == 0) && (bSensor[4] == 0) )    Error = -3;
    else if( (bSensor[0] == 1) && (bSensor[1] == 0) && (bSensor[2] == 0) && (bSensor[3] == 0) && (bSensor[4] == 0) )    Error = -4;
  
    Now = millis();
  
    TimeChange = Now - Past;                        // Diferencia de tiempo actual- Past
    
    if(TimeChange >= SamplingTime)                    // si se supera el tiempo de muestreo
    {
      SumError = SumError + (Error * SamplingTime);   // Cálculo de aproximación del area
      DerivateError    = (Error - LastError) / SamplingTime;    // Cálculo Error Derivativo 
        
      P = Kp * Error;                                     // Control Proporcional
      I = Ki * SumError;                                 // Control Integral
      D = Kd * DerivateError;                                    // Control Derivativo
        
      U = P + I + D;                                      // Señal de control
        
      Past = Now;                                     // actualizar tiempo
      LastError = Error;                                   // actualizar el Error     
    }  
  
    nSpeedMotorFrontLeft   = nSpeed - U;
    nSpeedMotorRearLeft    = nSpeed - U;
    nSpeedMotorFrontRight  = nSpeed + U;
    nSpeedMotorRearRight   = nSpeed + U;
  
    if(nSpeedMotorFrontLeft   >  255)   nSpeedMotorFrontLeft  =  255;
    if(nSpeedMotorFrontLeft   < -255)   nSpeedMotorFrontLeft  = -255;
    if(nSpeedMotorRearLeft    >  255)   nSpeedMotorRearLeft   =  255;
    if(nSpeedMotorRearLeft    < -255)   nSpeedMotorRearLeft   = -255;
    if(nSpeedMotorFrontRight  >  255)   nSpeedMotorFrontRight =  255;
    if(nSpeedMotorFrontRight  < -255)   nSpeedMotorFrontRight = -255;
    if(nSpeedMotorRearRight   >  255)   nSpeedMotorRearRight  =  255;
    if(nSpeedMotorRearRight   < -255)   nSpeedMotorRearRight  = -255;
      
    if(nSpeedMotorFrontLeft >= 0)   MotorFrontLeft.Forward(nSpeedMotorFrontLeft);
    else                            MotorFrontLeft.Backward(-1*nSpeedMotorFrontLeft);       
  
    if(nSpeedMotorRearLeft >= 0)    MotorRearLeft.Forward(nSpeedMotorRearLeft);
    else                            MotorRearLeft.Backward(-1*nSpeedMotorRearLeft);     
  
    if(nSpeedMotorFrontRight >= 0)  MotorFrontRight.Forward(nSpeedMotorFrontRight);
    else                            MotorFrontRight.Backward(-1*nSpeedMotorFrontRight);     
  
    if(nSpeedMotorRearRight >= 0)   MotorRearRight.Forward(nSpeedMotorRearRight);
    else                            MotorRearRight.Backward(-1*nSpeedMotorRearRight);     
  
    Serial.print(bSensor[0]);
    Serial.print(bSensor[1]);
    Serial.print(bSensor[2]);
    Serial.print(bSensor[3]);
    Serial.print(bSensor[4]);
    Serial.print("  EP:");
    Serial.print(SumError);
    Serial.print("  ED:");
    Serial.print(DerivateError);
    Serial.print("  EA:");
    Serial.print(LastError);
    Serial.print("  P:");
    Serial.print(P);
    Serial.print("  I:");
    Serial.print(I);
    Serial.print("  D:");
    Serial.print(D);
    Serial.print("  U:");
    Serial.print(U);
    Serial.print("  MFL:");
    Serial.print(nSpeedMotorFrontLeft);  
    Serial.print("  MRL:");
    Serial.print(nSpeedMotorRearLeft);
    Serial.print("  MFR:");
    Serial.print(nSpeedMotorFrontRight);
    Serial.print("  MRR:");
    Serial.println(nSpeedMotorRearRight);
    
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

