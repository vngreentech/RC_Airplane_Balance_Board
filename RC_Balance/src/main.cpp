#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <SimpleKalmanFilter.h>
#include "VERSION.h"

// #define CHECK_COMPARE_V1
#define CHECK_COMPARE_V2
// #define DEBUG

#define LED (13)
#define PIN_1 (6)
#define PIN_2 (7)
#define SERVO_1 (8)
#define SERVO_2 (9)
#define LEDON (digitalWrite(LED,HIGH))
#define LEDOF (digitalWrite(LED,LOW))
#define RCPin1 (2)
#define RCPin2 (3)
#define LimitMax  (0.2)
#define LimitMin (-0.2)
#define SupportValue (5)

struct Orientation 
{
  double Yaw;
  double Pitch;
  double Roll;
  bool Error;
};
struct Orientation prevOrientation;
struct Orientation Angle;

MPU6050 mpu;
Servo myservo1;
Servo myservo2;

Quaternion q;           
VectorFloat gravity;    
float ypr[3];           

bool dmpReady = false; 
uint8_t mpuIntStatus;   
uint16_t packetSize;
uint8_t fifoBuffer[64];
int ServoControl_1=0, ServoValue_1=0, Servo_1_LastValue=0;
int ServoControl_2=0, ServoValue_2=0, Servo_2_LastValue=0; 
volatile long Time_Star1 = 0;
volatile long Pulse1 = 0;
int CheckPulse1 = 0;
volatile long Time_Star2 = 0;
volatile long Pulse2 = 0;
int CheckPulse2 = 0;

int Pin_1_Read;

SimpleKalmanFilter Kalman_1(2, 1, 0.1);
SimpleKalmanFilter Kalman_2(2, 1, 0.1);

struct Orientation getIMUOrientation() 
{
  if (!dmpReady || !mpu.testConnection()) 
  {
    struct Orientation o;
    o.Yaw = 0;
    o.Pitch = 0;
    o.Roll = 0;
    o.Error = true;
    return o;
  }

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    struct Orientation o;
    o.Yaw = double( float(ypr[0] * float(180 / M_PI)) );
    o.Pitch = double( float(ypr[2] * float(180 / M_PI)) );
    o.Roll = double( float(ypr[1] * float(180 / M_PI)) );
    o.Error = false;

    prevOrientation.Yaw = o.Yaw;
    prevOrientation.Pitch = o.Pitch;
    prevOrientation.Roll = o.Roll;

    return o;
  } 
  else {return prevOrientation;}
}

void ReadPulse_1(void)
{
  if (micros() >= Time_Star1)
  {
    Pulse1 = micros() - Time_Star1;
    Time_Star1 = micros();
  }
}
void ReadPulse_2(void)
{
  if (micros() >= Time_Star2)
  {
    Pulse2 = micros() - Time_Star2;
    Time_Star2 = micros();
  }
}

void setup() 
{
  #ifdef DEBUG
  Serial.begin(115200);
  #endif /*DEBUG*/

  Wire.begin();
  Wire.setClock(400000);

  pinMode(LED,OUTPUT);

  pinMode(PIN_1, INPUT);

  pinMode(RCPin1, INPUT_PULLUP);
  pinMode(RCPin2, INPUT_PULLUP);
  myservo1.attach(SERVO_1);
  myservo2.attach(SERVO_2);
  attachInterrupt(digitalPinToInterrupt(RCPin1),ReadPulse_1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RCPin2),ReadPulse_2,CHANGE);  

  while(mpu.dmpInitialize()!=0)
  {
    digitalWrite(LED,!digitalRead(LED));
    delay(50);
  }

  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();  

  /*
  MPU6050_GYRO_FS_250  
  MPU6050_GYRO_FS_500  
  MPU6050_GYRO_FS_1000 
  MPU6050_GYRO_FS_2000 
  */
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  
  /*
  MPU6050_DLPF_BW_256
  MPU6050_DLPF_BW_188
  MPU6050_DLPF_BW_98 
  MPU6050_DLPF_BW_42 
  MPU6050_DLPF_BW_20 
  MPU6050_DLPF_BW_10 
  MPU6050_DLPF_BW_5  
  */
  mpu.setDLPFMode(MPU6050_DLPF_BW_256);

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);  

  LEDON;
  #ifndef DEBUG
  delay(5000);
  #endif
  LEDOF;

}

void loop()
{

  Pin_1_Read=pulseIn(PIN_1, HIGH);

  Angle = getIMUOrientation();
  if(Angle.Pitch>=70) Angle.Pitch=70;
  else if(Angle.Pitch<=-70) Angle.Pitch=-70;
  if(Angle.Roll>=70) Angle.Roll=70;
  else if(Angle.Roll<=-70) Angle.Roll=-70;

  if(Pulse1<2000) 
  {
    CheckPulse1 = Pulse1;
    if(CheckPulse1<=1000)CheckPulse1=1000;
  }
  if(Pulse2<2000) 
  {
    CheckPulse2 = Pulse2;
    if(CheckPulse2<=1000) CheckPulse2=1000;
  }

  ServoValue_1= map(CheckPulse1, 1000, 2000, 0, 179);
  ServoValue_2= map(CheckPulse2, 1000, 2000, 0, 179);

  if( Pin_1_Read<1500 )
  {
    #ifdef CHECK_COMPARE_V2
    if( Angle.Pitch>=LimitMax ) /* Nghien lui + */
    {
      ServoControl_1 = (int)( ServoValue_1 - (int)(Angle.Pitch) );
      ServoControl_2 = (int)( ServoValue_2 + (int)(Angle.Pitch) );   

      if( Angle.Roll>=LimitMax or Angle.Roll<=LimitMin )
      {
        ServoControl_1 = (int)( ServoControl_1 - (int)(Angle.Roll) );
        ServoControl_2 = (int)( ServoControl_2 - (int)(Angle.Roll) );
      }
    }
    else if( Angle.Pitch<=LimitMin ) /* Nghien toi - */
    {
      ServoControl_1 = (int)( ServoValue_1 - (int)(Angle.Pitch) );
      ServoControl_2 = (int)( ServoValue_2 + (int)(Angle.Pitch) );   

      if( Angle.Roll>=LimitMax or Angle.Roll<=LimitMin )
      {
        ServoControl_1 = (int)( ServoControl_1 - (int)(Angle.Roll) );
        ServoControl_2 = (int)( ServoControl_2 - (int)(Angle.Roll) );    
      } 
    }
    else if( Angle.Roll>=LimitMax ) /* Nghien trai + */
    {
      ServoControl_1 = (int)( ServoValue_1 - (int)(Angle.Pitch) );
      ServoControl_2 = (int)( ServoValue_2 + (int)(Angle.Pitch) );   

      if( Angle.Pitch>=LimitMax or Angle.Pitch<=LimitMin )
      {
        ServoControl_1 = (int)( ServoControl_1 - (int)(Angle.Roll) );
        ServoControl_2 = (int)( ServoControl_2 - (int)(Angle.Roll) );
      }
    }
    else if( Angle.Roll<=LimitMin ) /* Nghien phai - */
    {
      ServoControl_1 = (int)( ServoValue_1 - (int)(Angle.Pitch) );
      ServoControl_2 = (int)( ServoValue_2 + (int)(Angle.Pitch) );   

      if( Angle.Pitch>=LimitMax or Angle.Pitch<=LimitMin )
      {
        ServoControl_1 = (int)( ServoControl_1 - (int)(Angle.Roll) );
        ServoControl_2 = (int)( ServoControl_2 - (int)(Angle.Roll) );
      }
    } 
    else 
    {
      ServoControl_1=ServoValue_1;
      ServoControl_2=ServoValue_2;
    }   
    #endif /*CHECK_COMPARE_V2*/

    #ifdef CHECK_COMPARE_V1
    if( Angle.Pitch>=LimitMax ) /* Nghien lui + */
    {
      ServoControl_1 = (int)( ServoValue_1 - (int)(Angle.Pitch) );
      ServoControl_2 = (int)( ServoValue_2 + (int)(Angle.Pitch) );   

      if( Angle.Roll>=LimitMax or Angle.Roll<=LimitMin )
      {
        ServoControl_1 = (int)( ServoControl_1 - (int)(Angle.Roll) );
        ServoControl_2 = (int)( ServoControl_2 - (int)(Angle.Roll) );
      }
    }
    else if( Angle.Pitch<=LimitMin ) /* Nghien toi - */
    {
      ServoControl_1 = (int)( ServoValue_1 - (int)(Angle.Pitch) );
      ServoControl_2 = (int)( ServoValue_2 + (int)(Angle.Pitch) );   

      if( Angle.Roll>=LimitMax or Angle.Roll<=LimitMin )
      {
        ServoControl_1 = (int)( ServoControl_1 - (int)(Angle.Roll) );
        ServoControl_2 = (int)( ServoControl_2 - (int)(Angle.Roll) );    
      } 
    } 
    else 
    {
      if( Angle.Roll>=LimitMax ) /* Nghien trai + */
      {
        ServoControl_1 = (int)( ServoValue_1 - (int)(Angle.Pitch) );
        ServoControl_2 = (int)( ServoValue_2 + (int)(Angle.Pitch) );   

        if( Angle.Pitch>=LimitMax or Angle.Pitch<=LimitMin )
        {
          ServoControl_1 = (int)( ServoControl_1 - (int)(Angle.Roll) );
          ServoControl_2 = (int)( ServoControl_2 - (int)(Angle.Roll) );
        }   
      }
      else if( Angle.Roll<=LimitMin ) /* Nghien phai - */
      {
        ServoControl_1 = (int)( ServoValue_1 - (int)(Angle.Pitch) );
        ServoControl_2 = (int)( ServoValue_2 + (int)(Angle.Pitch) );   

        if( Angle.Pitch>=LimitMax or Angle.Pitch<=LimitMin )
        {
          ServoControl_1 = (int)( ServoControl_1 - (int)(Angle.Roll) );
          ServoControl_2 = (int)( ServoControl_2 - (int)(Angle.Roll) );
        }    
      } 
      else 
      {
        ServoControl_1=ServoValue_1;
        ServoControl_2=ServoValue_2;
      }
    } 

    if( Angle.Roll>=LimitMax ) /* Nghien trai + */
    {
      ServoControl_1 = (int)( ServoValue_1 - (int)(Angle.Pitch) );
      ServoControl_2 = (int)( ServoValue_2 + (int)(Angle.Pitch) );   

      if( Angle.Pitch>=LimitMax or Angle.Pitch<=LimitMin )
      {
        ServoControl_1 = (int)( ServoControl_1 - (int)(Angle.Roll) );
        ServoControl_2 = (int)( ServoControl_2 - (int)(Angle.Roll) );
      }
    }
    else if( Angle.Roll<=LimitMin ) /* Nghien phai - */
    {
      ServoControl_1 = (int)( ServoValue_1 - (int)(Angle.Pitch) );
      ServoControl_2 = (int)( ServoValue_2 + (int)(Angle.Pitch) );   

      if( Angle.Pitch>=LimitMax or Angle.Pitch<=LimitMin )
      {
        ServoControl_1 = (int)( ServoControl_1 - (int)(Angle.Roll) );
        ServoControl_2 = (int)( ServoControl_2 - (int)(Angle.Roll) );
      }
    } 
    else 
    {
      if( Angle.Pitch>=LimitMax ) /* Nghien lui + */
      {
        ServoControl_1 = (int)( ServoValue_1 - (int)(Angle.Pitch) );
        ServoControl_2 = (int)( ServoValue_2 + (int)(Angle.Pitch) );   

        if( Angle.Roll>=LimitMax or Angle.Roll<=LimitMin )
        {
          ServoControl_1 = (int)( ServoControl_1 - (int)(Angle.Roll) );
          ServoControl_2 = (int)( ServoControl_2 - (int)(Angle.Roll) );
        }
      }
      else if( Angle.Pitch<=LimitMin ) /* Nghien toi - */
      {
        ServoControl_1 = (int)( ServoValue_1 - (int)(Angle.Pitch) );
        ServoControl_2 = (int)( ServoValue_2 + (int)(Angle.Pitch) );   

        if( Angle.Roll>=LimitMax or Angle.Roll<=LimitMin )
        {
          ServoControl_1 = (int)( ServoControl_1 - (int)(Angle.Roll) );
          ServoControl_2 = (int)( ServoControl_2 - (int)(Angle.Roll) );
        } 
      } 
      else 
      {
        ServoControl_1=ServoValue_1;
        ServoControl_2=ServoValue_2;
      } 
    }            
    #endif /*CHECK_COMPARE_V1*/
  
    ServoControl_1 = Kalman_1.updateEstimate(ServoControl_1);
    ServoControl_2 = Kalman_2.updateEstimate(ServoControl_2);  
  }
  else 
  {
    ServoControl_1=ServoValue_1;
    ServoControl_2=ServoValue_2;
  }

  if(ServoControl_1<=0)ServoControl_1=0;
  else if(ServoControl_1>=179)ServoControl_1=179;
  if(ServoControl_2<=0)ServoControl_2=0;
  else if(ServoControl_2>=179)ServoControl_2=179;

  myservo1.write( ServoControl_1 );
  myservo2.write( ServoControl_2 );

  #ifdef DEBUG

  Serial.print("Servo 1: "); Serial.print(ServoControl_1);
  Serial.print(" - Servo 2: "); Serial.print(ServoControl_2);

  Serial.print(" - Pitch = ");Serial.print(Angle.Pitch);
  Serial.print(" - Roll = ");Serial.print(Angle.Roll);

  // Serial.print(" Get: "); Serial.print(mpu.getDLPFMode());
  Serial.print(" Pin_1_Read: "); Serial.print( Pin_1_Read );

  Serial.println();
  delay(5);
  #endif /*DEBUG*/

}




