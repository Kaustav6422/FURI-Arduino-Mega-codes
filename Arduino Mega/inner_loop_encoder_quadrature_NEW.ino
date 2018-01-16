//Author : Kaustav Mondal
//Date   : 2-16-2018 

/*
Pins on Atmega 2560 which offer pin change interrupts

Port B

PB0 - 19 - D53 (SS)
PB1 - 20 - D52 (SCK)
PB2 - 21 - D51 (MOSI)
PB3 - 22 - D50 (MISO)
PB4 - 23 - D10 
PB5 - 24 - D11
PB6 - 25 - D12
PB7 - 26 - D13

Port E
PE0 - 2 - D0 (RXD0)

Port J
PJ0 - 63 - D15 (RXD3)
PJ1 - 64 - D14 (TXD3)
PJ2 to PJ6 - not connected on board

Port K
PK0 - PK7 - (89 - 82)  - A8 - A15 (PCINT16-23)
*/

#include "DualVNH5019MotorShield.h"
#include <math.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// IMU class instance
Adafruit_BNO055 bno = Adafruit_BNO055();

double wd ;          // Desired angular speed of COM about ICC(Instantaneous center of curvature)
double vd ;    // Desired longitudinal speed of center of mass

// Subscriber call back to /cmd_vel
void twist_message_cmd(const geometry_msgs::Twist& msg)
{
  vd = msg.linear.x  ;
  wd = msg.angular.z ;
}

// Node handle
ros::NodeHandle arduino_nh ;

// broadcasting tf
//geometry_msgs::TransformStamped t ;
//tf::TransformBroadcaster broadcaster ;

//geometry_msgs::Twist msg ;
geometry_msgs::Vector3Stamped rpm_msg;

// Publisher of the right and left wheel angular velocities
ros::Publisher pub("arduino_vel", &rpm_msg);

nav_msgs::Odometry odom_robot;
//ros::Publisher pub("odom",&odom_robot);

// Subscriber of the reference velocities comming from the outerloop
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &twist_message_cmd );

// Left Encoder
#define LH_ENCODER_A PK0 //  pin A8 (PCINT16)
#define LH_ENCODER_B PK1 //  pin A9 (PCINT17)
volatile long left_ticks = 0L;
volatile bool LeftEncoderBSet ;

// Right Encoder
#define RH_ENCODER_A PB0  // Digital pin 53 (PCINT 0)
#define RH_ENCODER_B PB1  // Digital pin 52 (PCINT 1)
volatile long right_ticks = 0L;
volatile bool RightEncoderBSet ;

#define LEFT  0
#define RIGHT 1

static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0}; //encoder lookup table

/* Interrupt routine for LEFT encoder, taking care of actual counting */
ISR (PCINT2_vect) // pin change interrupts for port K (A8,A9)
{
  static uint8_t enc_last=0;
  enc_last <<=2; //shift previous state two places
  enc_last |= (PINK & (3 << 0)) >> 0 ;
  left_ticks -= ENC_STATES[(enc_last & 0x0f)];
}
  
/* Interrupt routine for RIGHT encoder, taking care of actual counting */
ISR (PCINT0_vect) // pin change interrupts for port J (Digital pin 14,15)
{
  static uint8_t enc_last=0;          
  enc_last <<=2; //shift previous state two places
  enc_last |= (PINB & (3 << 0)) >> 0 ;
  right_ticks -= ENC_STATES[(enc_last & 0x0f)];
}

DualVNH5019MotorShield md;

// Varibles for storing calculated velocity
double wR;      // present angular speed of right motor
double wL;      // present angular speed of left motor
double wRp=0.0; // previous angular speed right motor
double wLp=0.0; // previous angular speed left motor
double wLn;     // average angular speed (wL + wLp)/2  
double wRn;     // average angular speed (wR + wRp)/2
double LdVal = 0; //??
//double Lcount_last = 0 ;
double RdVal = 0; //??
//double Rcount_last = 0 ;
double Radius =0.045; // Change it (radius of wheel)
double Length =0.555; // Change it (distance between wheels)

double x = 0 ;
double y = 0 ;
double theta = 0 ;
double vd_odom = 0 ;
double wd_odom = 0 ;

double dxy = 0;
double dtheta = 0;
double dx = 0;
double dy = 0;
double x_final = 0;
double y_final = 0;
double theta_final = 0;

double angle = 0; // Serial data from outerloop (desired angle of pan tilt) // MAY NOT BE USED NOW
double wdr;       // Desired angular speed of right wheel using wd & vd /  prefilter parameter x_{n+1}
double wdl;       // Desired angular speed of left wheel using wd & vd  / prefilter parameter x_{n+1}
double wdr_p=0;   // prefilter parameter x_{n} for right motor
double wdl_p=0;   // prefilter parameter x_{n} for left motor
double wrf;       // prefilter parameter y_{n+1} for right motor
double wlf;       // prefilter parameter y_{n+1} for left motor
double wrf_p=0;   // prefilter paramter y_{n} for right motor
double wlf_p=0;   // prefilter parameter y_{n} for left motor'

double CR;       // Controller output y_{n+2} Right motor
double CR_p=0;   // Controller output y_{n+1} Right motor
double CR_pp=0;  // Controller output y_{n}   Right motor
double CL;       // Controller output y_{n+2} Left motor
double CL_p=0;   // Controller output y_{n+1} Left motor
double CL_pp=0;  // Controller output y_{n}   Left motor

double Lerror;   // Lerror = wlf(output of prefilter/ reference speed) - wLn.....or... Controller input x_{n+2}
double Lerror_p = 0; // Controller input x_{n+1}
double Lerror_pp = 0; // Controller input x_{n}
double Rerror;   // Rerror = wrf(output of prefilter/ reference speed) - wRn....or.....Controller input x_{n+2}
double Rerror_p = 0; // Controller input x_{n+1}
double Rerror_pp = 0; // Controller input x_{n}

double angularV = 0 ;

int PWMR; // Controller output for right motor
int PWML; // Controller output for left motor

// Depending on the number of counts in the encoder
// 2100x4 for 131:1
// 800x4 for 50:1
int CPR = 3200 ;//  Count per revolution(CPR) 131*64 
   
double kp = 5 ;          // Controller gain kp of K = (kp + ki/s) * (100/(s+100))
double ki = 50 ;          // Controller gain ki 
double alpha = 200;  // Roll off parameter alpha 
double h = ki/kp;    //  prefilter parameter z = ki/kp obtained from K = (g(s+z)/s)*(100/(s+100)) 

long Lcount; // Present Encoder value
long Rcount; // Present Encoder value    
long Lcount_last=0; // Previous encoder value
long Rcount_last=0;   // Previous encoder value

unsigned long Time=0; // Starting time
unsigned long lastMilli = 0; 
double td = 0.01; // T = 0.01 sec (100 hz)
unsigned long sample_time= td*1000 ;                             ;  // T = 100 msec
               
// Global variables for reading serial data
const int NUMBER_OF_FIELDS = 2 ;
int fieldIndex = 0;
double values[NUMBER_OF_FIELDS] ;
int sign = 1;

// SETUP MOTORS
void SetupMotors() 
{
  md.init() ;
}

// SETUP ENCODERS
void SetupEncoders()
{
  // Initializing the encoder pins as input pins
  
  // set as inputs DDRD(pins 0-7) , DDRC(A0-A5) 
  // (The Port D Data Direction Register - read/write)
  DDRK &= ~(1<<LH_ENCODER_A); // PK0 pin A8
  DDRK &= ~(1<<LH_ENCODER_B); // PK1 pin A9
  DDRB &= ~(1<<RH_ENCODER_A); // Digital pin 53 (PB0)
  DDRB &= ~(1<<RH_ENCODER_B); // Digital pin 52 (PB1)

  /* Pin to interrupt map:
   * D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
   * D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
   * A0-A5 (D14-D19) = PCINT 8-13 = PCIR1 = PC = PCIE1 = pcmsk1
  */

  /*
     For Atmega 2560 pin change interrupt enable flags 
     PCIE2 : PCINT23-16
     PCIE1 : PCINT15-8
     PCIE0 : PCINT7-0
  */  

  // tell pin change mask to listen to left encoder pins and right pins
  PCMSK2 |= (1 << LH_ENCODER_A)|(1 << LH_ENCODER_B);
  PCMSK0 |= (1 << RH_ENCODER_A)|(1 << RH_ENCODER_B);

  // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
  // the Pin Change Interrupt Enable flags have to be set in the PCICR register. These are bits PCIE0, PCIE1 and PCIE2 for the groups of pins PCINT7..0, PCINT14..8 and PCINT23..16 respectively
  PCICR |= (1 << PCIE0) | (1 << PCIE2);
  //PCICR |= (1 << PCIE2) ; 
  
}

void setup()
{
  // Init Serial port with 115200 baud rate
  // Serial.begin(9600); 
  bno.begin();

  // Arduino node
  arduino_nh.initNode() ;

  //broadcaster.init(arduino_nh) ; //added
  
  arduino_nh.getHardware()->setBaud(115200);
  arduino_nh.advertise(pub); // setting up subscriptions
  arduino_nh.subscribe(sub); // setting up publications
  
  // Settiing up encoders
  SetupEncoders() ;

  // Setting up motors
  SetupMotors() ;

  delay(1000) ;
}

// Main loop
void loop()
{ 
    if (millis() - Time > sample_time)
    { 
      Time = millis() ;

      // Update Motors with corresponding speed and send speed values through serial port
      Update_Motors(vd,wd);

      // Publish odom data
      Publish_odom() ; 

      arduino_nh.spinOnce() ;
    }   
}

// Publish angular velocities of right/left wheel
// Publish the angle rotated (obtained from IMU)
void Publish_odom()
{
  // twist message for velocity

  /*               
  dxy    = vd_odom * td ; //arduino_dt ;
  dtheta = wd_odom * td ; //arduino_dt ;
  dx =  cos(dtheta) * dxy ;
  dy = -sin(dtheta) * dxy ;
  x_final = x_final + (cos(theta_final)*dx - sin(theta_final)*dy) ;
  y_final = y_final + (sin(theta_final)*dx + cos(theta_final)*dy) ;
  theta_final = theta_final + dtheta ;

  t.header.stamp = arduino_nh.now() ;
  t.header.frame_id = "/odom" ;
  t.child_frame_id  = "/base_link" ;
  t.transform.translation.x = x_final ;
  t.transform.translation.y = y_final ;
  t.transform.translation.z = 0 ;
  t.transform.rotation = tf::createQuaternionFromYaw(theta_final) ;
  broadcaster.sendTransform(t) ;
  */

  /*
  odom_robot.header.frame_id = "/odom" ;
  odom_robot.child_frame_id = "/base_link" ;
  odom_robot.header.stamp = arduino_nh.now();
  odom_robot.pose.pose.position.x = x_final ;
  odom_robot.pose.pose.position.y = y_final ;
  odom_robot.pose.pose.position.z = 0       ;
  odom_robot.pose.pose.orientation = tf::createQuaternionFromYaw(theta_final) ;
  odom_robot.twist.twist.linear.x = vd_odom ;
  odom_robot.twist.twist.linear.y = 0 ;
  odom_robot.twist.twist.angular.z = wd_odom ;
  pub.publish(&odom_robot) ;
  */
 
  rpm_msg.header.stamp = arduino_nh.now();
  rpm_msg.vector.x =  wLn ;
  rpm_msg.vector.y =  wRn ;
  rpm_msg.vector.z =  angularV ;
  pub.publish(&rpm_msg);
  
  
  //arduino_nh.spinOnce();
}

// UPDATE MOTORS
void Update_Motors(double vd, double wd)
{
  // Desired angular speed of two motors
  wdr = (2*vd - Length*wd)/(2*Radius) ;
  wdl = (2*vd + Length*wd)/(2*Radius) ;

  //Prefilter
  wrf = ( (td*h)*wdr + (td*h)*wdr_p - (td*h - 2)*wrf_p )/(2 + td*h);
  wlf = ( (td*h)*wdl + (td*h)*wdl_p - (td*h - 2)*wlf_p )/(2 + td*h);
  wrf_p = wrf;
  wlf_p = wlf;
  wdr_p = wdr;
  wdl_p = wdl;

  // Encoder counts
  Lcount = left_ticks ;
  Rcount = right_ticks ;
  LdVal = (double) (Lcount - Lcount_last)/(td) ; // Counts per second // td not clear
  RdVal = (double) (Rcount - Rcount_last)/(td) ; // Counts per second // td not clear

  // Present angular velocities
  wL = (LdVal/CPR)*(2*3.14159) ; // rads/sec
  wR = (RdVal/CPR)*(2*3.14159) ; // rads/sec

  vd_odom = Radius*(wR + wL)/2 ;
  wd_odom = Radius*(wL - wR)/Length ;

  wLn = (wL + wLp)/2.0;
  wRn = (wR + wRp)/2.0;
   
  wLp = wL; // saving present angular velocities to be used in the next loop
  wRp = wR; // saving present angular velocities to be used in the next loop

  // I saw in the trial runs that if I don't use prefilter, the movement is very jerky !! So always use prefilter.    
  Rerror = wrf - wRn ; // error (ref - present)
  Lerror = wlf - wLn ; // error (ref - present)

  // Inner loop controller
  CL = ((alpha*td*td*ki+2*alpha*td*kp)*Lerror + (2*alpha*td*td*ki)*Lerror_p + (alpha*td*td*ki-2*alpha*td*kp)*Lerror_pp + 8*CL_p - (4-2*alpha*td)*CL_pp)/(2*alpha*td + 4);
  CR = ((alpha*td*td*ki+2*alpha*td*kp)*Rerror + (2*alpha*td*td*ki)*Rerror_p + (alpha*td*td*ki-2*alpha*td*kp)*Rerror_pp + 8*CR_p - (4-2*alpha*td)*CR_pp)/(2*alpha*td + 4);

  CR_pp = CR_p;
  CR_p = CR;
  CL_pp = CL_p;
  CL_p = CL;
  Lerror_pp = Lerror_p;
  Lerror_p = Lerror;
  Rerror_pp = Rerror_p;
  Rerror_p = Rerror; 

  PWMR = CR  ;//int(255.0*CR/5.15);  // CHANGE THIS !!
  PWML = CL  ;//int(255.0*CL/5.15);  // CHANGE THIS !!

  // Saturating input commands to right motor   
  if (PWMR>=400) 
  {  
    PWMR=400;
  } 
  else if (PWMR<=-400) 
  {
    PWMR=-400 ;
  }

  // Saturating input commands to left motor
  if (PWML>=400) 
  {
    PWML=400 ;
  }
  else if (PWML<=-400) 
  {
    PWML=-400 ;
  }
  
  // Running the motors
  md.setM1Speed(0) ; // l  +ve(YES)/-ve(NO) PWML
  md.setM2Speed(0) ; // r  +ve(YES)/-ve(YES) PWMR

  Lcount_last = Lcount;
  Rcount_last = Rcount;

  //imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  //angularV = double(sin(0.2516)*gyro.y() + cos(0.2516)*gyro.z());

  // Obtain the angle measurement from the IMU
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  angularV = (180 - euler.x()) ;
  
}

