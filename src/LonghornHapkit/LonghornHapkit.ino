
//--------------------------------------------------------------------------
// Emmanuel Akita, Kevin Torres, Frank Regal, University of Texas at Austin
// Last Modified: 08.27.21
// Code to test basic functionaility of the Longhorn Hapkit (w/ encoder)
//--------------------------------------------------------------------------
// whats up
// INCLUDES
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <TimerOne.h>  // This library manages the timing of the haptic loop 
#include <Encoder.h>   // This library manages the encoder read.


// Pin Declarations
const int PWMoutp = 4;
const int PWMoutn = 5;
const int PWMspeed = 6;

const int encoder0PinA = 2;
const int encoder0PinB = 3;

Encoder encoder(encoder0PinA, encoder0PinB);

double encoderResolution = 48;
double pos = 0;
double lastPos = 0;
double lastVel = 0;

// Kinematics variables
double xh = 0;           // position of the handle [m]
double theta_s = 0;      // Angle of the sector pulley in deg
double xh_prev;          // Distance of the handle at previous time step
double xh_prev2;
double dxh;              // Velocity of the handle
double vh;
double lastLastVh;
double lastVh;
double lastXh;
double dxh_prev;
double dxh_prev2;
double dxh_filt;         // Filtered velocity of the handle
double dxh_filt_prev;
double dxh_filt_prev2;


// *******************************************
// UNCOMMENT THESE AND INCLUDE CORRECT NUMBERS
// *******************************************
double rh = 0.090;     //[m]
double rp = 0.005;     //[m]
double rs = 0.074;     //[m]
double C = (rh*rp) / rs; // Constant for kinematic constant (0.006)

// *******************************************

// Force output variables
double force = 0;           // force at the handle
double force_min = 0;       // min force of system
double force_max = 0;       // max force of system
double Tp = 0;              // torque of the motor pulley
double Tp_max = 0;          // maximum torque based on maximum force found before motor slips
double Tp_min = 0;          // minimum torque based on the minimum force for motor to move
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor
double max_vel = 0;
double max_vel_new = 0;

// Timing Variables: Initalize Timer and Set Haptic Loop
boolean hapticLoopFlagOut = false;
boolean timeoutOccured = false;
double t = 0; // time
double T = 0.001; // time step
double i = 0;

//double A = 1;       // amplitude
//double lambda = 1;  // decay constant
//double phi = 1;     // intial phase angle, rad
//double freq = 5;    // sampling freq, Hz
//double omega = freq*2*3.1459;  // angular, rad/s
//double exponential_func = 0.0;


//--------------------------------------------------------------------------
// Initialize
//--------------------------------------------------------------------------
void setup()
{
  // Set Up Serial
  Serial.begin(115200);

  // Output Pins
  pinMode(PWMoutp, OUTPUT);
  pinMode(PWMoutn, OUTPUT);
  pinMode(PWMspeed, OUTPUT);

  // Haptic Loop Timer Initalization
  Timer1.initialize();
  long period = 1000; // [us]  10000 [us] - 100 Hz
  Timer1.attachInterrupt(hapticLoop, period);

  // Init Position and Velocity
  lastPos = encoder.read();
  lastVel = 0;

  // Initalize motor direction and set to 0 (no spin)
  digitalWrite(PWMoutp, HIGH);
  digitalWrite(PWMoutn, LOW);
  analogWrite(PWMspeed, 0);

}

//--------------------------------------------------------------------------
// Main Loop
//--------------------------------------------------------------------------

void loop()
{
  if (timeoutOccured)
  {
    Serial.println("timeout occured");
  }
}

// --------------------------
// Haptic Loop (Only thing we need to edit)
// --------------------------
void hapticLoop()
{

  // See if flag is out (couldn't finish before another call)
  if (hapticLoopFlagOut)
  {
    timeoutOccured = true;
  }
  //*************************************************************
  //*** Section 1. Compute position and velocity using encoder (DO NOT CHANGE!!) ***
  //*************************************************************
  pos = encoder.read();
  double vel = (.80) * lastVel + (.20) * (pos - lastPos) / (.01);


  //*************************************************************
  //*** Section 2. Compute handle position in meters ************
  //*************************************************************

  // ADD YOUR CODE HERE

  // SOLUTION:
  // Define kinematic parameters you may need

  // Step 2.1: print updatedPos via serial monitor
  //*************************************************************

  //Serial.println(pos);
  //pos_max (right) = -78
  //pos_min (left) = 70

  // Step 2.2: Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  //*************************************************************

  double ts = -0.0107 * pos + 4.9513; // NOTE - THESE NUMBERS MIGHT NOT BE CORRECT! USE KINEMATICS TO FIGRUE IT OUT!
  //double ts = -0.0107*pos;
  // Step 2.3: Compute the position of the handle based on ts
  //*************************************************************

  xh = rh * (ts * 3.14159 / 180); // Again, these numbers may not be correct. You need to determine these relationships.
  //Serial.println(ts,5);
  //Serial.println(xh,5);

  // Step 2.4: print xh via serial monitor
  //*************************************************************
  //Serial.println(xh,5);

  // Min (Left):  0.00660 (m)
  // Center:      0.00778 (m)
  // Max (Right): 0.00911 (m)

  //double xh_min = 0.00682; //[m]
  //double xh_max = 0.00900; //[m]

  // Step 2.5: compute handle velocity
  //*************************************************************

  vh = -(.95 * .95) * lastLastVh + 2 * .95 * lastVh + (1 - .95) * (1 - .95) * (xh - lastXh) / .0001; // filtered velocity (2nd-order filter)
  lastXh = xh;
  lastLastVh = lastVh;
  lastVh = vh;

  //Serial.println(lastVh,6);

  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******
  //*************************************************************


  // min PWM value for motor to move = 70;
  // max PWM value for motor to slip = 143;
  // max_force = 1.5515
  // min_force = 0.3718

  // Init force
  double force = 0; // N


  // Virtual Spring
  //*************************************************************
//              double K_spring = 3.5;      // N/m - spring stiffness
//              force = K_spring*xh;

  // Virtual Wall
  //*************************************************************
//             double K_wall = 150;
//             double x_wall = 0.0081;
//             //Serial.println(xh,5);
//  
//             if (xh < x_wall)
//             {
//              force = 0;
//             }
//             else if (xh > x_wall)
//             {
//              force = K_wall * (xh - x_wall);
//             }

  // Linear Damping
  //*************************************************************
//              double b = 0.7;
//              Serial.println(vh,5);
//              force = b * vh;

  // Nonlinear Friction
  //*************************************************************

//              double b_small = 0.3;
//              double b_big = 3;
//  
//              double max_vh = 0.04;
//              double min_vh = 0.01;
//  
//              if (pos < -10 )
//  
//              {
//               force = b_small * vh;
//              }
//              else if (pos > -10 && pos < 10)
//              {
//               force = b_big * vh;
//              }
//              else if (pos > 10)
//              {
//               force = b_small * vh;
//              }


  // A Hard Surface
  //*************************************************************
  //double K_surface = 150;
  //double x_wall = 0.0081;
  //Serial.println(xh,5);

// #NOTE: START THE HANDLE TO THE LEFT OF THE WALL
//
//  double A = 1;       // amplitude
//  double lambda = 0.5;  // decay constant
//  double phi = 1;     // intial phase angle, rad
//  double freq = 5;    // sampling freq, Hz
//  double omega = freq * 2 * 3.1459; // angular, rad/s
//  double exponential_func = 0.0;
//  double K_wall = 150;
//
//  if (xh > x_wall)
//  {
//    force = (K_wall * (xh - x_wall)) * (exp(-lambda * t) * (cos(omega * t * phi))) + 2;
//            t = t + T;
//  } else if (xh < x_wall)
//  {
//    t = 0;
//  }
//  //Serial.println(exp(-lambda*t) * (cos(omega*t*phi)));
  //Serial.println(force);


  // Bump and Valley
  //*************************************************************
//  double K_bump = 2;
//  double K_valley = 0.5;
//  double xbuffer = 0.05; // 0.0195 * 2 to degrees
//  double ts_scaled = 0; // new scaled value for ts
//  
//  if (ts > 4.9513 + xbuffer)
//  {
//    ts = ts - 4.9513;
//    ts_scaled = ts * 230.8144; // degrees
//    ts_scaled = (ts_scaled * 3.1459 / 180); // radians
//    force = K_bump * rh * sin(ts_scaled);
//    if(force < 0)
//      force = abs(force);
//  } else if (ts < 4.9513 - xbuffer)
//  {
//    ts = ts - 4.9513;
//    ts_scaled = ts * 230.8144; // degrees
//    ts_scaled = (ts_scaled * 3.1459 / 180); // radians
//    force = K_valley * rh * cos(ts_scaled);
//    force = - abs(force);
//  } else
//  {
//    ts = ts - 4.9513;
//    force = 0;
//  }


  // Texture
  //*************************************************************
//              double ts_scaled = 0;    // new scaled value for ts
//              double K_texture = 0.05;
//              if (ts > 4.9513)
//              {
//              ts = ts - 4.9513;
//              ts_scaled = ts * 230.8144; // degrees
//              ts_scaled = (ts_scaled*3.1459/180); // radians
//              force = K_texture*(0.5*sin(7*ts_scaled) + 0.5*sin(20*ts_scaled));
//              }
   //CHALLENGE POINTS: Try simulating a paddle ball! Hint you need to keep track of the virtual balls dynamics and
  // compute interaction forces relative to the changing ball position.
  //*************************************************************

  Tp = C * force;    // output torque based on force calculations above


  //force_min = K * xh_min;
  //force_max = K * xh_max;

  //Tp_max =  C * force_max; // Calculated max torque from force calibration

  //Tp_min =  C * force_min;   // Calculated min torque from force calibration
  //output = 140;//map(Tp, Tp_min, Tp_max, 70, 143);

  // max_duty = 0.5608 because max PWM (143)/255 = 0.5608
  // min_duty = 0.2745 because min PWM (70)/255 = 0.2745

  duty = sqrt(abs(Tp) / 0.0003);
  if (duty > 1)
  {
    duty = 1;
  } else if (duty < 0)
  {
    duty = 0;
  }
  output = (int)(duty * 255);
  //Serial.println(force,5);

  //*************************************************************
  //*** Section 5. Force output (do not change) *****************
  //*************************************************************

  // Determine correct direction
  //*************************************************************
  if (force < 0)
  {
    // motor turns counterclockwise
    // handle turns clockwise
    digitalWrite(PWMoutp, HIGH);
    digitalWrite(PWMoutn, LOW);
  } else
  {
    // motor turned clockwise
    // handle turns counterclockwise
    digitalWrite(PWMoutp, LOW);
    digitalWrite(PWMoutn, HIGH);
  }

  // Limit torque to motor and write
  //*************************************************************
  if (abs(output) > 255)
  {
    output = 255;
  }
  //Serial.println(force); // Could print this to troublshoot but don't leave it due to bogging down speed
  //Serial.println(output);

  // Write out the motor speed.
  //*************************************************************
  analogWrite(PWMspeed, abs(output)); //abs(force)

  // Update variables
  lastVel = vel;
  lastPos = pos;

}
