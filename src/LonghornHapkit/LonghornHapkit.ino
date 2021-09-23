
//--------------------------------------------------------------------------
// Ann Majewicz Fey, University of Texas at Austin
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

Encoder encoder(encoder0PinA,encoder0PinB);

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
double rh = 0.09;   //[m] 
double rp = 0.005;  //[m] 
double rs = 0.074;  //[m] 
double C = (rh*rp)/rs; // Constant for kinematic constant (0.006)

// *******************************************

// Force output variables
double force = 0;           // force at the handle
double force_min =0;        // min force of system
double force_max = 0;       // max force of system
double Tp = 0;              // torque of the motor pulley
double Tp_max = 0;          // maximum torque based on maximum force found before motor slips
double Tp_min = 0;          // minimum torque based on the minimum force for motor to move
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor

// Timing Variables: Initalize Timer and Set Haptic Loop
boolean hapticLoopFlagOut = false; 
boolean timeoutOccured = false; 

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
  Timer1.attachInterrupt(hapticLoop,period); 

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
    if(timeoutOccured)
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
      if(hapticLoopFlagOut)
      {
        timeoutOccured = true;
      }
      //*************************************************************
      //*** Section 1. Compute position and velocity using encoder (DO NOT CHANGE!!) ***  
      //*************************************************************
      pos = encoder.read();
      double vel = (.80)*lastVel + (.20)*(pos - lastPos)/(.01);


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

           double ts = -.0107*pos + 4.9513; // NOTE - THESE NUMBERS MIGHT NOT BE CORRECT! USE KINEMATICS TO FIGRUE IT OUT!
       
         // Step 2.3: Compute the position of the handle based on ts
          //*************************************************************

           xh = rh*(ts*3.14159/180);       // Again, these numbers may not be correct. You need to determine these relationships. 
           

        
          // Step 2.4: print xh via serial monitor
          //*************************************************************
           //Serial.println(pos);
           Serial.println(xh,5);
           
           /// Min (Left): 0.00660 (m)
           ///Center: 0.00778 (m)
           /// Max (Right): 0.00911 (m)
           double xh_min = 0.00682; 
           double xh_max = 0.009; 

          // Step 2.5: compute handle velocity
          //*************************************************************
            vh = -(.95*.95)*lastLastVh + 2*.95*lastVh + (1-.95)*(1-.95)*(xh-lastXh)/.0001;  // filtered velocity (2nd-order filter)
            lastXh = xh;
            lastLastVh = lastVh;
            lastVh = vh;

//           Serial.println(vh,5);

        //*************************************************************
        //*** Section 3. Assign a motor output force in Newtons *******  
        //*************************************************************
 
            // Init force 
            double force = 0; // N
            double K = 15;   // spring stiffness 
            //force = K * 0.008;
        // Virtual Spring 
        //*************************************************************

//           Serial.println(pos);
//           if(pos < 0)
//          {
//            force = -K*xh; 
//          } else 
//          {
//            force = K*xh; 
//          }
//        

          // Virtual Wall 
        //*************************************************************
//           if (xh<0)
//           {
//            force = -K*xh;
//           }
//           else 
//           {
//            force = 0;
//           }


         // Linear Damping 
        //*************************************************************
        

         // Nonlinear Friction
        //*************************************************************
        

         // A Hard Surface 
        //*************************************************************
        

         // Bump and Valley  
        //*************************************************************


          // Texture 
        //*************************************************************

           // CHALLENGE POINTS: Try simulating a paddle ball! Hint you need to keep track of the virtual balls dynamics and 
           // compute interaction forces relative to the changing ball position.  
        //*************************************************************
      
      Tp = C * force;    // output torque based on force calculations above
      
    
      force_min = K * xh_min; 
      force_max = K * xh_max; 

      Tp_max =  C * force_max; // Calculated max torque from force calibration
      Tp_min =  C * force_min;   // Calculated min torque from force calibration
      output = 0; //map(Tp, Tp_min, Tp_max, 70, 143);
       
      //*************************************************************
      //*** Section 5. Force output (do not change) *****************
      //*************************************************************

        // Determine correct direction 
        //*************************************************************
        if(output < 0)
        {
        digitalWrite(PWMoutp, HIGH);
        digitalWrite(PWMoutn, LOW);
        } else 
        {
        digitalWrite(PWMoutp, LOW);
        digitalWrite(PWMoutn, HIGH);
        } 
    
        // Limit torque to motor and write
        //*************************************************************
        if(abs(output) > 255)
        {
          output = 255; 
        }
            //Serial.println(force); // Could print this to troublshoot but don't leave it due to bogging down speed

        // Write out the motor speed.
        //*************************************************************    
        analogWrite(PWMspeed, abs(output)); //abs(force)
  
  // Update variables 
  lastVel = vel;
  lastPos = pos; 

}
