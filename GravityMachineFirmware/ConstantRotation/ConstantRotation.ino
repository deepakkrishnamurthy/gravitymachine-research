
/*Code to track a moving object using on-the-fly image analysis
  and Stepper motor control. See also the code to do the same using DC motor control (PID_ImageTracking_v3)
  Code accepts (X_com,Y_com) values from the Serial port of PC/Raspberry-pi
  and tracks this position.
  Code has two modes: OPEN LOOP and CLOSED LOOP
  Uses the Arduino PID library in Closed Loop Mode.
  Nomenclature: CW is +ve, CCW is -ve
  -by Deepak Krishnamurthy
  last mod: 2018-02-03
********************************************************/
/* Axis convention
********************************************************
X-axis - Along the plane of the wheel (+ve X axis Points Radially Outward)
Y axis: Optical axis; along rotational axis of the wheel (Orientation fixed by the Right-Hand Rule using the other two axes)
+ve Z- axis : anti-parallel to gravity vector 
********************************************************/
//=================================================================================
// HEADER FILES
//=================================================================================
#include <AccelStepper.h>
#include <stdlib.h>
//=================================================================================
// Mathematical constants
//=================================================================================
const double pi = 3.1415926535897;
//=================================================================================
// PIN Definitions
//=================================================================================
// Direction and Step outputs
//--------------------------------------------------
#define STEP_X 6
#define DIR_X 7
#define STEP_Y 4
#define DIR_Y 5
#define STEP_Z 8
#define DIR_Z 9
//--------------------------------------------------
// Manual Input Pins
//--------------------------------------------------
// X axis - Linear Actuator
#define moveXpos 44
#define moveXneg 42
// Y axis - Focus
#define moveYpos 26
#define moveYneg 27
// Z axis - Wheel
#define moveZpos 46
#define moveZneg 48 
//--------------------------------------------------
// Joystick Manual Input Pins
//--------------------------------------------------
#define analogStickX A3
#define analogStickZ A4
//--------------------------------------------------
// Speed control input pin
//--------------------------------------------------
#define analogSpeed A5
//--------------------------------------------------
// MicroStepping Pins
//--------------------------------------------------
// X axis: Along the plane of the wheel
//--------------------------------------------------
#define MS1_X 29
#define MS2_X 31
#define MS3_X 33
//--------------------------------------------------
// Y axis: along rotational axis of the wheel
//--------------------------------------------------
#define MS1_Y 28
#define MS2_Y 30
#define MS3_Y 32
//--------------------------------------------------
// +ve Z- axis : anti-parallel to gravity vector 
//--------------------------------------------------
#define MS1_Z 34
#define MS2_Z 36
#define MS3_Z 38
//--------------------------------------------------
// Other user input Pins
//--------------------------------------------------
// X axis: Along the plane of the wheel
//--------------------------------------------------
#define ManualPin 39
//=================================================================================
// Imaging Variables
//=================================================================================
float PixelPermm = 87;    // For a 1x magnification objective
float PixelPermmInv = 1/PixelPermm;
//=================================================================================
// Stage Movement/Physical Variables
//=================================================================================
// Common Physical Variables
//--------------------------------------------------
int adc_depth = 1023;                                       // Bit depth of the ADC 
int StageSampleTime = 500;                                   // Interval at which the stage speed control is monitored (milliseconds)
int StickSampleTime = 50;                                   // Interval at which the Joystick is monitored (milliseconds)
long int HomePos_X = 0, HomePos_Y = 0, HomePos_Z = 0;       // Initial position of the three steppers (in steps) 
long int CurrPos_X = 0,CurrPos_Y = 0, CurrPos_Z = 0;        // Current Position as measured by the Stepper in Steps
long int PosStart_X = 0, PosStart_Y = 0, PosStart_Z = 0;
float Xobj = 0, Zobj = 0, Robj = 0, Thetaobj = 0;
int TargetCurr_X = 0, TargetCurr_Y = 0, TargetCurr_Z = 0, TargetPrev_X = 0, TargetPrev_Y = 0, TargetPrev_Z = 0;            // Target required in Steps 
bool ms1_X,ms2_X,ms3_X,ms1_Y,ms2_Y,ms3_Y,ms1_Z,ms2_Z,ms3_Z;       // State of microstepping pins
int microSteps_X = 1, microSteps_Y = 1, microSteps_Z = 1;            // no:of fractional steps per full step

int unitSpeedX = 1000, unitSpeedY = 1000, unitSpeedZ = 1500; // Smallest set speed of the motors in Steps/s (higher speeds are multiples of this speed)
int stepperSpeedX, stepperSpeedY, stepperSpeedZ;
int maxSpeedX, maxSpeedY, maxSpeedZ;                        // Maximum speed of X,Y,Z steppers in steps/s

int maxAcclX, maxAcclY, maxAcclZ;                           // Maximum acceleration of X,Y,Z steppers in steps/s^2

int TrackingPi = 0;
int stickX, stickZ, stickHomeX = 0, stickHomeZ = 0;
int stageSpeed;
//--------------------------------------------------
// X Stepper (Linear Stage)
//--------------------------------------------------
int StepsPerRev_X = 20;
float mmPerRev_X = 0.5;   // Pitch of the lead screw in mm
float StepsPermm_X = StepsPerRev_X/mmPerRev_X;

//--------------------------------------------------
// Y Stepper (Focussing Stage)
//--------------------------------------------------
int StepsPerRev_Y = 200;
float mmPerRev_Y = 0.001524; // Pitch of the lead screw in mm
float StepsPermm_Y = StepsPerRev_Y/mmPerRev_Y;
//--------------------------------------------------
// Z stepper (Phidget Stepper) that drives the Wheel
//--------------------------------------------------
float gearRatio = 99+1044/2057, rotAngleStep=0, rotAngleCount = 0;   // Gear ratio of Phidgets Stepper
float Rcenter = 200;  // Radius to the center line of the fluidic chamber in mm
float mmPerRev_Z = 2*pi*Rcenter/gearRatio;          // Displacement along the centerline of wheel in mm for 1 revolution of the Motor shaft
int StepsPerRev_Z = gearRatio * 200;                  // No:of steps of the main motor shaft for 1 Rev of the output shaft
float StepsPerRad_Z = StepsPerRev_Z/(2*pi);         // No:of steps of the main motor shaft for 1 radian turn of the output shaft
int CountsPerRev_Z = 600;

float StepsPermm_Z = StepsPerRev_Z/mmPerRev_Z;
float AbsPos_Z = 20;  // required absolute position in degrees


float AbsPosSteps_Z = (AbsPos_Z/360.0)*gearRatio*StepsPerRev_Z;
float AbsPosCounts_Z = (AbsPos_Z/360.0)*gearRatio*CountsPerRev_Z;

//=================================================================================
// State Variables
//=================================================================================
bool Xpos, Xneg, Ypos, Yneg, Zpos, Zneg, ManualInput, ManualMode=0, ManualModePrev=0;
//=================================================================================
// Encoder Variables
//=================================================================================
#define c_EncoderPinA 20            // Pin to read encoder channel A (Z encoder of the wheel)
#define c_EncoderPinB 21            // Pin to read encoder channel B (Z encoder of the wheel)

#define c_InputEncoderPinA 2            // Pin to read encoder channel A (Rotary Encoder for Z-stage)
#define c_InputEncoderPinB 3            // Pin to read encoder channel B (Rotary Encoder for Z-stage)

volatile bool _EncoderASet;
volatile bool _EncoderBSet;
volatile bool _EncoderAPrev;
volatile bool _EncoderBPrev;
volatile long _EncoderTicks = 0;

volatile bool _InputEncoderASet;
volatile bool _InputEncoderBSet;
volatile bool _InputEncoderAPrev;
volatile bool _InputEncoderBPrev;
volatile long _InputEncoderTicks = 0;

volatile int Dir, DirInput, ErrorCount=0;
//=================================================================================
// Serial Comm Variables
//=================================================================================
int SerialRecFreq = 50;        // Number of times per sec the Serial Port is polled to read the Object coordinates
int SerialRecDt =int(1000/SerialRecFreq);     // Time between two reads of the serial port in millis
int SerialSendFreq = 50;
int SerialSendDt =int(1000/SerialSendFreq);

volatile unsigned long prevMillisSend = 0, currMillisSend = 0, prevMillisRec = 0, currMillisRec = 0, currMillisStage=0, prevMillisStage = 0, MillisStick = 0, currMillis = 0, prevMillis = 0;

const byte numCharsSmall = 2;
const byte numCharsLong = 14;
char receivedChars0[numCharsSmall];
char receivedChars1[numCharsLong]; // an array to store the received data
char receivedChars2[numCharsLong];


//============================================================================================================
// Create required number of instances of the AccelStepper Class based on the no:of Stepper motors to control
//============================================================================================================
// X axis stepper
AccelStepper stepperX(AccelStepper::DRIVER, STEP_X, DIR_X);     
// Y axis stepper
AccelStepper stepperY(AccelStepper::DRIVER, STEP_Y, DIR_Y);     
// Z axis stepper
AccelStepper stepperZ(AccelStepper::DRIVER, STEP_Z, DIR_Z);     

int TrackingPi_temp, StepsX_temp, StepsY_temp;
int StepsPrev = 1000;
void setup()
{
  //===============================================================================
  // Setup User Input Pins
  //===============================================================================
  pinMode(moveXpos, INPUT); pinMode(moveXneg, INPUT);
  pinMode(moveYpos, INPUT); pinMode(moveYneg, INPUT);
  pinMode(moveZpos, INPUT); pinMode(moveZneg, INPUT);

  pinMode(ManualPin, INPUT);
  //===============================================================================
  // Setup Serial Communication 
  //===============================================================================
  Serial.begin (115200);
  
  Serial.println('a');
  char a = 'b';

  while (a != 'a' && ManualMode == 0)        // Wait for command 'a' to be sent from the host computer.
  {
    ManualMode = digitalRead(ManualPin);
    a = Serial.read();
    

  }
  //-------------------------------------------------------------------------------
  // Store the home position of the Joystick
  //-------------------------------------------------------------------------------
  // stickHomeX = analogRead(analogStickX);          // This is 12 bit value (0-1023)
  // stickHomeZ = analogRead(analogStickZ);          // This is 12 bit value (0-1023)

  // stickX = stickHomeX;
  // stickZ = stickHomeZ;
  //===============================================================================
  // Stepper motor pin set-up
  //===============================================================================
  ms1_X = HIGH; ms2_X = HIGH; ms3_X = HIGH;    // X-axis linear stage
  ms1_Y = LOW; ms2_Y = LOW; ms3_Y = LOW;    // Focus stage
  ms1_Z = HIGH; ms2_Z = HIGH; ms3_Z = HIGH;    // Wheel 
  //-------------------------------------------------------------------------------
  // Set up the DIRECTION pins as OUTPUT
  //-------------------------------------------------------------------------------
  pinMode(DIR_X, OUTPUT); digitalWrite(DIR_X, LOW);
  pinMode(DIR_Y, OUTPUT); digitalWrite(DIR_Y, LOW);
  pinMode(DIR_Z, OUTPUT); digitalWrite(DIR_Z, LOW);
  //-------------------------------------------------------------------------------
  // Set up the STEP pins as OUTPUT
  //-------------------------------------------------------------------------------
  pinMode(STEP_X, OUTPUT); digitalWrite(STEP_X, LOW);
  pinMode(STEP_Y, OUTPUT); digitalWrite(STEP_Y, LOW);
  pinMode(STEP_Z, OUTPUT); digitalWrite(STEP_Z, LOW);
  //-------------------------------------------------------------------------------
  // Set the values of the MICROSTEPPING Pins
  //-------------------------------------------------------------------------------
  pinMode(MS1_X, OUTPUT); digitalWrite(MS1_X, ms1_X);
  pinMode(MS2_X, OUTPUT); digitalWrite(MS2_X, ms2_X);
  pinMode(MS3_X, OUTPUT); digitalWrite(MS3_X, ms3_X);

  pinMode(MS1_Y, OUTPUT); digitalWrite(MS1_Y, ms1_Y);
  pinMode(MS2_Y, OUTPUT); digitalWrite(MS2_Y, ms2_Y);
  pinMode(MS3_Y, OUTPUT); digitalWrite(MS3_Y, ms3_Y);

  pinMode(MS1_Z, OUTPUT); digitalWrite(MS1_Z, ms1_Z);
  pinMode(MS2_Z, OUTPUT); digitalWrite(MS2_Z, ms2_Z);
  pinMode(MS3_Z, OUTPUT); digitalWrite(MS3_Z, ms3_Z);

  //-------------------------------------------------------------------------------
  // Find the number of fractional steps for 1 full Step based on microstepping pin states
  //-------------------------------------------------------------------------------
  microSteps_X = 16;
  microSteps_Y = 16;
  microSteps_Z = 16;

  //-------------------------------------------------------------------------------
  // Set the MAX speed and MAX acceleration of all the steppers (Speeds > 1000 full-steps per second are unreliable)
  //-------------------------------------------------------------------------------
  
  maxSpeedX = maxStepperSpeed(microSteps_X, unitSpeedX); 
  maxSpeedY = maxStepperSpeed(microSteps_Y, unitSpeedY); 
  maxSpeedZ = maxStepperSpeed(microSteps_Z, unitSpeedZ); 

  stepperX.setMaxSpeed(maxSpeedX);
  stepperX.setAcceleration(1000);
   
  stepperY.setMaxSpeed(maxSpeedY);
  stepperY.setAcceleration(1000);

  stepperZ.setMaxSpeed(5000);
  stepperZ.setAcceleration(10000);
  //-------------------------------------------------------------------------------
  // Set the current positions of the steppers as the Zero position
  //-------------------------------------------------------------------------------
  stepperX.setCurrentPosition(stepperX.currentPosition());
  stepperY.setCurrentPosition(stepperY.currentPosition());
  stepperZ.setCurrentPosition(stepperZ.currentPosition());
  //-------------------------------------------------------------------------------
  // Quadrature Encoder pin setup
  //-------------------------------------------------------------------------------
  pinMode(c_EncoderPinA, INPUT_PULLUP);      // sets pin A as input
  pinMode(c_EncoderPinB, INPUT_PULLUP);      // sets pin B as input
  pinMode(c_InputEncoderPinA, INPUT);      // sets pin A as input
  pinMode(c_InputEncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(c_InputEncoderPinA, HIGH);
  digitalWrite(c_InputEncoderPinB, HIGH);
  
 
 // attachInterrupt(digitalPinToInterrupt(c_EncoderPinA), HandleMotorInterruptA, CHANGE);

 // attachInterrupt(digitalPinToInterrupt(c_InputEncoderPinA), HandleInputEncoderInterrupt, CHANGE);
 stepperZ.move(StepsPrev);
  
}

void loop()
{
 
    stepperZ.move(10000);
    stepperZ.run();
   
}
void setMicroSteps(int stageSpeed)
{
  // Set the desired microstepping configurations for the steppers based on position of the analog speed control.
  if(stageSpeed < adc_depth/5.0)
  {
    
    ms1_X = HIGH; ms2_X = HIGH; ms3_X = HIGH;
    ms1_Z = HIGH; ms2_Z = HIGH; ms3_Z = HIGH;
    
  }
  else if (stageSpeed > adc_depth/5.0 && stageSpeed<=adc_depth*(2/5.0))
  { 
    ms1_X = HIGH; ms2_X = HIGH; ms3_X = LOW;
    ms1_Z = HIGH; ms2_Z = HIGH; ms3_Z = LOW;  
    
  }
  else if (stageSpeed > adc_depth*(2/5.0) && stageSpeed<=adc_depth*(3/5.0))
  {
    ms1_X = LOW; ms2_X = HIGH; ms3_X = LOW;
    ms1_Z = LOW; ms2_Z = HIGH; ms3_Z = LOW;
  }
    
  else if (stageSpeed > adc_depth*(3/5.0) && stageSpeed<=adc_depth*(4/5.0))
  {
    ms1_X = HIGH; ms2_X = LOW; ms3_X = LOW;
    ms1_Z = HIGH; ms2_Z = LOW; ms3_Z = LOW;
    
  }

    
  else if (stageSpeed > adc_depth*(4/5.0) && stageSpeed<=adc_depth)
  {
    ms1_X = LOW; ms2_X = LOW; ms3_X = LOW;
    ms1_Z = LOW; ms2_Z = LOW; ms3_Z = LOW;
    
  }
  digitalWrite(MS1_X, ms1_X);
  digitalWrite(MS2_X, ms2_X);
  digitalWrite(MS3_X, ms3_X);

  digitalWrite(MS1_Z, ms1_Z);
  digitalWrite(MS2_Z, ms2_Z);
  digitalWrite(MS3_Z, ms3_Z);

    

}

void setMicroStepsTracking()
{
  // Default microstep settings for tracking

  ms1_X = HIGH; ms2_X = HIGH; ms3_X = HIGH;
  ms1_Z = HIGH; ms2_Z = HIGH; ms3_Z = HIGH;
  digitalWrite(MS1_X, ms1_X);
  digitalWrite(MS2_X, ms2_X);
  digitalWrite(MS3_X, ms3_X);

  digitalWrite(MS1_Z, ms1_Z);
  digitalWrite(MS2_Z, ms2_Z);
  digitalWrite(MS3_Z, ms3_Z);


}
int maxStepperSpeed(int microSteps, int unitSpeed)
{
  // Returns the maximum Speed of the steppers based on the microstep configuration
  if (microSteps == 1)
  {
    return unitSpeed;
  }
  else if (microSteps == 2)
  {
    return int(1.2*unitSpeed);
  }
  else if (microSteps == 4)
  {
    return int(2*unitSpeed);
  }
  else if (microSteps == 8)
  {
    return int(3*unitSpeed);
  }
  else if (microSteps == 16)
  {
    return int(5*unitSpeed);
  }

}
//int stepperSpeed(int diffSignal, int maxSpeed)
//{
//  if (diffSignal <= 10)
//    return 0;
//  else if (diffSignal > 10 && diffSignal<=adc_depth/5)
//    return int(0.2*maxSpeed);
//  else if (diffSignal > adc_depth/5 && diffSignal<=adc_depth*(2/5))
//    return int(0.4*maxSpeed);
//  else if (diffSignal > adc_depth*(2/5) && diffSignal<=adc_depth*(3/5))
//    return int(0.6*maxSpeed);
//  else if (diffSignal > adc_depth*(3/5) && diffSignal<=adc_depth*(4/5))
//    return int(0.8*maxSpeed);
//   else if (diffSignal > adc_depth*(4/5) && diffSignal<=adc_depth)
//    return int(maxSpeed);
//
//}

int microSteppingLUT(bool MS1, bool MS2, bool MS3)
{
  //-------------------------------------------------------------------------------
  // Returns the number of fractional steps per 1 full step of the motor shaft
  //-------------------------------------------------------------------------------
  if(MS1==LOW && MS2==LOW && MS3 == LOW)
  {
    return 1;
  }
  else if(MS1==HIGH && MS2==LOW && MS3 == LOW)
  {
    return 2;
  }
  else if(MS1==LOW && MS2==HIGH && MS3 == LOW)
  {
    return 4;
  }
  else if(MS1==HIGH && MS2==HIGH && MS3 == LOW)
  {
    return 8;
  }
  else if(MS1==HIGH && MS2==HIGH && MS3 == HIGH)
  {
    return 16;
  }


}

// Interrupt service routines for the motor's quadrature encoder
void HandleMotorInterruptA()
{
  _EncoderBSet = digitalRead(c_EncoderPinB);
  _EncoderASet = digitalRead(c_EncoderPinA);

   //Dir=ParseEncoder();
   Dir=Decoder();
  _EncoderTicks += Dir;
  
  _EncoderAPrev = _EncoderASet;
  _EncoderBPrev = _EncoderBSet;
}

void HandleInputEncoderInterrupt()
{
  _InputEncoderBSet = digitalRead(c_InputEncoderPinB);
  _InputEncoderASet = digitalRead(c_InputEncoderPinA);

   //Dir=ParseEncoder();
   DirInput=DecoderInput();
  _InputEncoderTicks += DirInput;
  
  _InputEncoderAPrev = _InputEncoderASet;
  _InputEncoderBPrev = _InputEncoderBSet;

  // Step the Focus stage Stepper by a small number of steps
  stepperY.move(DirInput*100);
  stepperY.setMaxSpeed(maxSpeedY);
  stepperY.run();
}

int Decoder()
{
  if(!_EncoderAPrev && _EncoderASet)
  {
    if(_EncoderBSet) return -1;
    else return 1;
  }
  else if(_EncoderAPrev && !_EncoderASet)
  {
    if(!_EncoderBSet) return -1;
    else return 1;
  }
  else return 0;
  
}
int DecoderInput()
{
  if(!_InputEncoderAPrev && _InputEncoderASet)
  {
    if(_InputEncoderBSet) return 1;
    else return -1;
  }
  else if(_InputEncoderAPrev && !_InputEncoderASet)
  {
    if(!_InputEncoderBSet) return 1;
    else return -1;
  }
  else return 0;

}
