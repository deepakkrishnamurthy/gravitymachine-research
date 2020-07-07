/********************************************************
  Code to track a moving object using on-the-fly image analysis
  and Stepper motor control. See also the code to do the same using DC motor control (PID_ImageTracking_v3)
  Code accepts (X_com,Y_com) values from the Serial port of PC/Raspberry-pi
  and tracks this position.
  Code has two modes: OPEN LOOP and CLOSED LOOP
  Uses the Arduino PID library in Closed Loop Mode.
  Nomenclature: CW is +ve, CCW is -ve
  -by Deepak Krishnamurthy & François BENOIT du REY
  last mod: 2018-05-19
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
#include <DueTimer.h>
#include <Wire.h>

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
#define SYNC_X 14
//--------------------------------------------------
// Manual Input Pins
//--------------------------------------------------
// X axis - Linear Actuator
#define moveXpos 42     //Orange
#define moveXneg 44     // Yellow
// Y axis - Focus
#define moveYpos 26
#define moveYneg 27
// Z axis - Wheel 0
#define moveZpos 46   // Green
#define moveZneg 48     // Blue


//--------------------------------------------------
// Speed control input pin (sliders)
//--------------------------------------------------
#define analogSpeedXZ A5
#define analogSpeedY A6
//--------------------------------------------------
// Light Intensity PWM pin
#define lightPWM 10
//--------------------------------------------------
//--------------------------------------------------
// Light Intensity Measurement
//--------------------------------------------------
#define lightPin A0
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
// Manual pin
//--------------------------------------------------
#define ManualPin 39
//--------------------------------------------------
// Limit Switch
//--------------------------------------------------
#define X_limit_1 14   // -ve X end of the stepper
#define X_limit_2 15    // +ve X end of the stepper
#define Y_limit_1 16    // -ve Y end of the Stepper
#define Y_limit_2 17    // +ve Y end of the stepper

//--------------------------------------------------
// Trigger Pin
//--------------------------------------------------
# define triggerPin 12
# define triggerPin_GND 11
# define trigger_indicator_Pin 50
# define sendSerial_indicator_pin 51
# define trigger_indicator_Pin_GND 52
# define sendSerial_indicator_pin_GND 53
//--------------------------------------------------
// Serial communication
//--------------------------------------------------
# define CMD_LENGTH 18
# define DATA_TX_LENGTH 26

//=================================================================================
// Stage Movement/Physical Variables
//=================================================================================
//--------------------------------------------------
// Common Physical Variables
//--------------------------------------------------
int adc_depth = 1023;                                        // Speed Slider: Bit depth of the ADC
int ManualSampleTime = 500;                                  // Speed Slider:Interval at which the joystick sensitivity is monitored (milliseconds)

long int CurrPos_X = 0,CurrPos_Y = 0, CurrPos_Z = 0;         // Current Position as measured by the Stepper in Steps
long int CurrPos_X_code = 0,CurrPos_Y_code = 0, CurrPos_Z_code = 0;         // Current Position as measured by the Stepper in Steps
long int CurrPos_X_Stepper = 0, CurrPos_Y_Stepper = 0, CurrPos_Z_Stepper = 0, PrevPos_X_Stepper = 0, PrevPos_Y_Stepper = 0, PrevPos_Z_Stepper = 0;
long int PosStart_X = 0, PosStart_Y = 0, PosStart_Z = 0;

float DeltaT=0, Homing=0, ZTargetVelocity=0; //Data from the computer
float Step_X = 0, Step_Y = 0, Step_Z = 0;

int TargetCurr_X = 0, TargetCurr_Y = 0, TargetCurr_Z = 0, TargetPrev_X = 0, TargetPrev_Y = 0, TargetPrev_Z = 0;            // Target required in Steps 
bool ms1_X,ms2_X,ms3_X,ms1_Y,ms2_Y,ms3_Y,ms1_Z,ms2_Z,ms3_Z;          // State of microstepping pins
int microSteps_X = 1, microSteps_Y = 1, microSteps_Z = 1;            // no:of fractional steps per full step
int microSteps_Old_X=1, microSteps_Old_Y=1;
//--------------------------------------------------
// Variables to store lower and upper limits of the stage:
//--------------------------------------------------

long int X_steps_lower=0, X_steps_upper = 0,Y_steps_lower=0, Y_steps_upper = 0,Z_steps_lower=0, Z_steps_upper = 0;
long int X_home = 0, Y_home = 0, Z_home = 0;

int maxManualSpeedX = 2000, maxManualSpeedY = 2000, maxManualSpeedZ = 1000; 
int stepperSpeedX, stepperSpeedY, stepperSpeedZ;
int instantaneousSpeedZ=0;

int maxAcclX, maxAcclY, maxAcclZ;                           // Maximum acceleration of X,Y,Z steppers in steps/s^2

int Tracking = 0;
int joystickSensitivity,joystickSensitivityPrev;
int YfocusSensitivity=0,YfocusSensitivityPrev=0;
bool joystickSensitivityChange=LOW;
bool YfocusSensitivityChange=LOW;
bool sensitivityChange=LOW;
//=================================================================================
// State Variables
//=================================================================================
bool Xpos, Xneg, Ypos, Yneg, Zpos, Zneg, ManualInput, ManualMode = LOW, ManualModePrev = LOW;
// State variables for stage limit switches
bool  _xLim_1=HIGH, _xLim_2 = HIGH, _xLim_1_prev = HIGH, _xLim_2_prev = HIGH,  _yLim_1 = HIGH, _yLim_2 = HIGH,  _yLim_1_prev = HIGH,  _yLim_2_prev = HIGH, _zLim_1 = HIGH, _zLim_2 = HIGH, _xLim = LOW;
bool _xFlip = LOW, _yFlip=LOW, _xLimPos=LOW, _xLimNeg = LOW, _yLimPos=LOW, _yLimNeg = LOW;

int reachedEnd=1;
//=================================================================================
// Encoder Variables
//=================================================================================
#define c_EncoderPinA 18            // Pin to read encoder channel A (Z encoder of the wheel)
#define c_EncoderPinB 19            // Pin to read encoder channel B (Z encoder of the wheel)

#define c_InputEncoderPinA 2            // Pin to read encoder channel A (Rotary Encoder for Z-stage)
#define c_InputEncoderPinB 3            // Pin to read encoder channel B (Rotary Encoder for Z-stage)

volatile bool _EncoderASet;
volatile bool _EncoderBSet;
volatile bool _EncoderAPrev;
volatile bool _EncoderBPrev;
volatile long int _EncoderTicks = 0;
volatile long int _EncoderTicks_code = 0;

volatile bool _InputEncoderASet;
volatile bool _InputEncoderBSet;
volatile bool _InputEncoderAPrev;
volatile bool _InputEncoderBPrev;
volatile long _InputEncoderTicks = 0;

volatile int Dir, DirInput, ErrorCount=0;

//=================================================================================
// Liquid Lens variable
//=================================================================================

bool isYtracking = false;
volatile float liquidLensSweepFrequency = 10;
volatile float liquidLensVOffset = 39.5;
volatile float liquidLensVAmplitude = 0;   //10 corresponds to 1.4 mm
bool liquidLensOrder=0;
volatile float phase = 0;
volatile int phase_code = 0;
volatile int phase_code_lastTrigger = 0;

int timerPeriod = 500; // in us
float liquidLensV = 0;
int liquidLensVCode = 0;
//=================================================================================
// Focus-stacks and Homing
//=================================================================================
int sweepCounter = 0, nSweeps = 10;
float distanceSwept = 0.5;  // Swept distance in mm
int nSteps = 16*200*distanceSwept;  // no:of 1/16 steps
bool inProgress = false, moveStart = true;
bool reachedX1 = false, reachedX2 = false, reachedY1 = false, reachedY2 = false;
bool StageLocked = false;  // Locks the stage so that manual inputs are over-ridden during HOMING
int x_sign = 1, y_sign = -1;
bool found_Xlimits = false, found_Ylimits = false;
bool atXhome = false, atYhome = false;
bool x_home_inProgress = false, y_home_inProgress = false;
int fullStepsToYhome = 3766;      // Number of full-steps to reach the center of the Y-stage from -ve limit swtch

//=================================================================================
// Light modulation variables
//=================================================================================
int lightOutput = 0, lightMeasured = 0;
int lightUpdateTime = 500;         // Only update the light intensity every 500 ms

//=================================================================================
// Serial Comm Variables
//=================================================================================
bool isReceived=true;
volatile bool sendData = false;

//=================================================================================
// Triggers per second
//=================================================================================
int fps = 50;
int numTimerCycles = 1000000/fps/timerPeriod;
volatile int counter_timer = -1;

volatile unsigned long prevMillisSend = 0, currMillisSend = 0, prevMillisRec = 0, currMillisRec = 0, currMillisStage=0, prevMillisStage = 0, MillisStick = 0, currMillis = 0, currMillisLight = 0;
volatile unsigned long timestamp_lastTrigger = 0;
bool startTriggering = false;

//=================================================================================
// Serial Communication
//=================================================================================
byte buffer_tx[DATA_TX_LENGTH];
byte buffer_rx[500];
int buffer_rx_ptr;
bool calculateCRC_tx = true;

//============================================================================================================
// Create required number of instances of the AccelStepper Class based on the no:of Stepper motors to control
//============================================================================================================
// X axis stepper
AccelStepper stepperX(AccelStepper::DRIVER, STEP_X, DIR_X);     
// Y axis stepper
AccelStepper stepperY(AccelStepper::DRIVER, STEP_Y, DIR_Y);     
// Z axis stepper
AccelStepper stepperZ(AccelStepper::DRIVER, STEP_Z, DIR_Z);     


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//-------------------------------------------------------------------------------
//                          Beginning of SETUP
//-------------------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


void setup()
{
  pinMode(ManualPin, INPUT_PULLUP);
  
  //===============================================================================
  // Setup Serial Communication 
  //===============================================================================
  Serial.begin(115200);
  
  //===============================================================================
  // Setup User Input Pins
  //===============================================================================
  pinMode(moveXpos, INPUT_PULLUP); pinMode(moveXneg, INPUT_PULLUP);
  pinMode(moveYpos, INPUT_PULLUP); pinMode(moveYneg, INPUT_PULLUP);
  pinMode(moveZpos, INPUT_PULLUP); pinMode(moveZneg, INPUT_PULLUP);

  
  
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
  pinMode(SYNC_X, OUTPUT); digitalWrite(SYNC_X, LOW);

  pinMode(MS1_X, OUTPUT); pinMode(MS2_X, OUTPUT); pinMode(MS3_X, OUTPUT); 
  pinMode(MS1_Y, OUTPUT); pinMode(MS2_Y, OUTPUT); pinMode(MS3_Y, OUTPUT); 
  pinMode(MS1_Z, OUTPUT); pinMode(MS2_Z, OUTPUT); pinMode(MS3_Z, OUTPUT); 

  //===============================================================================
  // Setup Limit Switch Pins
  //===============================================================================
  pinMode(X_limit_1,INPUT_PULLUP); pinMode(X_limit_2,INPUT_PULLUP);
  pinMode(Y_limit_1,INPUT_PULLUP); pinMode(Y_limit_2,INPUT_PULLUP);

  //===============================================================================
  // Setup Trigger Pin
  //===============================================================================
  pinMode(triggerPin_GND,OUTPUT);
  digitalWrite(triggerPin_GND,LOW);
  pinMode(triggerPin,OUTPUT);
  digitalWrite(triggerPin,LOW);

  //===============================================================================
  // Setup Indicator Pin
  //===============================================================================
  pinMode(trigger_indicator_Pin,OUTPUT);
  digitalWrite(trigger_indicator_Pin,LOW);
  pinMode(sendSerial_indicator_pin,OUTPUT);
  digitalWrite(sendSerial_indicator_pin,LOW);
  pinMode(trigger_indicator_Pin_GND,OUTPUT);
  digitalWrite(trigger_indicator_Pin_GND,LOW);
  pinMode(sendSerial_indicator_pin_GND,OUTPUT);
  digitalWrite(sendSerial_indicator_pin_GND,LOW);
    
  //-------------------------------------------------------------------------------
  // Set the initial microstepping
  //-------------------------------------------------------------------------------
  microSteps_X = 16;
  microSteps_Y = 16;
  microSteps_Z = 16;

  //-------------------------------------------------------------------------------
  // actualise local variable ms1_X (resp. Y & Z) ms2.. ms3.. and write in the pin MS1.. MS2.. MS3..
  //-------------------------------------------------------------------------------
  setMS123(microSteps_X,'X');
  setMS123(microSteps_Y,'Y');
  setMS123(microSteps_Y,'Z');

  //-------------------------------------------------------------------------------
  // Set the initial speed and the MAX acceleration of all the steppers (Speeds > 1000 full-steps per second are unreliable)
  //-------------------------------------------------------------------------------
  
  stepperSpeedX = microstep_to_manualSpeed(microSteps_X, maxManualSpeedX); 
  stepperSpeedY = microstep_to_manualSpeed(microSteps_Y, maxManualSpeedY); 
  stepperSpeedZ = microstep_to_manualSpeed(microSteps_Z, maxManualSpeedZ); 

  maxAcclX=1000; maxAcclY=1000; maxAcclZ=1000;

  stepperX.setMaxSpeed(stepperSpeedX);
  stepperX.setAcceleration(maxAcclX);
   
  stepperY.setMaxSpeed(stepperSpeedY);
  stepperY.setAcceleration(maxAcclY);

  stepperZ.setMaxSpeed(stepperSpeedZ);
  stepperZ.setAcceleration(maxAcclZ);
  

  //-------------------------------------------------------------------------------
  // Quadrature Encoder pin setup
  //-------------------------------------------------------------------------------
  pinMode(c_EncoderPinA, INPUT_PULLUP);      // sets pin A as input
  pinMode(c_EncoderPinB, INPUT_PULLUP);      // sets pin B as input
  pinMode(c_InputEncoderPinA, INPUT);        // sets pin A as input
  pinMode(c_InputEncoderPinB, INPUT);        // sets pin B as input
  digitalWrite(c_InputEncoderPinA, HIGH);
  digitalWrite(c_InputEncoderPinB, HIGH);
  
 
  attachInterrupt(digitalPinToInterrupt(c_EncoderPinA), HandleMotorInterruptA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(c_InputEncoderPinA), HandleInputEncoderInterrupt, CHANGE);

  //-------------------------------------------------------------------------------
  // Liquid Lens Setup
  //-------------------------------------------------------------------------------
  
  Wire.begin();
  Wire.setClock(1000000);

  // init DAC
  analogWriteResolution(10);
  //-------------------------------------------------------------------------------
  // Light modulation setup
  //-------------------------------------------------------------------------------
  pinMode(lightPWM, OUTPUT);

  delay(1000);

  // initialize timer
  Timer3.attachInterrupt(liquid_lens_handler_timer_500us);
  Timer3.start(timerPeriod); // Calls every 500 us
  startTriggering = true;


  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperZ.setCurrentPosition(0);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//-------------------------------------------------------------------------------
//                          Beginning of the main loop
//-------------------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


void loop()
{
  

  ManualModePrev = ManualMode;
  
  ManualMode = digitalRead(ManualPin);
  readLimitSwitch();
  currMillisRec = millis();
  sensitivityChange=LOW;

  //-------------------------------------------------------------------------------
  // Read the sensitivity of the joystick at a fixed frequency
  //-------------------------------------------------------------------------------
  
  currMillisStage = millis() - currMillisStage;
   if(currMillisStage >= ManualSampleTime)
    {      
      
      // Read the desired stage speed from the analog Slider
      joystickSensitivity = analogRead(analogSpeedXZ);
  
      if (joystickSensitivity!=joystickSensitivityPrev){
        // Set the microstepping mode based on the stage speed
        setMicroStepsJoystick(joystickSensitivity);
        joystickSensitivityPrev=joystickSensitivity;
  
        // Set the speed in manual Mode based on the microstep mode
        stepperSpeedX = microstep_to_manualSpeed(microSteps_X, maxManualSpeedX);  
        stepperSpeedZ = microstep_to_manualSpeed(microSteps_Z, maxManualSpeedZ); 
  
        sensitivityChange=HIGH;
      }
  
       YfocusSensitivity = analogRead(analogSpeedY);
  
      if (YfocusSensitivity!=YfocusSensitivityPrev){
        // Set the microstepping mode based on the stage speed
        setMicroStepsYfocus(YfocusSensitivity);
        YfocusSensitivityPrev=YfocusSensitivity;
  
        // Set the speed in manual Mode based on the microstep mode
        stepperSpeedY = microstep_to_manualSpeed(microSteps_Y, maxManualSpeedY);   
  
        YfocusSensitivityChange=HIGH;
      }

      lightMeasured = analogRead(lightPin);
      // Print for debugging
      Serial.print("Y limit found:");
      Serial.println(found_Ylimits);
      Serial.print("Y home in progress:");
      Serial.println(y_home_inProgress);
      Serial.print("Y home reached:");
      Serial.println(atYhome);
      

      
    }

  //-------------------------------------------------------------------------------
  // Manual Input Block
  //-------------------------------------------------------------------------------
  if(ManualMode && StageLocked==false)
  {

    Zpos = digitalRead(moveZpos);
    Zneg = digitalRead(moveZneg);

    Xpos = digitalRead(moveXpos);
    Xneg = digitalRead(moveXneg);
//      TargetCurr_X = (Xpos+(-1)*Xneg)*100;

    TargetCurr_X = (Xpos*(_xLimPos)+(-1)*Xneg*(_xLimNeg))*100;
    TargetCurr_Z = (Zpos+(-1)*Zneg)*100;

    stepperZ.move(microSteps_Z * TargetCurr_Z );
    stepperX.move(microSteps_X * TargetCurr_X );
  }


  //-------------------------------------------------------------------------------
  // Update the Stepper Position
  //-------------------------------------------------------------------------------
  
  // Current position is stored in terms of no:of fractional steps (1/16)
  CurrPos_X_Stepper = stepperX.currentPosition();
  CurrPos_Y_Stepper = stepperY.currentPosition();
  
  CurrPos_Z_Stepper = stepperZ.currentPosition();

  CurrPos_X += (CurrPos_X_Stepper - PrevPos_X_Stepper)*16/microSteps_X;
  CurrPos_Y += (CurrPos_Y_Stepper - PrevPos_Y_Stepper)*16/microSteps_Y;
  CurrPos_Z += (CurrPos_Z_Stepper - PrevPos_Z_Stepper)*16/microSteps_Z;
  
  PrevPos_X_Stepper = CurrPos_X_Stepper;
  PrevPos_Y_Stepper = CurrPos_Y_Stepper;
  PrevPos_Z_Stepper = CurrPos_Z_Stepper;


  //-------------------------------------------------------------------------------
  // Homing
  //-------------------------------------------------------------------------------
      
  if(!ManualModePrev && ManualMode)
  {
    Homing = 1;
  }
  else
  {
    Homing = 0;
  }
        

  if (Homing==1 || inProgress == true)
  {

      if(Homing==1 && inProgress == false)
      { 
            // Lock the stage so Manual inputs and speed changes are not acted upon.
          StageLocked = true;

          microSteps_Old_X = XHoming_SetSpeed();
          microSteps_Old_Y = YHoming_SetSpeed();

          inProgress = true;
       }
      //--------------------------
      // X-homing routine 

      if(found_Xlimits == false)
      {
            X_Stepper_FindLimits();
      }
      else if(found_Xlimits == true && atXhome == false && x_home_inProgress == false)
      {
            X_home = (int)round((X_steps_upper + X_steps_lower)/2);
            stepperX.moveTo(X_home);
            stepperX.setSpeed(5000);
            x_home_inProgress = true;
      }

      if(x_home_inProgress == true && stepperX.distanceToGo() == 0)
      {
           atXhome = true;
      }

      // Y-homing routine
      if(found_Ylimits == false)
      {
          Y_Stepper_FindLimits();
      }
          
      else if(found_Ylimits == true && atYhome == false && y_home_inProgress == false)
      {
           
          Y_home = microSteps_Y*fullStepsToYhome;
          stepperY.move(Y_home);
          y_home_inProgress = true;

      }

      if(y_home_inProgress == true && stepperY.distanceToGo() == 0)
      {
          atYhome = true;
      }

      //If Homing is complete for both stages
      if(atXhome && atYhome)
      { 
          
         
          stepperX.setCurrentPosition(0);
          stepperY.setCurrentPosition(0);
          stepperZ.setCurrentPosition(0);

          CurrPos_X_Stepper = 0;
          CurrPos_Y_Stepper = 0;
          CurrPos_Z_Stepper = 0;

          PrevPos_X_Stepper = 0;
          PrevPos_Y_Stepper = 0;
          PrevPos_Z_Stepper = 0;

          resetHomingState();
          
          reset_X_StepperSpeed(microSteps_Old_X);
          reset_Y_StepperSpeed(microSteps_Old_Y);

      }
  }
  
   
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //Step the stepper (This block should run as frequently as possible)
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
  
  stepperX.run();
  stepperY.run();
  stepperZ.run();

  

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//-------------------------------------------------------------------------------
//                             End of the main loop
//-------------------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


// Reset homing State variables
void resetHomingState()
{
  inProgress = false;
  Homing = 0;
  StageLocked = false;
  atXhome = false;
  atYhome = false;
  found_Xlimits = false;
  found_Ylimits = false;
  x_home_inProgress = false;
  y_home_inProgress = false;
  reachedX1 = false;
  reachedX2 = false;
  reachedY1 = false;
  reachedY2 = false;
  x_sign = 1;
  y_sign = -1;
  X_home = 0;
  X_steps_lower = 0;
  X_steps_upper = 0;

  
  
}


//-------------------------------------------------------------------------------
//                       Homing function for the X-stepper
//-------------------------------------------------------------------------------
int XHoming_SetSpeed()
{
    int microSteps_old=microSteps_X;
    microSteps_X = 4;
    setMS123(microSteps_X,'X');
    stepperX.setMaxSpeed(10000);

    return microSteps_old;

}
int YHoming_SetSpeed()
{

  int microSteps_old=microSteps_Y;
  microSteps_Y = 4;
  setMS123(microSteps_Y,'Y');
  stepperY.setMaxSpeed(10000);

  return microSteps_old;

}
void reset_X_StepperSpeed(int microSteps_old)
{
  
  setMS123(microSteps_old,'X');
  stepperX.setMaxSpeed(stepperSpeedX);
}

void reset_Y_StepperSpeed(int microSteps_old)
{
  setMS123(microSteps_old,'Y');
  stepperY.setMaxSpeed(stepperSpeedY);
}

void X_Stepper_FindLimits()
{

  _xLim_1_prev = _xLim_1;
  _xLim_2_prev = _xLim_2;
  
  _xLim_1 = digitalRead(X_limit_1);
  _xLim_2 = digitalRead(X_limit_2);
   
  stepperX.move(x_sign*400);
  
  if(!_xLim_1 && _xLim_1_prev)
  {
      reachedX1 = true;
      x_sign = -x_sign;
      // Store the position as one limit
      X_steps_lower = stepperX.currentPosition();
     
  }
  else if(!_xLim_2 &&_xLim_2_prev)
  {
      reachedX2 = true;
      x_sign = -x_sign;
      X_steps_upper = stepperX.currentPosition();
    

  }

  if(reachedX1 && reachedX2)
  {   
      found_Xlimits = true;
  }

 



}

void Y_Stepper_FindLimits()
{
    _yLim_1_prev = _yLim_1;
    _yLim_2_prev = _yLim_2;
    _yLim_1 = digitalRead(Y_limit_1);
    _yLim_2 = digitalRead(Y_limit_2);

    stepperY.move(y_sign*400);
    
    if(!_yLim_1 && _yLim_1_prev)
    {
        reachedY1 = true;
        y_sign = -y_sign;
        // Store the position as one limit
       

    }
    else if(!_yLim_2 && _yLim_2_prev)
    {
        reachedY2 = true;
        y_sign = -y_sign;
    }

    if(reachedY1 || reachedY2)
    {
      found_Ylimits = true;
    
    }

    

}


void XStepperHoming(){

  //-------------------------------------------------------------------------------
  //                                 X-stepper
  //-------------------------------------------------------------------------------
  
  int microSteps_old=microSteps_X;
  microSteps_X=int(2);
  setMS123(microSteps_X,'X');
  stepperX.setMaxSpeed(8000);
  
  // First move stepper in the +ve direction till one of the limits is reached
  do
  {
    stepperX.move(100);
    stepperX.run();
    _xLim_1 = digitalRead(X_limit_1);
    _xLim_2 = digitalRead(X_limit_2);
  }while(_xLim_1 && _xLim_2);
  
  
  if(!_xLim_1)
    // If lim1 is reached when stepping in the +ve direction
  {
    // Store the position as one limit
    X_steps_upper = stepperX.currentPosition();
    _xFlip = HIGH;

    do
    {
      stepperX.move(-100);
      stepperX.run();
      _xLim_2 = digitalRead(X_limit_2);
    }while(_xLim_2);

    X_steps_lower = stepperX.currentPosition();
    


  }
  // If lim2 is reached when stepping in +ve direction
  else if(!_xLim_2)
  {  
    // Store the position as one limit 
    X_steps_upper = stepperX.currentPosition();
    _xFlip = LOW;
    do
    {
      stepperX.move(-100);
      stepperX.run();
      _xLim_1 = digitalRead(X_limit_1);
      
    }while(_xLim_1);

    X_steps_lower = stepperX.currentPosition();
    
  }

//  Serial.print("Lower X limit: ");
//  Serial.println(X_steps_lower);
//
//  Serial.print("Upper X limit: ");
//  Serial.println(X_steps_upper);

  X_home = (X_steps_upper + X_steps_lower)/2;
  
//  Serial.print("Home position");
//  Serial.println(X_home);

  stepperX.moveTo(X_home);
  
  while(stepperX.currentPosition() != X_home)
  {
    stepperX.run();
  }

//  Serial.print("Final position");
//  Serial.println(stepperX.currentPosition());
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperZ.setCurrentPosition(0);

  // Once the homing is completed send a signal to the computer
  //SerialUSB.write(byte(1));

  microSteps_X=microSteps_old;
  setMS123(microSteps_X,'X');
  stepperX.setMaxSpeed(stepperSpeedX);

  
}
void YStepperHoming()
{
  //-------------------------------------------------------------------------------
  //                                 Y-stepper
  // Note that this a bit different from X-stepper homing since due to space constrains we cannot use both limit switches for homing
  //-------------------------------------------------------------------------------
  int fullStepsToYhome = 3766;      // Number of full-steps to reach the center of the Y-stage from -ve limit swtch
   
  int microSteps_old=microSteps_Y;
  microSteps_Y = 2;
  setMS123(microSteps_Y,'Y');
  stepperY.setMaxSpeed(8000);

  // Move Y stepper in the -ve direction till the limits is reached. This is due to space constraints. Moving in +ve direction reach the limit would cause crashing into the chamber
  do
  {
    stepperY.move(-100);
    stepperY.run();
    _yLim_1 = digitalRead(Y_limit_1);
    _yLim_2 = digitalRead(Y_limit_2);
  }while(_yLim_1 && _yLim_2);
  
  
  if(!_yLim_1)
    // If lim2 is reached when stepping in the -ve direction
  {
    // Store the position as one limit
    Y_steps_lower = stepperY.currentPosition();
    _yFlip = LOW;
  }
  // If lim1 is reached when stepping in +ve direction
  else if(!_yLim_2)
  {  
    // Store the position as one limit 
    Y_steps_upper = stepperY.currentPosition();
    _yFlip = HIGH;
    
  }

  



  Y_home = microSteps_Y*fullStepsToYhome;
  stepperY.setCurrentPosition(0);
  stepperY.move(Y_home);

  while(stepperY.currentPosition() != Y_home)
  {
    stepperY.run();
  }
 

  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperZ.setCurrentPosition(0);

  microSteps_Y=microSteps_old;
  setMS123(microSteps_Y,'Y');
  stepperY.setMaxSpeed(stepperSpeedY);
}

  

//-------------------------------------------------------------------------------
//                               Limit Switch Function
//-------------------------------------------------------------------------------

void readLimitSwitch()
{
  if(_xFlip)
  {
    _xLimPos = digitalRead(X_limit_1);
    _xLimNeg = digitalRead(X_limit_2);
  }

  else
  {
    _xLimPos = digitalRead(X_limit_2);
    _xLimNeg = digitalRead(X_limit_1);
  }

  if(_yFlip)
  {
    _yLimPos = digitalRead(Y_limit_1);
    _yLimNeg = digitalRead(Y_limit_2);
  }

  else
  {
    _yLimPos = digitalRead(Y_limit_2);
    _yLimNeg = digitalRead(Y_limit_1);
  }
}


//---------------------------------------------------------------------------------------------------------------  
// Returns the maximum Speed of the steppers based on the microstep configuration
//---------------------------------------------------------------------------------------------------------------

int microstep_to_manualSpeed(int microSteps, int maxManualSpeed)
{
  if (microSteps == 1)
  {
    return maxManualSpeed;          //The speed is maxManualSpeed
  }
  else if (microSteps == 2) 
  {
    return int(1.2*maxManualSpeed); //The speed has been multiplied by 1.2/2
  }
  else if (microSteps == 4)        
  {
    return int(2*maxManualSpeed);   //The speed has been multiplied by 2/4
  }
  else if (microSteps == 8)
  {
    return int(3*maxManualSpeed);   //The speed has been multiplied by 3/8
  }
  else if (microSteps == 16)
  {
    return int(5*maxManualSpeed);   //The speed has been multiplied by 5/16
  }
}

//int microstep_to_manualSpeed(int microSteps, int maxManualSpeed)
//{
//  if (microSteps == 1)
//  {
//    return maxManualSpeed;          //The speed is maxManualSpeed
//  }
//  else if (microSteps == 2) 
//  {
//    return int(2*maxManualSpeed); //The speed has been multiplied by 2/2
//  }
//  else if (microSteps == 4)        
//  {
//    return int(4*maxManualSpeed);   //The speed has been multiplied by 4/4
//  }
//  else if (microSteps == 8)
//  {
//    return int(6*maxManualSpeed);   //The speed has been multiplied by 6/8
//  }
//  else if (microSteps == 16)
//  {
//    return int(8*maxManualSpeed);   //The speed has been multiplied by 10/16
//  }
//}
//---------------------------------------------------------------------------------------------------------------  
// For a speed given by the compute, return the most suitable microstep
//---------------------------------------------------------------------------------------------------------------

int autoSpeed_to_microstep(int autoSpeed,int maxManualSpeed){
  if (autoSpeed>=0 && autoSpeed< 5./16.*maxManualSpeed){
    return 16;
  }
  else if (autoSpeed>=5./16.*maxManualSpeed && autoSpeed<3./8.*maxManualSpeed ){
    return 8;
  }
  else if (autoSpeed>=3./8.*maxManualSpeed && autoSpeed<2./4.*maxManualSpeed ){
    return 4;
  }
  else if (autoSpeed>=2./4.*maxManualSpeed && autoSpeed<1.2/2.*maxManualSpeed ){
    return 2;
  }
  else{
    return 1;
  }
}


//-------------------------------------------------------------------------------
// Returns the number of fractional steps per 1 full step of the motor shaft
//-------------------------------------------------------------------------------

int MS123_to_microstepp(bool MS1, bool MS2, bool MS3)   //microSteppingLUT
{

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

//-------------------------------------------------------------------------------
// Update the pin and the msi variable for a given microstepping and a given motor
//-------------------------------------------------------------------------------

void setMS123(int microstepp,char motor){
  bool MS1,MS2,MS3;
  if (microstepp==1){
    MS1 = LOW; MS2 = LOW; MS3 = LOW;
  }
  else if (microstepp==2){
    MS1 = HIGH; MS2 = LOW; MS3 = LOW;  
  }
  else if (microstepp==4){
    MS1 = LOW; MS2 = HIGH; MS3 = LOW; 
  }
  else if (microstepp==8){
    MS1 = HIGH; MS2 = HIGH; MS3 = LOW; 
  }
  else if (microstepp==16){
    MS1 = HIGH; MS2 = HIGH; MS3 = HIGH;
  }

  if (motor=='X'){
    microSteps_X=microstepp;
    ms1_X = MS1; ms2_X = MS2; ms3_X = MS3;
    digitalWrite(MS1_X, ms1_X);digitalWrite(MS2_X, ms2_X);digitalWrite(MS3_X, ms3_X);
  }
  else if (motor=='Y'){
    microSteps_Y=microstepp;
    ms1_Y = MS1; ms2_Y = MS2; ms3_Y = MS3;
    digitalWrite(MS1_Y, ms1_Y);digitalWrite(MS2_Y, ms2_Y);digitalWrite(MS3_Y, ms3_Y);
  }
  else if (motor=='Z'){
    microSteps_Z=microstepp;
    ms1_Z = MS1; ms2_Z = MS2; ms3_Z = MS3;
    digitalWrite(MS1_Z, ms1_Z);digitalWrite(MS2_Z, ms2_Z);digitalWrite(MS3_Z, ms3_Z);
  }
}

//-------------------------------------------------------------------------------
//              Function for the joystick's sensitivity slider (X & Z)
//-------------------------------------------------------------------------------


void setMicroStepsJoystick(int joystickSensitivity)
{
  //---------------------------------------------------------------------------------------------------------------
  // Set the desired microstepping configurations for the steppers based on position of the analog speed control.
  // Write these values to the DigitalPins
  // Update the microStep variables
  //---------------------------------------------------------------------------------------------------------------  
  if(joystickSensitivity < adc_depth/5.0)
  { 
   microSteps_X = 16;
   microSteps_Z = 16;
  }
  else if (joystickSensitivity > adc_depth/5.0 && joystickSensitivity<=adc_depth*(2/5.0))
  { 
   microSteps_X = 8;
   microSteps_Z = 8; 
  }
  else if (joystickSensitivity > adc_depth*(2/5.0) && joystickSensitivity<=adc_depth*(3/5.0))
  {
   microSteps_X = 4;
   microSteps_Z = 4;
  }  
  else if (joystickSensitivity > adc_depth*(3/5.0) && joystickSensitivity<=adc_depth*(4/5.0))
  {
   microSteps_X = 2;
   microSteps_Z = 2; 
  }  
  else if (joystickSensitivity > adc_depth*(4/5.0) && joystickSensitivity <=adc_depth)
  {
   microSteps_X = 1;
   microSteps_Z = 1; 
  }
  
   setMS123(microSteps_X,'X');
   setMS123(microSteps_Z,'Z');
}

  //-------------------------------------------------------------------------------
//              Function for the Y Focus's sensitivity slider (Y)
//-------------------------------------------------------------------------------


void setMicroStepsYfocus(int YfocusSensitivity)
{
  //---------------------------------------------------------------------------------------------------------------
  // Set the desired microstepping configurations for the steppers based on position of the analog speed control.
  // Write these values to the DigitalPins
  // Update the microStep variables
  //---------------------------------------------------------------------------------------------------------------  
  if(YfocusSensitivity < adc_depth/5.0)
  { 
   microSteps_Y = 16;
  }
  else if (YfocusSensitivity > adc_depth/5.0 && YfocusSensitivity<=adc_depth*(2/5.0))
  { 
   microSteps_Y = 8;
  }
  else if (YfocusSensitivity > adc_depth*(2/5.0) && YfocusSensitivity<=adc_depth*(3/5.0))
  {
   microSteps_Y = 4;
  }  
  else if (YfocusSensitivity > adc_depth*(3/5.0) && YfocusSensitivity<=adc_depth*(4/5.0))
  {
   microSteps_Y = 2;
  }  
  else if (YfocusSensitivity > adc_depth*(4/5.0) && YfocusSensitivity <=adc_depth)
  {
   microSteps_Y = 1;
  }
  
   setMS123(microSteps_Y,'Y');

}

//-------------------------------------------------------------------------------
//        Interrupt service routines for the motor's quadrature encoder
//-------------------------------------------------------------------------------

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

  if(StageLocked == false)
  {
  if(!isYtracking || ManualMode){
    // Step the Focus stage Stepper by a small number of steps
    if(DirInput > 0)
    {
      stepperY.move(50*_yLimPos);
    }
    else if (DirInput < 0)
    {  
      stepperY.move(-50*_yLimNeg);
    } 
    stepperY.setMaxSpeed(stepperSpeedY);
    stepperY.run();
  }
  }
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


// https://www.nongnu.org/avr-libc/user-manual/group__util__crc.html
/*
uint16_t crc_ccitt_update (uint16_t crc, uint8_t data)
{
  data ^= lo8 (crc);
  data ^= data << 4;
  return ((((uint16_t)data << 8) | hi8 (crc)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}
*/

uint16_t crc_ccitt (byte* data_array, int data_length)
{
  uint16_t crc = 0xffff;
  for (int i=0; i <= data_length; i++){
    uint8_t data = data_array[i];
    data ^= (crc%256); // data ^= (crc&0xff);
    data ^= data << 4;
    crc = (((uint16_t)data << 8) | (crc >> 8)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3);
  }
  return crc;
}

/*
uint16_t crc_xmodem (byte* data_array, uint8_t data_length)
{
  uint16_t crc = 0x0;
  for (int i=0; i <= data_length; i++){
    data = data_array[i];
    int j;
    crc = crc ^ ((uint16_t)data << 8);
    for (j=0; j<8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}
*/



    
//-------------------------------------------------------------------------------
//                                 Timer for liquid lens and camera trigger
//-------------------------------------------------------------------------------

void liquid_lens_handler_timer_500us(){
  
  // update phase
  phase += 2*pi*(liquidLensSweepFrequency*timerPeriod/1000000);
  if (phase > 2*pi) {
    phase -= 2*pi;
  }
  // update phase_code
  phase_code =(int) round(65535*(phase)/(2*pi));
  
  liquidLensV = liquidLensVOffset + liquidLensVAmplitude*sin(phase);
  liquidLensVCode = liquidLensV*22.5828 - 551.0199;
  //liquidLensVCode = liquidLensVOffset*10 + liquidLensVAmplitude*10*sin(phase);
  
  // translate phase into voltage and send it to the liquid lens driver
  if (true){
    Wire.beginTransmission(0x77); // transmit to device (0xEE)
    // Wire.write(byte(0xEE));
    Wire.write(byte(0x03));   // UserMode reg address
    Wire.write(byte(0x03));   // UserMode: ACTIVE = 1, SM = 1
    Wire.write(byte(liquidLensVCode<<6));   // OIS_LSB reg value 
    Wire.endTransmission();     // stop transmitting
  }
  if (true){
    Wire.beginTransmission(0x77); // transmit to device (0xEE)
    Wire.write(byte(0x08));   // LLV4 reg address
    Wire.write(byte(liquidLensVCode>>2));   // LLV4
    Wire.write(byte(0x02));   // Command
    Wire.endTransmission();     // stop transmitting
  }

  if(startTriggering){
    counter_timer++;
    if (counter_timer==numTimerCycles) {
      counter_timer = 0;
      digitalWrite(triggerPin,HIGH);
      phase_code_lastTrigger = phase_code;
      //timestamp_lastTrigger = millis();
      timestamp_lastTrigger = timestamp_lastTrigger+1; // repurposed as trigger counter
      sendData = true;
    }
    if (counter_timer==1) {
      digitalWrite(triggerPin,LOW);
    }
  }

  
   
  /*
  if (counter_timer == 0) {
    digitalWrite(triggerPin,HIGH);
    counter_timer++;
  }
  else if (counter_timer == 1) {
    digitalWrite(triggerPin,LOW);
    counter_timer++;
  }
  else if (counter_timer == numTimerCycles-1) {
    counter_timer = 0;
  }
  else {
    counter_timer++;
  }
  */
  
}
