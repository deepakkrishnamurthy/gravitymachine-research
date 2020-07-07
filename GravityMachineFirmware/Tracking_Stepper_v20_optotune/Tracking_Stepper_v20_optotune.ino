/********************************************************
  uController code for gravity machine
  Axes convention:
  X (Linear stage): +ve radially outwards at 3'O Clock Position 
  Y (Linear stage): +ve Away from the camera
  Z (Linear stage): +ve upwards from gravity
  Theta (Rotational stage): CW +ve
  -by Deepak Krishnamurthy & François BENOIT du REY
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
//----------------------------------------------------------------------------------------------------
// STEPPER PINS
//----------------------------------------------------------------------------------------------------
// Direction and Step outputs
//--------------------------------------------------
#define STEP_X 15
#define DIR_X 14
#define STEP_Y 25
#define DIR_Y 23
#define STEP_THETA 35
#define DIR_THETA 33
#define STEP_Z 45
#define DIR_Z 43
//--------------------------------------------------
// MicroStepping Pins
//--------------------------------------------------
// X axis: Along the plane of the wheel
//--------------------------------------------------
#define MS1_X 18
#define MS2_X 17
#define MS3_X 16
//--------------------------------------------------
// Y axis: along rotational axis of the wheel
//--------------------------------------------------
#define MS1_Y 31
#define MS2_Y 29
#define MS3_Y 27
//--------------------------------------------------
// Z- axis (Wheel) : anti-parallel to gravity vector 
//--------------------------------------------------
#define MS1_THETA 41
#define MS2_THETA 39
#define MS3_THETA 37
//--------------------------------------------------
// Z-spare : 
//--------------------------------------------------
#define MS1_Z 51
#define MS2_Z 49
#define MS3_Z 47
//----------------------------------------------------------------------------------------------------
// CONTROL-PANEL PINS
//----------------------------------------------------------------------------------------------------
//--------------------------------------------------
// Manual Input Pins
//--------------------------------------------------
// X axis - Linear Actuator
#define moveXpos 3     
#define moveXneg 2    
// Z axis - Linear Actuator
#define moveZpos 9     
#define moveZneg 10  
// Theta axis - Wheel 0
#define moveThetapos 5   
#define moveThetaneg 4   
#define triggerTrack 6
//--------------------------------------------------
// Manual pin
//--------------------------------------------------
#define ManualPin 13
//--------------------------------------------------
// Input encoder pin
//--------------------------------------------------
#define c_InputEncoderPinA 7            // Pin to read encoder channel A (Rotary Encoder for Z-stage)
#define c_InputEncoderPinB 8            // Pin to read encoder channel B (Rotary Encoder for Z-stage)
//--------------------------------------------------
// Buttons
//--------------------------------------------------
#define Z1 9
#define Z2 10
#define SP1 11
#define SP2 12
//--------------------------------------------------
// Speed control input pin (sliders)
//--------------------------------------------------
#define analogSpeedX A8
#define analogSpeedY A9
#define analogSpeedTheta A10
#define analogSpeedZ A11
//--------------------------------------------------
// Light Intensity PWM pin
//#define lightPWM 19
//--------------------------------------------------
//--------------------------------------------------
// Light Intensity Measurement
//--------------------------------------------------
//#define lightPin A0/
//----------------------------------------------------------------------------------------------------
// ENCODER-BUS PINS
//----------------------------------------------------------------------------------------------------
//=================================================================================
// Stage Encoder Pins
//=================================================================================
#define x_EncoderPinA 53            // Pin to read encoder channel A 
#define x_EncoderPinB 52            // Pin to read encoder channel B 

#define y_EncoderPinA 50            // Pin to read encoder channel A
#define y_EncoderPinB 48            // Pin to read encoder channel B

#define z_EncoderPinA 46            // Pin to read encoder channel A 
#define z_EncoderPinB 44            // Pin to read encoder channel B

#define theta_EncoderPinA 42            // Pin to read encoder channel A (Z encoder of the wheel)
#define theta_EncoderPinB 40            // Pin to read encoder channel B (Z encoder of the wheel)
//--------------------------------------------------
// Limit Switch
//--------------------------------------------------
//#define USE_ANALOG_LIMIT_SWITCHES
#define USE_DIGITAL_LIMIT_SWITCHES
#ifdef USE_ANALOG_LIMIT_SWITCHES
#define LIMIT_SWITCH_PINMODE INPUT

#define X_limit_1 A0   // -ve X end of the stepper
#define X_limit_2 A1    // +ve X end of the stepper

#define Y_limit_1 A2    // -ve Y end of the Stepper
#define Y_limit_2 A3    // +ve Y end of the stepper

#define Z_limit_1 A4    // -ve Z end of the Stepper
#define Z_limit_2 A5    // +ve Z end of the stepper
#else
#define LIMIT_SWITCH_PINMODE INPUT_PULLUP

#define X_limit_1 38   // -ve X end of the stepper
#define X_limit_2 36    // +ve X end of the stepper

#define Y_limit_1 34    // -ve Y end of the Stepper
#define Y_limit_2 32    // +ve Y end of the stepper

#define Z_limit_1 30    // -ve Z end of the Stepper
#define Z_limit_2 28    // +ve Z end of the stepper
#endif
//--------------------------------------------------
// Camera Trigger Pin
//--------------------------------------------------
# define triggerCamera 26
# define triggerCamera_FL 0
# define optotune_SYNC 19 //@@@
//# define trigger_indicator_Pin 50
# define sendSerial_indicator_pin 24
//# define trigger_indicator_Pin_GND 52
//# define sendSerial_indicator_pin_GND 53
//--------------------------------------------------
// Serial communication
//--------------------------------------------------
# define CMD_LENGTH 18
# define DATA_TX_LENGTH 27
//=================================================================================
// Stage Movement/Physical Variables
//=================================================================================
//--------------------------------------------------
// Common Physical Variables
//--------------------------------------------------
int adc_depth = 1023;                                        // Speed Slider: Bit depth of the ADC
int ManualSampleTime = 500;                                  // Speed Slider:Interval at which the joystick sensitivity is monitored (milliseconds)
int limitThreshold = 1.8 / 3.3 * 1023;

long int CurrPos_X = 0,CurrPos_Y = 0, CurrPos_Z = 0, CurrPos_Theta = 0;         // Current Position as measured by the Stepper in Steps
long int CurrPos_X_code = 0,CurrPos_Y_code = 0, CurrPos_Z_code = 0, CurrPos_Theta_code = 0;         // Current Position as measured by the Stepper in Steps
long int CurrPos_X_Stepper = 0, CurrPos_Y_Stepper = 0, CurrPos_Theta_Stepper = 0, CurrPos_Z_Stepper = 0;
long int PrevPos_X_Stepper = 0, PrevPos_Y_Stepper = 0, PrevPos_Theta_Stepper = 0, PrevPos_Z_Stepper = 0;
long int PosStart_X = 0, PosStart_Y = 0, PosStart_Z = 0, PosStart_Theta = 0;

float DeltaT=0, Homing=0; //Data from the computer

// Steps to move (command from the computer)
float Step_X = 0, Step_Y = 0, Step_Theta = 0, Step_Z = 0;
// Steps to move (based on Manual input)
int TargetCurr_X = 0, TargetCurr_Y = 0, TargetCurr_Z = 0, TargetCurr_Theta = 0, TargetPrev_X = 0, TargetPrev_Y = 0, TargetPrev_Z = 0;            // Target required in Steps 
bool ms1_X,ms2_X,ms3_X,ms1_Y,ms2_Y,ms3_Y,ms1_Z,ms2_Z,ms3_Z, ms1_Theta, ms2_Theta, ms3_Theta;          // State of microstepping pins
int microSteps_X = 1, microSteps_Y = 1, microSteps_Z = 1, microSteps_Theta = 1;            // no:of fractional steps per full step
int microSteps_Old_X=1, microSteps_Old_Y = 1, microSteps_Old_Z = 1;
//--------------------------------------------------
// Variables to store lower and upper limits of the stage:
//--------------------------------------------------

long int X_steps_lower=0, X_steps_upper = 0,Y_steps_lower=0, Y_steps_upper = 0,Z_steps_lower=0, Z_steps_upper = 0;
long int X_home = 0, Y_home = 0, Z_home = 0;
//--------------------------------------------------
// Maximum speed of stage in Manual mode
//--------------------------------------------------
int maxManualSpeedX = 5000, maxManualSpeedY = 2000, maxManualSpeedTheta = 5000, maxManualSpeedZ = 1000; 
//--------------------------------------------------
// Speed of stepper based on micro-stepping state
//--------------------------------------------------
int stepperSpeedX, stepperSpeedY, stepperSpeedZ, stepperSpeedTheta;

int maxAcclX, maxAcclY, maxAcclZ, maxAcclTheta;                           // Maximum acceleration of X,Y,Z steppers in steps/s^2

int Tracking = 0;
// Set the stage speed of X and Theta stage
int joystickSensitivity_X,joystickSensitivity_X_Prev;
// Set the stage speed of X and Theta stage
int joystickSensitivity_Theta,joystickSensitivity_Theta_Prev;
// Set the stage speed of focus (Y) stage
int YfocusSensitivity=0,YfocusSensitivityPrev=0;
bool joystickSensitivityChange=LOW;
bool joystickSensitivity_X_Change=LOW;
bool YfocusSensitivityChange=LOW;
bool sensitivityChange=LOW;
//=================================================================================
// State Variables
//=================================================================================
bool Xpos, Xneg, Ypos, Yneg, Zpos, Zneg, Thetapos, Thetaneg, ManualInput, ManualMode = LOW, ManualModePrev = LOW;
// State variables for stage limit switches
bool  _xLim_1=LOW, _xLim_2 = LOW, _yLim_1 = LOW, _yLim_2 = LOW, _zLim_1 = LOW, _zLim_2 = LOW, _xLim = LOW;
bool _xLim_1_prev = LOW, _xLim_2_prev = LOW, _yLim_1_prev = LOW, _yLim_2_prev = LOW,_zLim_1_prev = LOW,_zLim_2_prev = LOW;
bool _xFlip = LOW, _yFlip=LOW, _zFlip = LOW, _xLimPos=LOW, _xLimNeg = LOW, _yLimPos=LOW, _yLimNeg = LOW, _zLimPos=LOW, _zLimNeg = LOW ;


volatile bool x_EncoderASet;
volatile bool x_EncoderBSet;
volatile bool x_EncoderAPrev;
volatile bool x_EncoderBPrev;
volatile long int x_EncoderTicks = 0;
volatile long int x_EncoderTicks_code = 0;

volatile bool y_EncoderASet;
volatile bool y_EncoderBSet;
volatile bool y_EncoderAPrev;
volatile bool y_EncoderBPrev;
volatile long int y_EncoderTicks = 0;
volatile long int y_EncoderTicks_code = 0;

volatile bool theta_EncoderASet;
volatile bool theta_EncoderBSet;
volatile bool theta_EncoderAPrev;
volatile bool theta_EncoderBPrev;
volatile long int theta_EncoderTicks = 0;
volatile long int theta_EncoderTicks_code = 0;

volatile bool z_EncoderASet;
volatile bool z_EncoderBSet;
volatile bool z_EncoderAPrev;
volatile bool z_EncoderBPrev;
volatile long int z_EncoderTicks = 0;
volatile long int z_EncoderTicks_code = 0;

volatile bool _InputEncoderASet;
volatile bool _InputEncoderBSet;
volatile bool _InputEncoderAPrev;
volatile bool _InputEncoderBPrev;
volatile long _InputEncoderTicks = 0;

volatile int x_Dir=0, y_Dir=0, z_Dir=0, theta_Dir=0, DirInput=0, ErrorCount=0;

//=================================================================================
// Liquid Lens variable
//=================================================================================

bool isYtracking = false;
volatile float liquidLensSweepFrequency = 2.0;
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
// Triggering parameters (Tracking camera)
//=================================================================================
int fps = 120;
int numTimerCycles = 1000000/fps/timerPeriod;
volatile int counter_timer = -1;

volatile unsigned long prevMillisSend = 0, currMillisSend = 0, prevMillisRec = 0, currMillisRec = 0, currMillisStage=0, prevMillisStage = 0, MillisStick = 0, currMillis = 0, currMillisLight = 0;
volatile unsigned long timestamp_lastTrigger = 0;
bool startTriggering = false;
//=================================================================================
// Triggering parameters (FL, High speed camera etc.)
//=================================================================================
long int sample_interval_FL = 1; // Sample interval in 1/100 seconds
long int sample_interval_FL_prev = sample_interval_FL;
long int numTimerCycles_FL = 1000000*sample_interval_FL/(100*timerPeriod);
volatile long int counter_timer_FL = -1;
bool startTriggering_FL = false;
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
// THETA stepper
AccelStepper stepperTHETA(AccelStepper::DRIVER, STEP_THETA, DIR_THETA);     
// Z stepper
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
  SerialUSB.begin (2000000);
  
  while(!SerialUSB); //Wait until connection is established
  buffer_rx_ptr=0;
  int a = 1;
  //SerialUSB.println(1,DEC); 
  SerialUSB.write(a);
  int init=0;
  
  while (init!=2)        // Wait for command 'a' to be sent from the host computer.
  {
    init=int(SerialUSB.read());
  }
  SerialUSB.write(init);
 //===============================================================================
  // Serial Communication for debugging
  //===============================================================================

//  Serial.begin(115200);
  
  //===============================================================================
  // Setup User Input Pins
  //===============================================================================
  pinMode(moveXpos, INPUT_PULLUP); pinMode(moveXneg, INPUT_PULLUP);
  
  pinMode(moveThetapos, INPUT_PULLUP); pinMode(moveThetaneg, INPUT_PULLUP);

  pinMode(triggerTrack, INPUT_PULLUP);
  
  //-------------------------------------------------------------------------------
  // Set up the DIRECTION pins as OUTPUT
  //-------------------------------------------------------------------------------
  pinMode(DIR_X, OUTPUT); digitalWrite(DIR_X, LOW);
  pinMode(DIR_Y, OUTPUT); digitalWrite(DIR_Y, LOW);
  pinMode(DIR_THETA, OUTPUT); digitalWrite(DIR_THETA, LOW);
  pinMode(DIR_Z, OUTPUT); digitalWrite(DIR_Z, LOW);
  //-------------------------------------------------------------------------------
  // Set up the STEP pins as OUTPUT
  //-------------------------------------------------------------------------------
  pinMode(STEP_X, OUTPUT); digitalWrite(STEP_X, LOW);
  pinMode(STEP_Y, OUTPUT); digitalWrite(STEP_Y, LOW);
  pinMode(STEP_THETA, OUTPUT); digitalWrite(STEP_THETA, LOW);
  pinMode(STEP_Z, OUTPUT); digitalWrite(STEP_Z, LOW);
  

  pinMode(MS1_X, OUTPUT); pinMode(MS2_X, OUTPUT); pinMode(MS3_X, OUTPUT); 
  pinMode(MS1_Y, OUTPUT); pinMode(MS2_Y, OUTPUT); pinMode(MS3_Y, OUTPUT);
  pinMode(MS1_THETA, OUTPUT); pinMode(MS2_THETA, OUTPUT); pinMode(MS3_THETA, OUTPUT);  
  pinMode(MS1_Z, OUTPUT); pinMode(MS2_Z, OUTPUT); pinMode(MS3_Z, OUTPUT); 
  //===============================================================================
  // Setup Limit Switch Pins
  //===============================================================================
  pinMode(X_limit_1,LIMIT_SWITCH_PINMODE); pinMode(X_limit_2,LIMIT_SWITCH_PINMODE);
  pinMode(Y_limit_1,LIMIT_SWITCH_PINMODE); pinMode(Y_limit_2,LIMIT_SWITCH_PINMODE);
  pinMode(Z_limit_1,LIMIT_SWITCH_PINMODE); pinMode(Z_limit_2,LIMIT_SWITCH_PINMODE);
  //===============================================================================
  // Setup Camera Trigger Pin
  //===============================================================================
  pinMode(triggerCamera,OUTPUT);
  digitalWrite(triggerCamera,LOW);
  pinMode(triggerCamera_FL,OUTPUT);
  digitalWrite(triggerCamera_FL,LOW);
  //===============================================================================
  // liquid lens SYNC pin
  //===============================================================================
  pinMode(optotune_SYNC,INPUT);
  //===============================================================================
  // Setup Indicator Pin
  //===============================================================================
//  pinMode(trigger_indicator_Pin,OUTPUT);
//  digitalWrite(trigger_indicator_Pin,LOW);
//  pinMode(sendSerial_indicator_pin,OUTPUT);
//  digitalWrite(sendSerial_indicator_pin,LOW);
//  pinMode(trigger_indicator_Pin_GND,OUTPUT);
//  digitalWrite(trigger_indicator_Pin_GND,LOW);
//  pinMode(sendSerial_indicator_pin_GND,OUTPUT);
//  digitalWrite(sendSerial_indicator_pin_GND,LOW);
    
  //-------------------------------------------------------------------------------
  // Set the initial microstepping
  //-------------------------------------------------------------------------------
  microSteps_X = 16;
  microSteps_Y = 16;
  microSteps_Theta = 16;
  microSteps_Z = 16;

  //-------------------------------------------------------------------------------
  // actualise local variable ms1_X (resp. Y & Z) ms2.. ms3.. and write in the pin MS1.. MS2.. MS3..
  //-------------------------------------------------------------------------------
  setMS123(microSteps_X,'X');
  setMS123(microSteps_Y,'Y');
  setMS123(microSteps_Z,'Z');
  setMS123(microSteps_Theta,'T');

  //-------------------------------------------------------------------------------
  // Set the initial speed and the MAX acceleration of all the steppers (Speeds > 1000 full-steps per second are unreliable)
  //-------------------------------------------------------------------------------
  
  stepperSpeedX = microstep_to_manualSpeed(microSteps_X, maxManualSpeedX); 
  stepperSpeedY = microstep_to_manualSpeed(microSteps_Y, maxManualSpeedY); 
  stepperSpeedTheta = microstep_to_manualSpeed(microSteps_Theta, maxManualSpeedTheta); 
  stepperSpeedZ = microstep_to_manualSpeed(microSteps_Z, maxManualSpeedZ); 

  maxAcclX=1000; maxAcclY=1000; maxAcclZ=1000, maxAcclTheta=1000;

  stepperX.setMaxSpeed(stepperSpeedX);
  stepperX.setAcceleration(maxAcclX);
   
  stepperY.setMaxSpeed(stepperSpeedY);
  stepperY.setAcceleration(maxAcclY);

  stepperZ.setMaxSpeed(stepperSpeedZ);
  stepperZ.setAcceleration(maxAcclZ);

  stepperTHETA.setMaxSpeed(stepperSpeedTheta);
  stepperTHETA.setAcceleration(maxAcclTheta);
  

  //-------------------------------------------------------------------------------
  // Quadrature Encoder pin setup
  //-------------------------------------------------------------------------------
  pinMode(x_EncoderPinA, INPUT_PULLUP);
  pinMode(x_EncoderPinB, INPUT_PULLUP);

  pinMode(y_EncoderPinA, INPUT_PULLUP);
  pinMode(y_EncoderPinB, INPUT_PULLUP);

  pinMode(z_EncoderPinA, INPUT_PULLUP);
  pinMode(z_EncoderPinB, INPUT_PULLUP);

  pinMode(theta_EncoderPinA, INPUT);
  pinMode(theta_EncoderPinB, INPUT);
  
  pinMode(c_InputEncoderPinA, INPUT);        // sets pin A as input
  pinMode(c_InputEncoderPinB, INPUT);        // sets pin B as input
  
  digitalWrite(c_InputEncoderPinA, HIGH);
  digitalWrite(c_InputEncoderPinB, HIGH);
  
 
  attachInterrupt(digitalPinToInterrupt(x_EncoderPinA), HandleMotorInterrupt_x, CHANGE);
  attachInterrupt(digitalPinToInterrupt(y_EncoderPinA), HandleMotorInterrupt_y, CHANGE);
  attachInterrupt(digitalPinToInterrupt(theta_EncoderPinA), HandleMotorInterrupt_theta, CHANGE);
  attachInterrupt(digitalPinToInterrupt(z_EncoderPinA), HandleMotorInterrupt_z, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(c_InputEncoderPinA), HandleInputEncoderInterrupt, CHANGE);

  attachInterrupt(digitalPinToInterrupt(optotune_SYNC), HandleOptotuneSYNCInterrupt, RISING);

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
  //pinMode(lightPWM, OUTPUT);

  delay(1000);

  // initialize timer
  Timer3.attachInterrupt(liquid_lens_handler_timer_500us);
  Timer3.start(timerPeriod); // Calls every 500 us
  startTriggering = true;

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
  currMillisRec = millis();
  sensitivityChange=LOW;


  //-------------------------------------------------------------------------------
  // Read the sensitivity of the joystick at a fixed frequency
  //-------------------------------------------------------------------------------
  
  currMillisStage = millis() - currMillisStage;
   if(currMillisStage >= ManualSampleTime  && StageLocked == false)
    {      
      #ifdef USE_ANALOG_LIMIT_SWITCHES
      readLimitSwitchAnalog();
      #else
      readLimitSwitchDigital();
      #endif
      
      // Read the desired stage speed from the analog Slider
      joystickSensitivity_X = analogRead(analogSpeedX);
      joystickSensitivity_Theta = analogRead(analogSpeedTheta);
  
      if (joystickSensitivity_X != joystickSensitivity_X_Prev || joystickSensitivity_Theta != joystickSensitivity_Theta_Prev )
      {
        // Set the microstepping mode based on the stage speed
        setMicroStepsJoystick(joystickSensitivity_X, 'X');
        setMicroStepsJoystick(joystickSensitivity_Theta, 'T');
        joystickSensitivity_X_Prev = joystickSensitivity_X;
        joystickSensitivity_Theta_Prev = joystickSensitivity_Theta;
  
        // Set the speed in manual Mode based on the microstep mode
        stepperSpeedX = microstep_to_manualSpeed(microSteps_X, maxManualSpeedX);  
        stepperSpeedZ = microstep_to_manualSpeed(microSteps_Z, maxManualSpeedZ); 
        stepperSpeedTheta = microstep_to_manualSpeed(microSteps_Theta, maxManualSpeedTheta); 
  
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

//      lightMeasured = analogRead(lightPin);
    }

  
  //-------------------------------------------------------------------------------
  // reset the sensitivity of the joystick on 16 step when go to autoMode
  //-------------------------------------------------------------------------------
    
  if (ManualMode==LOW and ManualModePrev==HIGH  && StageLocked == false){
    microSteps_X=16;
    microSteps_Theta=16;
    microSteps_Z=16;
    microSteps_Y=16;
    setMS123(microSteps_X,'X');
    setMS123(microSteps_Y,'Y');
    setMS123(microSteps_Z,'Z');
    setMS123(microSteps_Theta,'T');
    stepperSpeedX = microstep_to_manualSpeed(microSteps_X, maxManualSpeedX);
    stepperSpeedY = microstep_to_manualSpeed(microSteps_Y, maxManualSpeedX);
    stepperSpeedZ = microstep_to_manualSpeed(microSteps_Z, maxManualSpeedX);
    stepperSpeedTheta = microstep_to_manualSpeed(microSteps_Theta, maxManualSpeedTheta);
    sensitivityChange=HIGH;
  }
    
  //-------------------------------------------------------------------------------
  // set the speed in accordance to the microstepping if it has changed
  //-------------------------------------------------------------------------------

  if (sensitivityChange){
    stepperX.setMaxSpeed(stepperSpeedX);
    stepperY.setMaxSpeed(stepperSpeedY);
    stepperZ.setMaxSpeed(stepperSpeedZ);
    stepperTHETA.setMaxSpeed(stepperSpeedTheta);
  }

  //-------------------------------------------------------------------------------
  // Manual Input Block
  //-------------------------------------------------------------------------------
  if(ManualMode  && StageLocked == false)
  {

    Thetapos = !digitalRead(moveThetapos);
    Thetaneg = !digitalRead(moveThetaneg);

    Xpos = !digitalRead(moveXpos);
    Xneg = !digitalRead(moveXneg);

    // To be used when a Z linear stage is added
//    Zpos = digitalRead(moveZpos);
//    Zneg = digitalRead(moveZneg);

//    TargetCurr_X = -(Xpos+(-1)*Xneg)*100;
    TargetCurr_X = -(Xpos*(_xLimPos)+(-1)*Xneg*(_xLimNeg))*100;
    // TargetCurr_Z = (Zpos+(-1)*Zneg)*100;
    TargetCurr_Theta = -(Thetapos+(-1)*Thetaneg)*100;

    // stepperZ.move(microSteps_Z * TargetCurr_Z );
    stepperTHETA.move(microSteps_Theta * TargetCurr_Theta);
    stepperX.move(microSteps_X * TargetCurr_X );
  }


  //-------------------------------------------------------------------------------
  // Update the Stepper Position 
  //-------------------------------------------------------------------------------
  // Current position is stored in terms of no:of fractional steps (1/16)
  CurrPos_X_Stepper = stepperX.currentPosition();
  CurrPos_Y_Stepper = stepperY.currentPosition();
  CurrPos_Theta_Stepper = stepperTHETA.currentPosition();
  
  // CurrPos_Z_Stepper = stepperZ.currentPosition();

  CurrPos_X += (CurrPos_X_Stepper - PrevPos_X_Stepper)*16/microSteps_X;
  CurrPos_Y += (CurrPos_Y_Stepper - PrevPos_Y_Stepper)*16/microSteps_Y;
  CurrPos_Theta += (CurrPos_Theta_Stepper - PrevPos_Theta_Stepper)*16/microSteps_Theta;
  // CurrPos_Z += (CurrPos_Z_Stepper - PrevPos_Z_Stepper)*16/microSteps_Z;
  
  PrevPos_X_Stepper = CurrPos_X_Stepper;
  PrevPos_Y_Stepper = CurrPos_Y_Stepper;
  PrevPos_Theta_Stepper = CurrPos_Theta_Stepper;
  // PrevPos_Z_Stepper = CurrPos_Z_Stepper;
  
  //-------------------------------------------------------------------------------
  // Serial sending block (Send data to computer)
  //-------------------------------------------------------------------------------

  if (sendData){
    buffer_tx[0] = byte(phase_code_lastTrigger%256);
    buffer_tx[1] = byte(phase_code_lastTrigger>>8);
    
    // When only open-loop position is available
//    CurrPos_X_code = CurrPos_X;  //right sens of the motor
    // When a closed-loop encoder is implemented
    CurrPos_X_code = x_EncoderTicks;  //right sens of the motor
    if (CurrPos_X_code>0){
      buffer_tx[2] = byte(int(0));
    }
    else{
      buffer_tx[2] = byte(int(1));
      CurrPos_X_code = -CurrPos_X_code;
    }
    buffer_tx[3] = byte(CurrPos_X_code>>24);
    buffer_tx[4] = byte(CurrPos_X_code>>16);
    buffer_tx[5] = byte(CurrPos_X_code>>8);
    buffer_tx[6] = byte(CurrPos_X_code%256);

    // CurrPos_Y_code= CurrPos_Y;  //right sens of the motor
    CurrPos_Y_code = y_EncoderTicks;

    if (CurrPos_Y_code>0){
      buffer_tx[7] = byte(int(0));
    }
    else{
      buffer_tx[7] = byte(int(1));
      CurrPos_Y_code = -CurrPos_Y_code;
    }
    buffer_tx[8]  = byte(CurrPos_Y_code>>24);
    buffer_tx[9]  = byte(CurrPos_Y_code>>16);
    buffer_tx[10] = byte(CurrPos_Y_code>>8);
    buffer_tx[11] = byte(CurrPos_Y_code%256);
    
    
    CurrPos_Theta_code=theta_EncoderTicks;
    if(CurrPos_Theta_code>0) 
    {
      buffer_tx[12] = byte(int(0));
      
    }
    else 
    {
      buffer_tx[12] = byte(int(1));
      CurrPos_Theta_code = -CurrPos_Theta_code;
    }
    buffer_tx[13] = byte(CurrPos_Theta_code>>24);
    buffer_tx[14] = byte(CurrPos_Theta_code>>16);
    buffer_tx[15] = byte(CurrPos_Theta_code>>8);
    buffer_tx[16] = byte(CurrPos_Theta_code%256);

    buffer_tx[17] = byte(ManualMode);

    // light intensity
    buffer_tx[18] = byte(lightMeasured%256);
    buffer_tx[19] = byte(lightMeasured>>8);

    // time stamp
    buffer_tx[20] = byte(timestamp_lastTrigger>>24);
    buffer_tx[21] = byte(timestamp_lastTrigger>>16);
    buffer_tx[22] = byte(timestamp_lastTrigger>>8);
    buffer_tx[23] = byte(timestamp_lastTrigger%256);

    // tracking trigger
    buffer_tx[24] = byte(!digitalRead(triggerTrack));
    
    // 16 bit crc
    calculateCRC_tx = false;
    if(calculateCRC_tx) {
      uint16_t crc = crc_ccitt(buffer_tx,24);
      // uint16_t crc = crc_xmodem(buffer_tx,24);
      buffer_tx[25] = byte(crc>>8);
      buffer_tx[26] = byte(crc%256);
    }
    else{
      buffer_tx[0] = 0;
      buffer_tx[0] = 0;
    }    

    digitalWrite(sendSerial_indicator_pin,HIGH);
    SerialUSB.write(buffer_tx,DATA_TX_LENGTH);
    digitalWrite(sendSerial_indicator_pin,LOW);

    sendData=false;
  }

  //-------------------------------------------------------------------------------
  // Serial print (for debugging)
  //-------------------------------------------------------------------------------
//  Serial.print("X pos: ");
//  Serial.println(CurrPos_X);
//  Serial.print("X pos encoder: ");
//  Serial.println(x_EncoderTicks);
//  Serial.print("Y pos encoder: ");
//  Serial.println(y_EncoderTicks);
//  Serial.print("X encoder pin A:");
//  Serial.println(x_EncoderASet);
//  Serial.print("X encoder pin B:");
//  Serial.println(x_EncoderBSet);
//  Serial.print("Input encoder");
//  Serial.print(_InputEncoderTicks);
  //-------------------------------------------------------------------------------
  // Serial Receiving Block 
  //-------------------------------------------------------------------------------

  //Data reception: the data is read at the frequency of the computer
   
   if(SerialUSB.available()){
      buffer_rx_ptr=0;
      int cyclesElapsed = 0;
      while(buffer_rx_ptr < CMD_LENGTH ){ 
        buffer_rx[buffer_rx_ptr] = SerialUSB.read();
        buffer_rx_ptr++;
        // timeout:
        if(cyclesElapsed++>=10000)
          break;
      }

      //Data analysis: if the right message is read, lets compute the data and update arduino position
      isReceived=true;
      if (buffer_rx_ptr == CMD_LENGTH) {
 
        isReceived=true;

        // Holding freq fized (TESTING)
        liquidLensSweepFrequency = float(int(buffer_rx[0]) + int(buffer_rx[1]) *256)/100;
//        liquidLensVAmplitude = float(int(buffer_rx[2]) + int(buffer_rx[3]) *256)/1000*20/1.4;        //the amplitude is sent in mm, and is converted in V: 10V = 1.4 mm
//        liquidLensVOffset = float(int(buffer_rx[4]) + int(buffer_rx[5]) *256)/100;
        
        liquidLensOrder=int(buffer_rx[2]);
        
        if (liquidLensOrder==1 ){
          isYtracking=true;
        }
        else{
          isYtracking=false;
        }
        
        
        
        Homing = int(buffer_rx[3]);                                                                  
        Tracking=int(buffer_rx[4]);
        
        Step_X = float(int(buffer_rx[5])+(int(buffer_rx[6]) << 8));                                   // X steps to move  ATTENTION: We change the sens of the motor 
        if (Step_X>32767){
          Step_X=Step_X-65536;
        }
        Step_X = Step_X/100;                                                                   
         
        Step_Y = float(int(buffer_rx[7])+(int(buffer_rx[8]) << 8));
        if (Step_Y>32767){
          Step_Y=Step_Y-65536;
        }
        Step_Y = -Step_Y/100;
        

        Step_Theta = float(int(buffer_rx[9])+(int(buffer_rx[10]) << 8)); 
        if (Step_Theta > 32767){
          Step_Theta = Step_Theta - 65536;
        }
        Step_Theta= Step_Theta/100;
        
        DeltaT = float( int(buffer_rx[11])+(int(buffer_rx[12]) << 8))/10;                     // Delta T received in ms 

        lightOutput = int(buffer_rx[13]);              // Intensity sent to the LED        

        startTriggering_FL = int( buffer_rx[14]);

        
        sample_interval_FL_prev = sample_interval_FL;
        // Sampling interval for FL images (100*seconds)
        sample_interval_FL = int(buffer_rx[15] + int(buffer_rx[16]) << 8 + int(buffer_rx[17]) << 16);
        if(sample_interval_FL_prev != sample_interval_FL)
        {
          updateNumTimerCycles();
        }
        //-------------------------------------------------------------------------------
        // Automatic Input Block
        //-------------------------------------------------------------------------------
   
        // If an object is detected then move the stage based on the positional error signal
        if(ManualMode == LOW && Tracking > 0 && StageLocked == false)
        { 
          stepperTHETA.move((int) round(microSteps_Theta * Step_Theta));
          
          stepperX.move((int) round(microSteps_X * Step_X *((Step_X>0)*_xLimPos + (Step_X < 0) *_xLimNeg)));
          
          if (isYtracking)
          {
             stepperY.move((int) round(microSteps_Y * Step_Y * ((Step_Y>0)*_yLimPos + (Step_Y < 0) *_yLimNeg)));
                // Disabling limit switch control
//                 stepperY.move((int) round(microSteps_Y * Step_Y)); 

          }
        }
      }
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
           
           Y_home = y_sign * microSteps_Y*fullStepsToYhome;
           stepperY.move(Y_home);
           y_home_inProgress = true;

      }

      if(y_home_inProgress == true && stepperY.distanceToGo() == 0)
      {
          atYhome = true;
      }

//      atXhome = true;
      //If Homing is complete for both stages
      if(atXhome && atYhome)
      { 
          
          setStepperZero();
          setEncoderZero();
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
  //stepperZ.run()
  stepperTHETA.run();

  

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//-------------------------------------------------------------------------------
//                             End of the main loop
//-------------------------------------------------------------------------------
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// -------------------------------------------------------------------------------
//                 Update camera trigger and acquisition parameters
//-------------------------------------------------------------------------------

void updateNumTimerCycles()
{
  numTimerCycles_FL = 1000000*sample_interval_FL/(100*timerPeriod);
  
}


//-------------------------------------------------------------------------------
//                       Homing functions
//-------------------------------------------------------------------------------
// Reset the encoders position
void setEncoderZero()
{
    x_EncoderTicks = 0;
    y_EncoderTicks = 0;
    theta_EncoderTicks = 0;
    z_EncoderTicks = 0;
}

// Reset the current Stepper position to the new zero position
void setStepperZero()
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
    
    CurrPos_X = 0;
    CurrPos_Y = 0;
    CurrPos_Z = 0;
}

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
  
  #ifdef USE_ANALOG_LIMIT_SWITCHES
  readLimitSwitchAnalog();
  #else
  readLimitSwitchDigital();
  #endif
  _xLim_1 = _xLimNeg;
  _xLim_2 = _xLimPos;
    
   // If lim1 is reached when stepping in the +ve direction
  if(!_xLim_1 && _xLim_1_prev && reachedX1==false)
  {
      reachedX1 = true;
      x_sign = -x_sign;
      // Store the position as one limit
      X_steps_lower = stepperX.currentPosition();
     
  }
  else if(!_xLim_2 && _xLim_2_prev && reachedX2 == false)
  {
      reachedX2 = true;
      x_sign = -x_sign;
      X_steps_upper = stepperX.currentPosition();
    

  }

  if(reachedX1 && reachedX2)
  {   
      found_Xlimits = true;
  }

  stepperX.move(x_sign*400);
 



}

void Y_Stepper_FindLimits()
{
    _yLim_1_prev = _yLim_1;
    _yLim_2_prev = _yLim_2;
    
    #ifdef USE_ANALOG_LIMIT_SWITCHES
    readLimitSwitchAnalog();
    #else
    readLimitSwitchDigital();
    #endif
    _yLim_1 = _yLimNeg;
    _yLim_2 = _yLimPos;

    
    
    if(!_yLim_1 && _yLim_1_prev && reachedY1 == false)
    {
        reachedY1 = true;
        y_sign = -y_sign;
        // Store the position as one limit
       

    }
    else if(!_yLim_2 && _yLim_2_prev && reachedY2 == false)
    {
        reachedY2 = true;
        y_sign = -y_sign;
    }

    if(reachedY1 || reachedY2)
    {
      found_Ylimits = true;
    
    }
    stepperY.move(y_sign*400);

    

}
//-------------------------------------------------------------------------------
//                               Limit Switch Function
//-------------------------------------------------------------------------------

void readLimitSwitchDigital()
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

  if(_zFlip)
  {
    _zLimPos = digitalRead(Z_limit_1);
    _zLimNeg = digitalRead(Z_limit_2);
  }

  else
  {
    _zLimPos = digitalRead(Z_limit_2);
    _zLimNeg = digitalRead(Z_limit_1);
  }

}

void readLimitSwitchAnalog()
{
  if(_xFlip)
  {
    _xLimPos = analogRead(X_limit_1) > limitThreshold;
    _xLimNeg = analogRead(X_limit_2) > limitThreshold;
  }

  else
  {
    _xLimPos = analogRead(X_limit_2) > limitThreshold;
    _xLimNeg = analogRead(X_limit_1) > limitThreshold;
  }

  if(_yFlip)
  {
    _yLimPos = analogRead(Y_limit_1) > limitThreshold;
    _yLimNeg = analogRead(Y_limit_2) > limitThreshold;
  }

  else
  {
    _yLimPos = analogRead(Y_limit_2) > limitThreshold;
    _yLimNeg = analogRead(Y_limit_1) > limitThreshold;
  }

  if(_zFlip)
  {
    _zLimPos = analogRead(Z_limit_1) > limitThreshold;
    _zLimNeg = analogRead(Z_limit_2) > limitThreshold;
  }

  else
  {
    _zLimPos = analogRead(Z_limit_2) > limitThreshold;
    _zLimNeg = analogRead(Z_limit_1) > limitThreshold;
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

void setMS123(int microstep,char motor){
  bool MS1,MS2,MS3;
  if (microstep==1){
    MS1 = LOW; MS2 = LOW; MS3 = LOW;
  }
  else if (microstep==2){
    MS1 = HIGH; MS2 = LOW; MS3 = LOW;  
  }
  else if (microstep==4){
    MS1 = LOW; MS2 = HIGH; MS3 = LOW; 
  }
  else if (microstep==8){
    MS1 = HIGH; MS2 = HIGH; MS3 = LOW; 
  }
  else if (microstep==16){
    MS1 = HIGH; MS2 = HIGH; MS3 = HIGH;
  }

  if (motor=='X'){
    microSteps_X=microstep;
    ms1_X = MS1; ms2_X = MS2; ms3_X = MS3;
    digitalWrite(MS1_X, ms1_X);digitalWrite(MS2_X, ms2_X);digitalWrite(MS3_X, ms3_X);
  }
  else if (motor=='Y'){
    microSteps_Y=microstep;
    ms1_Y = MS1; ms2_Y = MS2; ms3_Y = MS3;
    digitalWrite(MS1_Y, ms1_Y);digitalWrite(MS2_Y, ms2_Y);digitalWrite(MS3_Y, ms3_Y);
  }
  else if(motor == 'T')
  {
    microSteps_Theta=microstep;
    ms1_Theta = MS1; ms2_Theta = MS2; ms3_Theta = MS3;
    digitalWrite(MS1_THETA, ms1_Theta);digitalWrite(MS2_THETA, ms2_Theta);digitalWrite(MS3_THETA, ms3_Theta);
    
  }
  else if (motor=='Z'){
    microSteps_Z=microstep;
    ms1_Z = MS1; ms2_Z = MS2; ms3_Z = MS3;
    digitalWrite(MS1_Z, ms1_Z);digitalWrite(MS2_Z, ms2_Z);digitalWrite(MS3_Z, ms3_Z);
  }
}

//-------------------------------------------------------------------------------
//              Function for the joystick's sensitivity slider (X & Z)
//-------------------------------------------------------------------------------


void setMicroStepsJoystick(int joystickSensitivity, char axis)
{
  //---------------------------------------------------------------------------------------------------------------
  // Set the desired microstepping configurations for the steppers based on position of the analog speed control.
  // Write these values to the DigitalPins
  // Update the microStep variables
  //---------------------------------------------------------------------------------------------------------------  
  if(axis == 'X')
  {
  
    if(joystickSensitivity < adc_depth/5.0)
    { 
     microSteps_X = 16;
     
    }
    else if (joystickSensitivity > adc_depth/5.0 && joystickSensitivity<=adc_depth*(2/5.0))
    { 
     microSteps_X = 8;
    
    }
    else if (joystickSensitivity > adc_depth*(2/5.0) && joystickSensitivity<=adc_depth*(3/5.0))
    {
     microSteps_X = 4;
   
    }  
    else if (joystickSensitivity > adc_depth*(3/5.0) && joystickSensitivity<=adc_depth*(4/5.0))
    {
     microSteps_X = 2;
     
    }  
    else if (joystickSensitivity > adc_depth*(4/5.0) && joystickSensitivity <=adc_depth)
    {
     microSteps_X = 1;
    
    }
    setMS123(microSteps_X,'X');
  }
    else if (axis == 'T')
    {
      if(joystickSensitivity < adc_depth/5.0)
      { 
       microSteps_Theta = 16;
      }
      else if (joystickSensitivity > adc_depth/5.0 && joystickSensitivity<=adc_depth*(2/5.0))
      { 

       microSteps_Theta = 8;
      }
      else if (joystickSensitivity > adc_depth*(2/5.0) && joystickSensitivity<=adc_depth*(3/5.0))
      {
       microSteps_Theta = 4;
      }  
      else if (joystickSensitivity > adc_depth*(3/5.0) && joystickSensitivity<=adc_depth*(4/5.0))
      {

       microSteps_Theta = 2;
      }  
      else if (joystickSensitivity > adc_depth*(4/5.0) && joystickSensitivity <=adc_depth)
      {
       microSteps_Theta = 1;
      }
      setMS123(microSteps_Theta,'T');
  
      
    }
  
 
   
   
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
void HandleMotorInterrupt_x()
{

	  x_EncoderBSet = digitalRead(x_EncoderPinB);
    x_EncoderASet = digitalRead(x_EncoderPinA);

   //Dir=ParseEncoder();
   	x_Dir = Decoder(x_EncoderAPrev, x_EncoderBPrev, x_EncoderASet, x_EncoderBSet);
  	x_EncoderTicks += -x_Dir;
  

  	x_EncoderAPrev = x_EncoderASet;
  	x_EncoderBPrev = x_EncoderBSet;

}
void HandleMotorInterrupt_y()
{
    y_EncoderBSet = digitalRead(y_EncoderPinB);
    y_EncoderASet = digitalRead(y_EncoderPinA);

   //Dir=ParseEncoder();
   	y_Dir = Decoder(y_EncoderAPrev, y_EncoderBPrev, y_EncoderASet, y_EncoderBSet);
  	y_EncoderTicks += y_Dir;
  

  	y_EncoderAPrev = y_EncoderASet;
  	y_EncoderBPrev = y_EncoderBSet;


}
void HandleMotorInterrupt_theta()
{

	theta_EncoderBSet = digitalRead(theta_EncoderPinB);
    theta_EncoderASet = digitalRead(theta_EncoderPinA);

   //Dir=ParseEncoder();
   	theta_Dir = Decoder(theta_EncoderAPrev, theta_EncoderBPrev, theta_EncoderASet, theta_EncoderBSet);
  	theta_EncoderTicks += -theta_Dir;
  

  	theta_EncoderAPrev = theta_EncoderASet;
  	theta_EncoderBPrev = theta_EncoderBSet;
}

// To be used when the Z linear stage is added
void HandleMotorInterrupt_z()
{

	z_EncoderBSet = digitalRead(z_EncoderPinB);
    z_EncoderASet = digitalRead(z_EncoderPinA);

   //Dir=ParseEncoder();
   	z_Dir = Decoder(z_EncoderAPrev, z_EncoderBPrev, z_EncoderASet, z_EncoderBSet);
  	z_EncoderTicks += z_Dir;
  

  	z_EncoderAPrev = z_EncoderASet;
  	z_EncoderBPrev = z_EncoderBSet;
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

  if(!isYtracking || ManualMode){
    // Step the Focus stage Stepper by a small number of steps
    if(DirInput > 0)
    {
      stepperY.move(-25*_yLimPos);
    }
    else if (DirInput < 0)
    {  
      stepperY.move(25*_yLimNeg);
    } 

    // 
//     if(DirInput > 0)
//    {
//      stepperY.move(50);
//    }
//    else if (DirInput < 0)
//    {  
//      stepperY.move(-50);
//    } 
    stepperY.setMaxSpeed(stepperSpeedY);
    stepperY.run();
  }
}

int Decoder(bool EncoderAPrev, bool EncoderBPrev, bool EncoderASet, bool EncoderBSet)
{
  if(!EncoderAPrev && EncoderASet)
  {
    if(EncoderBSet) return -1;
    else return 1;
  }
  else if(EncoderAPrev && !EncoderASet)
  {
    if(!EncoderBSet) return -1;
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

// Interrupt 

    
//-------------------------------------------------------------------------------
//                                 Timer camera trigger
//-------------------------------------------------------------------------------

void liquid_lens_handler_timer_500us(){
  
  // update phase (add *0.1 to debug )
  phase += 2*pi*(liquidLensSweepFrequency*timerPeriod/1000000);
  if (phase > 2*pi) {
    phase -= 2*pi;
  }
  // update phase_code
  phase_code =(int) round(65535*(phase)/(2*pi));

  //@@@ for debugging
  analogWrite(DAC0, phase_code/64);

  /*
  liquidLensV = liquidLensVOffset + liquidLensVAmplitude*sin(phase);
  liquidLensVCode = liquidLensV*22.5828 - 551.0199;
  
  // translate phase into voltage and send it to the liquid lens driver
  if (false){
    Wire.beginTransmission(0x77); // transmit to device (0xEE)
    // Wire.write(byte(0xEE));
    Wire.write(byte(0x03));   // UserMode reg address
    Wire.write(byte(0x03));   // UserMode: ACTIVE = 1, SM = 1
    Wire.write(byte(liquidLensVCode<<6));   // OIS_LSB reg value 
    Wire.endTransmission();     // stop transmitting
  }
  if (false){
    Wire.beginTransmission(0x77); // transmit to device (0xEE)
    Wire.write(byte(0x08));   // LLV4 reg address
    Wire.write(byte(liquidLensVCode>>2));   // LLV4
    Wire.write(byte(0x02));   // Command
    Wire.endTransmission();     // stop transmitting
  }
  */
  
  if(startTriggering)
  {
    counter_timer++;
    if (counter_timer==numTimerCycles) {
      counter_timer = 0;
      digitalWrite(triggerCamera,HIGH);
      // co-triggering FL camera for testing
      digitalWrite(triggerCamera_FL,HIGH);
      phase_code_lastTrigger = phase_code;
      //timestamp_lastTrigger = millis();
      timestamp_lastTrigger = timestamp_lastTrigger+1; // repurposed as trigger counter
      sendData = true;
    }
    if (counter_timer==1) {
      digitalWrite(triggerCamera,LOW);
      digitalWrite(triggerCamera_FL,LOW);
    }
  }
  if(startTriggering_FL)
  {
    counter_timer_FL++;
    if (counter_timer_FL == numTimerCycles_FL) {
      counter_timer_FL = 0;
      // Trigger FL camera
      digitalWrite(triggerCamera_FL,HIGH);
    }
    // We want this trigger signal to be longer duration.
    if (counter_timer_FL==5) 
    {
      digitalWrite(triggerCamera_FL,LOW);
    }

      
  }
}

//-------------------------------------------------------------------------------
//                                 Hardware interrupt: set phase to 0
//-------------------------------------------------------------------------------

void HandleOptotuneSYNCInterrupt() {
  phase = pi/2.0;
}
