//This will PWM magnet wire with two k-type thermalcouples for use as a nociceptive probe

/***************************************************************************
This will heat magnet wire by PWM with two k-type thermalcouples for use as a nociceptive probe
M1 controls PWM  
M2 unused
Thermocouple code from PlayingSEN30006_MAX31856_example.ino
***************************************************************************/
#include "PlayingWithFusion_MAX31856.h" //Playing With Fusion thermocouple breakout library 
#include "PlayingWithFusion_MAX31856_STRUCT.h"  //Playing With Fusion thermocouple breakout library 
#include "SPI.h"  //microcontroler library 
#include "PID_v1.h" //v1.2.0 by Brett Beauregard


// ##### Assay variables ###############################################
// #####################################################################
int targetTemp = 46; //target temperature of inside probe 
float assayTime = 60*120; // 2hrs in seconds 
// #####################################################################
// #####################################################################

float tmpOffset = 7.5; //changes target temp by 0.5oC
float caliTargetTemp = targetTemp + tmpOffset;
int PWMmax = 50; //constrain scaling to not burn magnet wire 
bool endHeat = false;

//Playing With Fusion breakout 

uint8_t TC0_CS  = 2;
uint8_t TC1_CS  = 3;

PWF_MAX31856  thermocouple0(TC0_CS);
PWF_MAX31856  thermocouple1(TC1_CS);

//Program variables 

int startDelay = 10000; //delay before PWM starts
float startTime; //store time at which the program is started after button click 

int limitOutput; // contrained to 0-255 

//PID 1.2.0 by B.B.
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
int gapSet = 1;
double aggKp=3, aggKi=0.5, aggKd=1;
double consKp=2, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
//P_ON_M specifies that Proportional on Measurement be used //P_ON_E (Proportional on Error) is the default behavior
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd,P_ON_M, DIRECT);

//Button controler settings 
int ledPin = 9; // button set up
int buttonStart = A0;
int buttonStop = A1;
int buttonUp = A2;
int buttonDown = A3;

byte leds = 0;
bool trigger = false;

//Motor controler settings 
byte M1ArrayPower = 0; //Motor1 Array of Peltiers 
char peltierPower[4];

#define URC10_MOTOR_1_DIR 4 // set motor for direction control for Peltier
#define URC10_MOTOR_1_PWM 5 // set PWM for power control for Peltier

void setup(){
  delay(250);                            // give chip a chance to stabilize
  Serial.begin(115200);                   // set baudrate of serial port
  //Serial.println("Playing With Fusion: MAX31856, SEN-30005;");

  // setup for the the SPI library:
  SPI.begin();                            // begin SPI
  SPI.setClockDivider(SPI_CLOCK_DIV16);   // SPI speed to SPI_CLOCK_DIV16 (1MHz)
  SPI.setDataMode(SPI_MODE3);             // MAX31856 is a MODE3 device
  
  // call config command... options can be seen in the PlayingWithFusion_MAX31856.h file
  thermocouple0.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_4SAMP, CMODE_AUTO);
  thermocouple1.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_4SAMP, CMODE_AUTO);

  pinMode(ledPin, OUTPUT); // button set up
  pinMode(buttonStart, INPUT_PULLUP);  
  pinMode(buttonStop, INPUT_PULLUP);
  pinMode(buttonUp, INPUT_PULLUP);
  pinMode(buttonDown, INPUT_PULLUP);

  //PID 1.2.0
  //initialize the variables we're linked to
  Setpoint = caliTargetTemp;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  digitalWrite(URC10_MOTOR_1_DIR, 1); //Board motor controler current direction  

  Serial.print("#Target temp is ");
  Serial.println(targetTemp);
  
  //Serial.println("Int-Temp,Ext-Temp,Target%,Offset"); 
  Serial.println("Int-Temp,Ext-Temp,Target%,Offset,PWM"); //Trouble shooting
}

void loop(){

// ##### Start Button trigger ##############################    
  while (trigger == false){
    if (digitalRead(buttonStart) == LOW){
      digitalWrite(ledPin, HIGH);
      trigger = true;
    } else if (digitalRead(buttonStop) == LOW){
      digitalWrite(ledPin, LOW);
      trigger = false;
    }
 
    delay (1000);
    startTime = millis();  
  }

// ##### Thermocouple code ##############################  
  delay(500);                                   // 500ms delay... can be as fast as ~100ms in continuous mode, 1 samp avg
  
  double tmp0; // thermocouple #1
  double tmp1; // thermocouple #2
   
  read_thermocouple(tmp0, tmp1);
  Serial.print(tmp0); // print temperature sensor 0 
  Serial.print(",");
  Serial.print(tmp1); // print temperature sensor 1
  Serial.print(",");
  
  float currentTime = (millis() - startTime)/1000.0;

// ################ PWM control ########################################

  //PID 1.2.0
  Setpoint = caliTargetTemp;
  Input = tmp0;
  double gap = abs(Setpoint-Input); //distance away from setpoint
  
  if(gap<gapSet) {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  } else {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  
  myPID.Compute();

  if(Output > PWMmax) { //constrain scaling to not burn wire 
    limitOutput = PWMmax; 
  } else if (Output < 0){
    limitOutput = 0;
  } else {
    limitOutput = Output;
  }

  if (endHeat == true){
    limitOutput = 0;
  }
  
  M1ArrayPower = (byte) limitOutput;  //set PWM byte from polynomial scaling 

  analogWrite(URC10_MOTOR_1_PWM, M1ArrayPower);   //send PWM value to magnet wire 

// ################ Print variables control ########################################

  float diffTarget = ((targetTemp-tmp1) / targetTemp) * 100;

  Serial.print(diffTarget);
  Serial.print(","); 
  Serial.print(tmpOffset,1);

  //Trouble Shooting  
  Serial.print(",");
  Serial.print(M1ArrayPower);
  /*
  Serial.print(",");
  Serial.print(currentTime);
  Serial.print(",");
  //Trouble Shooting 
  */

  Serial.println();

// ################ Assay controls ########################################

  if (digitalRead(buttonStop) == LOW){
    digitalWrite(ledPin, LOW);
    endHeat = true;
    Serial.println();
  }

  if (digitalRead(buttonUp) == LOW){
    tmpOffset = tmpOffset + 0.5;
  }

  if (digitalRead(buttonDown) == LOW){
    tmpOffset = tmpOffset - 0.5;
  }

  caliTargetTemp = targetTemp + tmpOffset;

  while (currentTime > assayTime){ //end program when assay length is done and hold in loop 
    analogWrite(URC10_MOTOR_1_PWM, 0); //turn wire off 
    digitalWrite(ledPin, LOW);
    Serial.println();
    Serial.print("DONE,");
    delay (2000);
    digitalWrite(ledPin, HIGH);
  }
}
