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
int Probe_targetTemp = 46; //target temperature of inside probe 
int Plate_targetTemp = 25; //temperature of calibration plate
float assayTime = 60*240; // 4hrs in seconds 
// #####################################################################
// #####################################################################

float probeOffset = 0; //changes target temp by 0.5oC
float plateOffset = 0; //changes target temp by 0.5oC
float caliProbe_targetTemp = Probe_targetTemp + probeOffset;
float caliPlate_targetTemp = Plate_targetTemp + plateOffset;
int PWMmax = 150; //constrain scaling to not burn magnet wire 
int holdingPWM = 0; // Store PWM to hold power for calibration 
bool calibration = false; 
bool enterPWMhold = false;
bool endHeat = false;

//Playing With Fusion breakout 

uint8_t TC0_CS  = 2;
uint8_t TC1_CS  = 3;

PWF_MAX31856  thermocouple0(TC0_CS);
PWF_MAX31856  thermocouple1(TC1_CS);

//Program variables 
float startTime; //store time at which the program is started after button click 
int limitWireOutput; // contrained to 0-255 
int limitPeltierOutput;

//PID 1.2.0 by B.B.
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

// Custom PID variables 
int tempPercent; //percent difference from sensor and target temp 
int absTempPercent; //make all temp values positive 
int rateAdjust; //adjusted rate of PWM power based

float proportion;
float cProportion = 370 / Probe_targetTemp * 1.6; //testing
float cIntegral = cProportion / 20.0;
float maxIntegral = 200; //testing
float integralActual = 0.0; // the "I" in PID ##### changing to try to prevent initial drop
float integralFunctional;

//Define the aggressive and conservative Tuning Parameters
int gapSet = 1;
double aggKp=12, aggKi=2, aggKd=1;
double aggKistart = 2;
double aggKiassay = 8;
//double consKp=2, consKi=0.05, consKd=0.25;
double consKp=12, consKi=2, consKd=1;

//Specify the links and initial tuning parameters
//P_ON_M specifies that Proportional on Measurement be used //P_ON_E (Proportional on Error) is the default behavior
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd,P_ON_M, DIRECT);

//Button controler settings 
int sysOnLED = 8; // LED system start

//int rangeGoLED = 18; // Range good to use
//int rangeStopLED = 19; // Outside range
int calibrationLED = A4; // Calibration loop active 
int PWMLED = A5; // hold PWM 

int buttonStart = A0;
int buttonStop = A1;
int offsetUp = A2;
int offsetDown = A3;

int videoPin = 9; //video input from rPi

byte video_record = 0;
bool trigger = false;

//Motor controler settings 
byte M1ArrayPower = 0; //Motor1 Array of Magnet wire 
byte M2ArrayPower = 0; //Motor1 Array of Peltier 

#define URC10_MOTOR_1_DIR 4 // set motor for direction control for Magnet wire
#define URC10_MOTOR_1_PWM 5 // set PWM for power control for Magnet wire

#define URC10_MOTOR_2_DIR 7 // set motor for direction control for Peltier
#define URC10_MOTOR_2_PWM 6 // set PWM for power control for Peltier 

#define COOL 0       // motor current direction for cooling effect 
#define HEAT 1       // motor current direction for heating effect 

int tempDirection = HEAT; 

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

  pinMode(sysOnLED, OUTPUT); // LED system start
  
  //pinMode(rangeGoLED, OUTPUT); // Range good to use
  //pinMode(rangeStopLED, OUTPUT); // Outside range
  pinMode(calibrationLED, OUTPUT); // Calibration loop active 
  pinMode(PWMLED, OUTPUT); // hold PWM 
  
  pinMode(buttonStart, INPUT_PULLUP);  
  pinMode(buttonStop, INPUT_PULLUP);
  pinMode(offsetUp, INPUT_PULLUP);
  pinMode(offsetDown, INPUT_PULLUP);
  
  pinMode(videoPin, INPUT); 

  //PID 1.2.0
  //initialize the variables we're linked to
  Setpoint = caliProbe_targetTemp;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);   

  Serial.print("#Target temp is ");
  Serial.println(Probe_targetTemp);
  
  Serial.println("Int-Temp,Ext-Temp,T%Probe,T%Plate,probeOffset,plateOffset,PWM,Video"); 
}

void loop(){

// ##### Start Button trigger ##############################    
  while (trigger == false){
    //digitalWrite(rangeGoLED, LOW);
    //digitalWrite(rangeStopLED, LOW);
    
    if (trigger == false && digitalRead(buttonStart) == LOW && digitalRead(buttonStop) == HIGH){
      digitalWrite(sysOnLED, HIGH);
      trigger = true;
    } else if (trigger == true && digitalRead(buttonStart) == HIGH && digitalRead(buttonStop) == LOW){
      digitalWrite(sysOnLED, LOW);
      trigger = false;
    }
 
    delay (1000);
    startTime = millis();  
  }

// ##### Video code ####################################
  if (digitalRead(videoPin) == LOW){
    video_record = 0 ;
  } else if (digitalRead(videoPin) == HIGH){
    video_record = 1 ;
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

// ################ Range LEDs ########################################
  if ((caliProbe_targetTemp-tmp0) > 0.5){
    //digitalWrite(rangeGoLED, LOW);
    //digitalWrite(rangeStopLED, HIGH);
  } else {
    //digitalWrite(rangeGoLED, HIGH);
    //digitalWrite(rangeStopLED, LOW);
  }  

// ################ Calibration button ########################################
  if (calibration == false && digitalRead(buttonStart) == LOW && digitalRead(buttonStop) == LOW){
    digitalWrite(calibrationLED, HIGH);
    calibration = true; 
  } else if (calibration == true && digitalRead(buttonStart) == LOW && digitalRead(buttonStop) == LOW) {
    digitalWrite(calibrationLED, LOW);
    calibration = false; 
  }  

// ################ PWM control ########################################

  //PID 1.2.0

  if (currentTime<180){ // 3 min
    aggKi=aggKistart;
  }else{
    aggKi=aggKiassay;
  }
  
  Setpoint = caliProbe_targetTemp;
  Input = tmp0;
  double gap = abs(Setpoint-Input); //distance away from setpoint
  
  if(gap<gapSet) { //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  } else { //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  
  myPID.Compute();

  if(Output > PWMmax) { //constrain scaling to not burn wire 
    limitWireOutput = PWMmax; 
  } else if (Output < 0){
    limitWireOutput = 0;
  } else {
    limitWireOutput = Output;
  }

  if (calibration == true) { 

    if (enterPWMhold == false && digitalRead(offsetUp) == LOW && digitalRead(offsetDown) == LOW){
      digitalWrite(PWMLED, HIGH);
      //holdingPWM = limitWireOutput; 
      enterPWMhold = true;
    } else if (enterPWMhold == true && digitalRead(offsetUp) == LOW && digitalRead(offsetDown) == LOW){
      digitalWrite(PWMLED, LOW);
      enterPWMhold = false;
    }      
    
    tempPercent = ((caliPlate_targetTemp - tmp1)/caliPlate_targetTemp) * 100; 
    proportion = caliPlate_targetTemp - tmp1; 
      
    integralActual += proportion;
    integralFunctional = integralActual; 
  
    if (integralActual > maxIntegral)
      integralFunctional = maxIntegral;
    else if (integralActual < -maxIntegral)
      integralFunctional = -maxIntegral;   

    rateAdjust = cProportion * proportion + cIntegral * integralFunctional;
  
    if(rateAdjust > 225) { //constrain scaling to 255 
      limitPeltierOutput = 225; 
    } else if (rateAdjust < 0){
      limitPeltierOutput = 0;
    } else {
      limitPeltierOutput = rateAdjust;
    }

    if (proportion > 0){
      tempDirection = HEAT; 
    } else if (proportion < 0){
      //tempDirection = COOL; 
    }
    
  } else { // Calibration == false 
    limitPeltierOutput = 0;
  }
  
  if (endHeat == true){
    limitWireOutput = 0;
    limitPeltierOutput = 0;
  }

  if (enterPWMhold == false){
    M1ArrayPower = (byte) limitWireOutput;  //set PWM byte from polynomial scaling 
  } else if (enterPWMhold == true){
    //M1ArrayPower = (byte) holdingPWM;
    M1ArrayPower = (byte) limitWireOutput;
  }

  digitalWrite(URC10_MOTOR_1_DIR, 1); //Board motor controler current direction for magnet wire
  analogWrite(URC10_MOTOR_1_PWM, M1ArrayPower);   //send PWM value to magnet wire
    
  digitalWrite(URC10_MOTOR_2_DIR, tempDirection);
  M2ArrayPower = (byte) limitPeltierOutput;  //set PWM byte from polynomial scaling 
  analogWrite(URC10_MOTOR_2_PWM, M2ArrayPower);   //send PWM value to Peltier   

// ################ Print variables control ########################################

  float diffProbeTarget = ((caliProbe_targetTemp-tmp0) / caliProbe_targetTemp) * 100;
  float diffPlateTarget = ((caliPlate_targetTemp-tmp1) / caliPlate_targetTemp) * 100;

  Serial.print(diffProbeTarget);
  Serial.print(","); 
  Serial.print(diffPlateTarget);
  Serial.print(","); 
  Serial.print(probeOffset,1);
  Serial.print(",");
  Serial.print(plateOffset,1);
  Serial.print(",");
  Serial.print(M1ArrayPower);
  Serial.print(",");
  Serial.print(video_record);

  //Trouble Shooting
  /*
  Serial.print(",");
  Serial.print(currentTime);
  Serial.print(",");
  Serial.print(",");
  Serial.print(M2ArrayPower);
  //Trouble Shooting 
  */
  Serial.println();

// ################ Assay controls ########################################

  if (digitalRead(buttonStart) == HIGH && digitalRead(buttonStop) == LOW){
    digitalWrite(sysOnLED, LOW);
    endHeat = true;
    Serial.println();
  }

  if (enterPWMhold == false){
    if (digitalRead(offsetUp) == LOW && digitalRead(offsetDown) == HIGH){
      probeOffset = probeOffset + 0.5;
    }
    if (digitalRead(offsetUp) == HIGH && digitalRead(offsetDown) == LOW){
      probeOffset = probeOffset - 0.5;
    }
  } else if (enterPWMhold == true){
    if (digitalRead(offsetUp) == LOW && digitalRead(offsetDown) == HIGH){
      plateOffset = plateOffset + 0.5;
    }
    if (digitalRead(offsetUp) == HIGH && digitalRead(offsetDown) == LOW){
      plateOffset = plateOffset - 0.5;
    }

    if (digitalRead(offsetUp) == LOW && digitalRead(offsetDown) == HIGH){
      holdingPWM = holdingPWM + 1;
    }
    if (digitalRead(offsetUp) == HIGH && digitalRead(offsetDown) == LOW){
      holdingPWM = holdingPWM - 1;
    }
  }

  caliProbe_targetTemp = Probe_targetTemp + probeOffset;
  caliPlate_targetTemp = Plate_targetTemp + plateOffset;

  while (currentTime > assayTime){ //end program when assay length is done and hold in loop 
    analogWrite(URC10_MOTOR_1_PWM, 0); //turn wire off 
    digitalWrite(sysOnLED, LOW);
    Serial.println();
    Serial.print("DONE,");
    delay (2000);
    digitalWrite(sysOnLED, HIGH);
  }
}
