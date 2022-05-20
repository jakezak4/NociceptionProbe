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
int Probe_targetTemp = 38; //target temperature of inside probe 
int Plate_targetTemp = 25; //temperature of assay plate
float assayTime = 60*240; // 4hrs in seconds 
// #####################################################################
// #####################################################################

float probeOffset = 0; //changes target temp by 0.5oC
float plateOffset = 0; //changes target temp by 0.5oC
float caliProbe_targetTemp = Probe_targetTemp + probeOffset;
float caliPlate_targetTemp = Plate_targetTemp + plateOffset;
int PWMmax = 200; //constrain scaling to not burn magnet wire 
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
double Setpoint_probe, Input_probe, Output_probe, gap_probe;
double Setpoint_plate, Input_plate, Output_plate, gap_plate;

//Define the aggressive and conservative Tuning Parameters
//int gapSet_probe = 0; //conservative PID control is off 
double aggKp_probe=36, aggKi_probe=30, aggKd_probe=5;
double consKp_probe=36, consKi_probe=30, consKd_probe=5;
//double consKp_probe=24, consKi_probe=2, consKd_probe=1;

double Kp_probe=consKp_probe,Ki_probe=consKi_probe,Kd_probe=consKd_probe;
double Kp_plate=100,Ki_plate=20,Kd_plate=0;

//Specify the links and initial tuning parameters
//P_ON_M specifies that Proportional on Measurement be used, make the output move more smoothly when the setpoint is changed
//P_ON_E (Proportional on Error) is the default behavior
PID myPID_probe(&Input_probe, &Output_probe, &Setpoint_probe, Kp_probe, Ki_probe, Kd_probe,P_ON_M, DIRECT);
PID myPID_plate(&Input_plate, &Output_plate, &Setpoint_plate, Kp_plate, Ki_plate, Kd_plate,P_ON_M, DIRECT);

//Button controler settings 
int sysOnLED = 8; // LED system start

//int rangeGoLED = 18; // Range good to use
//int rangeStopLED = 19; // Outside range
int targetLED = A4; // target gap LED 
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
  pinMode(targetLED, OUTPUT); // target gap LED 
  pinMode(PWMLED, OUTPUT); // hold PWM 
  
  pinMode(buttonStart, INPUT_PULLUP);  
  pinMode(buttonStop, INPUT_PULLUP);
  pinMode(offsetUp, INPUT_PULLUP);
  pinMode(offsetDown, INPUT_PULLUP);
  
  pinMode(videoPin, INPUT); 

  //PID 1.2.0
  //initialize the variables we're linked to
  Setpoint_probe = caliProbe_targetTemp;
  Setpoint_plate = caliPlate_targetTemp;
  //turn the PID on
  myPID_probe.SetMode(AUTOMATIC);   
  myPID_plate.SetMode(AUTOMATIC);

  //Serial.print("#Target temp is ");
  //Serial.println(Probe_targetTemp);
  
  Serial.println("Int-Temp,Ext-Temp,diff-Probe,diff-Plate,probeOffset,plateOffset,PWM,Video"); 
}

void loop(){

  float currentTime = (millis() - startTime)/1000.0;

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

// ################ PWM control ########################################

  //PID 1.2.0
 
  Setpoint_probe = caliProbe_targetTemp;
  Input_probe = tmp0;
  gap_probe = abs(Setpoint_probe-Input_probe); //distance away from setpoint

  Setpoint_plate = caliPlate_targetTemp;
  Input_plate = tmp1;

  // ######### Target Light and probe K control ###############
  if (currentTime < (60*3)){ // 3 min
    digitalWrite(targetLED, LOW);
    Kp_probe = consKp_probe;
    Ki_probe = consKi_probe;
  } else if (gap_probe < 0.2 && currentTime > (60*3)){ // 3 min
    digitalWrite(targetLED, HIGH);
  } else {
    digitalWrite(targetLED, LOW);
    Kp_probe = aggKp_probe;
    Ki_probe = aggKi_probe;
  } 

  myPID_probe.SetTunings(Kp_probe, Ki_probe, Kd_probe);
  myPID_probe.Compute();
  
  if(Output_probe > PWMmax) { //constrain scaling to not burn wire 
    limitWireOutput = PWMmax; 
  } else if (Output_probe < 0){
    limitWireOutput = 0;
  } else {
    limitWireOutput = Output_probe;
  }

  myPID_plate.Compute();
  myPID_probe.SetTunings(Kp_plate, Ki_plate, Kd_plate);

  if(Output_plate > 254) { //constrain scaling to not burn wire 
    limitPeltierOutput = 254; 
  } else if (Output_plate < 0){
    limitPeltierOutput = 0;
  } else {
    limitPeltierOutput = Output_plate;
  }

  if (enterPWMhold == false && digitalRead(offsetUp) == LOW && digitalRead(offsetDown) == LOW){
    digitalWrite(PWMLED, HIGH);
    //holdingPWM = limitWireOutput; 
    enterPWMhold = true;
  } else if (enterPWMhold == true && digitalRead(offsetUp) == LOW && digitalRead(offsetDown) == LOW){
    digitalWrite(PWMLED, LOW);
    enterPWMhold = false;
  }      

  //set peltier direction for cooling or heating 
  //need a better way to compare to room temperature 
  if ((Plate_targetTemp - 20) > 0){
    tempDirection = HEAT; 
  } else if ((Plate_targetTemp - 20) < 0){
    tempDirection = COOL; 
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

  float diffProbeTarget = (caliProbe_targetTemp-tmp0)*10;
  float diffPlateTarget = (caliPlate_targetTemp-tmp1)*10;
   
  Serial.print(tmp0); // print temperature sensor 0 
  Serial.print(",");
  Serial.print(tmp1); // print temperature sensor 1
  Serial.print(",");
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
