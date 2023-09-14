//PWM magnet wire with two k-type thermalcouples for use as a nociceptive probe

/***************************************************************************
This will heat magnet wire by PWM with two k-type thermalcouples for use as a nociceptive probe
M1 controls PWM  
M2 controls peltiers for assay plate heating
Thermocouple code from PlayingSEN30006_MAX31856_example.ino
***************************************************************************/
#include "PlayingWithFusion_MAX31856.h" //Playing With Fusion thermocouple breakout library 
#include "PlayingWithFusion_MAX31856_STRUCT.h"  //Playing With Fusion thermocouple breakout library 
#include "SPI.h"  //microcontroler library 
#include "PID_v1.h" //v1.2.0 by Brett Beauregard

// ##### Assay variables ###############################################
// #####################################################################
float Probe_targetTemp = 48.0; //target temperature of inside probe 
float Plate_targetTemp = 25.0; //temperature of assay plate
unsigned long assayTime = 4UL*3600000; // 4hrs in milliseconds 
// #####################################################################
// #####################################################################

//PID 1.2.0 by B.B.
//Define Variables we'll be connecting to
double Setpoint_probe, Input_probe, Output_probe, gap_probe;
double Setpoint_plate, Input_plate, Output_plate, gap_plate;

// ##### PID variables ###############################################
//double Kp_probe=24,Ki_probe=4,Kd_probe=5; //6/15
//double Kp_probe=36,Ki_probe=20,Kd_probe=5; //6/10 

double Kp_probe=18,Ki_probe=1,Kd_probe=0; //
//double Kp_plate=100,Ki_plate=10,Kd_plate=0; // old
double Kp_plate=15,Ki_plate=1,Kd_plate=0; // new 

//Specify the links and initial tuning parameters
//P_ON_M specifies that Proportional on Measurement be used, make the output move more smoothly when the setpoint is changed
//P_ON_E (Proportional on Error) is the default behavior
PID myPID_probe(&Input_probe, &Output_probe, &Setpoint_probe, Kp_probe, Ki_probe, Kd_probe,P_ON_M, DIRECT);
PID myPID_plate(&Input_plate, &Output_plate, &Setpoint_plate, Kp_plate, Ki_plate, Kd_plate,P_ON_M, DIRECT);


float probeOffset = 0.0; //changes target temp by 0.5oC
float plateOffset = 0.0; //changes target temp by 0.5oC
float caliProbe_targetTemp = Probe_targetTemp + probeOffset;
float caliPlate_targetTemp = Plate_targetTemp + plateOffset; 
float diffProbeTarget = 0.0;
float diffPlateTarget = 0.0;
int startTimeDelay = 2; //start time delay in minutes 

int PWMmax = 255; //constrain scaling to not burn magnet wire 
int maxCount = 0; //control fast PWM max 
int maxDelay = 0; //control fast PWM max 
float maxCountlength = 0.125; //secs max PWM is held for touch
float maxDelaylength = 20; //secs after touch the max can start again   

int holdingPWM = 0; // Store PWM to hold power for calibration 
bool calibration = false; 
bool enterPWMhold = false;

bool buttonPause = false; //debounce for offset buttons
unsigned long buttonTime = 0L; 

//Playing With Fusion breakout 

uint8_t TC0_CS  = 2;
uint8_t TC1_CS  = 3;

PWF_MAX31856  thermocouple0(TC0_CS);
PWF_MAX31856  thermocouple1(TC1_CS);

//Program variables 
unsigned long startTime; //store time at which the program is started after button click  
float loopCount = 0.0;
unsigned long runInterval = 125; // milliseconds interval of read-PID-print loop 
unsigned long loopMillis; //store last time of loop
int limitWireOutput; // contrained to 0-255 
int limitPeltierOutput;

//Button controler settings 
int sysOnLED = 8; // LED system start

int targetLED = A4; // target gap LED 
int errorLED = A5; // thermocouple error, others too later 

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

#define COOL 1       // motor current direction for cooling effect 
#define HEAT 0       // motor current direction for heating effect 
    
double tmp0; // thermocouple #1
double tmp1; // thermocouple #2
bool tmpStatus = false; //thermocouple error status 

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
  
  pinMode(targetLED, OUTPUT); // target gap LED 
  pinMode(errorLED, OUTPUT); 
  
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
  
  Serial.println("Probe_oC,Plate_oC,diff_Probe,diff_Plate,probeOffset,plateOffset,PWM_probe,PWM_plate,Video");  
}

void loop(){

// ##### Start Button trigger ##############################    
  while (trigger == false){  
    if (trigger == false && digitalRead(buttonStart) == LOW && digitalRead(buttonStop) == HIGH){
      digitalWrite(sysOnLED, HIGH);
      trigger = true;
    } else if (trigger == true && digitalRead(buttonStart) == HIGH && digitalRead(buttonStop) == LOW){
      digitalWrite(sysOnLED, LOW);
      trigger = false;
    }
    delay (1000);
    startTime = millis();  
    loopMillis = millis(); 
  }

// ##### Loop delay ####################################
  if(millis() - loopMillis >= runInterval){ 
    loopMillis = millis(); //store last time of loop
    
  // ##### Thermocouple code ##############################  
    //delay(125); //delay is now happening as while loop //... can be as fast as ~100ms in continuous mode, 1 samp avg
    read_thermocouple(tmp0, tmp1, tmpStatus);
  
  // ################ PWM control ########################################
    //PID 1.2.0
    Setpoint_probe = caliProbe_targetTemp;
    Input_probe = tmp0;
    gap_probe = abs(Setpoint_probe - Input_probe); //distance away from setpoint
  
    Setpoint_plate = caliPlate_targetTemp + 3; //approx degree difference from lights to plate 
    Input_plate = tmp1;
/*
    if(Setpoint_plate - Input_plate >= 0){
      tempDirection = HEAT;
    }else{
      tempDirection = COOL;
    }
*/   
    myPID_probe.SetTunings(Kp_probe, Ki_probe, Kd_probe);
    myPID_probe.Compute();

    myPID_plate.SetTunings(Kp_plate, Ki_plate, Kd_plate);
    myPID_plate.Compute();
  
    if(Output_plate > 50) { //constrain scaling for peltier because heating is delayed and because power supply limit for probe  
      limitPeltierOutput = 50; 
    } else if (Output_plate < 0){
      limitPeltierOutput = 0;
    } else {
      limitPeltierOutput = Output_plate;
    }

    // ######### Probe Range control ###############  
    if (gap_probe < 0.1 && (millis() - startTime) > (60000*startTimeDelay)){        
      digitalWrite(targetLED, HIGH); // Target light
    } else {
      digitalWrite(targetLED, LOW);
    } 

    if(Output_probe > PWMmax) { //constrain scaling 
      limitWireOutput = PWMmax; 
    } else if (Output_probe < 0){
      limitWireOutput = 0;
    } else {
      limitWireOutput = Output_probe; 
    }

    if(tmpStatus == true){ // thermocouple faults will shut PWM off of plate and probe
      limitPeltierOutput = 0;
      limitWireOutput = 0;
    }

    digitalWrite(URC10_MOTOR_1_DIR, 1); //Board motor controler current direction for magnet wire
    M1ArrayPower = (byte) limitWireOutput;  //set PWM byte from polynomial scaling 
    analogWrite(URC10_MOTOR_1_PWM, M1ArrayPower);   //send PWM value to magnet wire
      
    digitalWrite(URC10_MOTOR_2_DIR, tempDirection);
    M2ArrayPower = (byte) limitPeltierOutput;  //set PWM byte from polynomial scaling 
    analogWrite(URC10_MOTOR_2_PWM, M2ArrayPower);   //send PWM value to Peltier   

  // ################ Print variables ########################################
  
    diffProbeTarget = (caliProbe_targetTemp-tmp0);
    diffPlateTarget = (caliPlate_targetTemp-tmp1);
    
    Serial.print(tmp0); // print temperature sensor probe 
    Serial.print(",");
    Serial.print(tmp1); // print temperature sensor plate
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
    Serial.print(M2ArrayPower);
    Serial.print(","); 
    Serial.print(video_record); 

    Serial.println();
  }
// ##### End Loop delay ####################################

// ##### Video sync ####################################
  if (digitalRead(videoPin) == LOW){
    video_record = 0 ;
  } else if (digitalRead(videoPin) == HIGH){
    video_record = 1 ;
  }  

// ################ Assay controls ######################################## 

  if (digitalRead(offsetUp) == LOW && digitalRead(offsetDown) == HIGH && buttonPause == false){
    probeOffset = probeOffset + 0.5;
    caliProbe_targetTemp = Probe_targetTemp + probeOffset;
  } else if (digitalRead(offsetUp) == HIGH && digitalRead(offsetDown) == LOW && buttonPause == false){
    probeOffset = probeOffset - 0.5;
    caliProbe_targetTemp = Probe_targetTemp + probeOffset;
  }

  if (digitalRead(offsetUp) == LOW || digitalRead(offsetDown) == LOW && buttonPause == false){
    buttonTime = millis();
    buttonPause = true; 
  } else if (millis() - buttonTime > 100){
    buttonPause = false; 
  }

 // ################ Auto Shutoff ########################################
  while (millis() - startTime > assayTime){ //end program when assay length is done and hold in loop 
    analogWrite(URC10_MOTOR_1_PWM, 0); //turn wire off 
    analogWrite(URC10_MOTOR_2_PWM, 0); //turn peltier off
    digitalWrite(sysOnLED, LOW);
    Serial.println();
    Serial.print("DONE,");
    delay (2000);
    digitalWrite(sysOnLED, HIGH);
  }
}
