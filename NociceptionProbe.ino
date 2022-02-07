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

float tmpOffset = 0; //changes target temp by 0.5oC
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
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

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
  Serial.begin(38400);                   // set baudrate of serial port
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
  
  static struct var_max31856 TC_CH0, TC_CH1;
  double tmp0; // thermocouple #1
  double tmp1; // thermocouple #2
   
  struct var_max31856 *tc_ptr;
  
  // Read CH 0
  tc_ptr = &TC_CH0;                             // set pointer
  thermocouple0.MAX31856_update(tc_ptr);        // Update MAX31856 channel 0
  // Read CH 1
  tc_ptr = &TC_CH1;                             // set pointer
  thermocouple1.MAX31856_update(tc_ptr);        // Update MAX31856 channel 1
  
  
// ##### Print thermo information to serial port ##############################

  // Thermocouple channel 0
  //Serial.print("Int-Tmp ");            // Print TC0 header
  if(TC_CH0.status)
  {
    // lots of faults possible at once, technically... handle all 8 of them
    // Faults detected can be masked, please refer to library file to enable faults you want represented
    Serial.println("fault(s) detected");
    Serial.print("Fault List: ");
    if(0x01 & TC_CH0.status){Serial.print("OPEN  ");}
    if(0x02 & TC_CH0.status){Serial.print("Overvolt/Undervolt  ");}
    if(0x04 & TC_CH0.status){Serial.print("TC Low  ");}
    if(0x08 & TC_CH0.status){Serial.print("TC High  ");}
    if(0x10 & TC_CH0.status){Serial.print("CJ Low  ");}
    if(0x20 & TC_CH0.status){Serial.print("CJ High  ");}
    if(0x40 & TC_CH0.status){Serial.print("TC Range  ");}
    if(0x80 & TC_CH0.status){Serial.print("CJ Range  ");}
    Serial.println(" ");
  }
  else  // no fault, print temperature data
  {
    //Serial.println("no faults detected");
    // MAX31856 Internal Temp
    tmp0 = (double)TC_CH0.ref_jcn_temp * 0.015625;  // convert fixed pt # to double
    //Serial.print("Tint = ");                      // print internal temp heading
    if((-100 > tmp0) || (150 < tmp0)){Serial.println("unknown fault");}
    else{Serial.print("");}
    
    // MAX31856 External (thermocouple) Temp
    tmp0 = (double)TC_CH0.lin_tc_temp * 0.0078125;           // convert fixed pt # to double
    Serial.print(tmp0); // print temperature sensor 0 
    Serial.print(",");
  }

  // Thermocouple channel 1
  //Serial.print("Ext-Tmp ");            // Print TC0 header
  if(TC_CH1.status)
  {
    // lots of faults possible at once, technically... handle all 8 of them
    // Faults detected can be masked, please refer to library file to enable faults you want represented
    Serial.println("fault(s) detected");
    Serial.print("Fault List: ");
    if(0x01 & TC_CH1.status){Serial.print("OPEN  ");}
    if(0x02 & TC_CH1.status){Serial.print("Overvolt/Undervolt  ");}
    if(0x04 & TC_CH1.status){Serial.print("TC Low  ");}
    if(0x08 & TC_CH1.status){Serial.print("TC High  ");}
    if(0x10 & TC_CH1.status){Serial.print("CJ Low  ");}
    if(0x20 & TC_CH1.status){Serial.print("CJ High  ");}
    if(0x40 & TC_CH1.status){Serial.print("TC Range  ");}
    if(0x80 & TC_CH1.status){Serial.print("CJ Range  ");}
    Serial.println(" ");
  }
  else  // no fault, print temperature data
  {
    //Serial.println("no faults detected");
    // MAX31856 Internal Temp
    tmp1 = (double)TC_CH1.ref_jcn_temp * 0.015625;  // convert fixed pt # to double
    
    // MAX31856 External (thermocouple) Temp
    tmp1 = (double)TC_CH1.lin_tc_temp * 0.0078125;           // convert fixed pt # to double
    Serial.print(tmp1); // print temperature sensor 1
    Serial.print(",");
  }

  float currentTime = (millis() - startTime)/1000.0;
  
// ################ PWM control ########################################

  //PID 1.2.0
  Setpoint = caliTargetTemp;
  Input = tmp0;
  double gap = abs(Setpoint-Input); //distance away from setpoint
  
  if(gap<10) {  //we're close to setpoint, use conservative tuning parameters
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
    Serial.print("END,");
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
