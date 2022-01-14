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

// ##### Assay variables ###############################################
// #####################################################################
int targetTemp = 52; //final temp of the ramp
float tmpOffset = 1; //
int beginningHold = 0; //time before PWM begins UNIT = seconds 
float holdTarget = 600; //length of time that target temp held UNIT = seconds 
float assayTime = 800; //total assay time (beginningHold+ramp+holdTarget+extra) UNIT = seconds 
// #####################################################################
// #####################################################################

float caliTargetTemp = targetTemp * tmpOffset;
int PWMmax = 50; //constrain scaling to not burn wire 
bool assayMax = false; 
bool holdMax = false;
bool endHeat = false;
float startHold = 0; 

//Playing With Fusion breakout 

uint8_t TC0_CS  = 2;
uint8_t TC1_CS  = 3;

PWF_MAX31856  thermocouple0(TC0_CS);
PWF_MAX31856  thermocouple1(TC1_CS);

//Program variables 

int startDelay = 10000; //delay before PWM starts
float startTime; //store time at which the program is started after button click 

int tempPercent; //percent difference from sensor and target temp 
int absTempPercent; //make all temp values positive 
int rateAdjust; //adjusted rate of PWM power based
int constRateAdjust; // contrained to 0-255 

// PID variables 
float proportion;
float cProportion = 100 / caliTargetTemp * 1.4; //testing
//float cProportion = 25.788 * pow(caliTargetTemp,-0.549); //this works for 4 and 38 to 46oc
float cIntegral = cProportion / 10.0;
float maxIntegral = 150; //testing
//float maxIntegral = 0.1418 * pow(caliTargetTemp,2) - 5.6618 * caliTargetTemp + 100.38; //this works for 4 and 38 to 46oc
float integralActual = 0.0; // the "I" in PID ##### changing to try to prevent initial drop
float integralFunctional;

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

#define URC10_MOTOR_2_DIR 7 // set motor for direction control for Fan
#define URC10_MOTOR_2_PWM 6 // set PWM for power control for Fan

#define COOL 0       // motor current direction for cooling effect 
#define HEAT 1       // motor current direction for heating effect 

int tempDirection = HEAT;

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
/*  
// ##### Readout Labels ################################################
// #####################################################################
  Serial.print("Press ON to start. ");
  Serial.print("Target Temp is ");
  Serial.print(targetTemp);
  Serial.print("oC");
  Serial.print(";");
  Serial.println();  
// #####################################################################
// #####################################################################
*/
}

void loop(){

// ##### Button trigger ##############################    
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
  
  
// ##### Print information to serial port ##############################

  // Thermocouple channel 0
  Serial.print("Int-Tmp:");            // Print TC0 header
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
  Serial.print("Ext-Tmp:");            // Print TC0 header
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

  tempPercent = ((caliTargetTemp - tmp0)/caliTargetTemp) * 100; 
  proportion = caliTargetTemp - tmp0; 
  
  if (tempPercent <= 0) {
    assayMax = true; //sets when the ramp PWM should switch to hold PWM
  }

  if (currentTime < beginningHold){
    rateAdjust = 0;
  } else {
    
    integralActual += proportion;
    integralFunctional = integralActual; 
  
    if (integralActual > maxIntegral)
      integralFunctional = maxIntegral;
    else if (integralActual < -maxIntegral)
      integralFunctional = -maxIntegral;  
  }
  
  if (assayMax == false & currentTime > beginningHold) { 
    digitalWrite(URC10_MOTOR_1_DIR, tempDirection);
    rateAdjust = cProportion * proportion + cIntegral * integralFunctional;
  } else if (assayMax == true) { 
    if (holdMax == false) {
      startHold = currentTime;
      holdMax = true;
    }
    
    float holdTargetCount = currentTime - startHold;
    
    if (holdTargetCount <= holdTarget){
      digitalWrite(URC10_MOTOR_1_DIR, tempDirection);
      rateAdjust = cProportion * proportion + cIntegral * integralFunctional;
      } 
    else if (holdTargetCount > holdTarget){ //end thermal cycle 
      rateAdjust = 0;
      }    
  }

  if(rateAdjust > PWMmax) { //constrain scaling to not burn wire 
    constRateAdjust = PWMmax; 
  } else if (rateAdjust < 0){
    constRateAdjust = 0;
  } else {
    constRateAdjust = rateAdjust;
  }

  if (endHeat == true){
    constRateAdjust = 0;
  }

  M1ArrayPower = (byte) constRateAdjust;  //set PWM byte from polynomial scaling 

  /*
  //Trouble Shooting  
  Serial.print("integralActual;");
  Serial.print(integralActual);
  Serial.print("; ");
  Serial.print("integralFunctional;");
  Serial.print(integralFunctional);
  
  Serial.print("PWM;");
  Serial.print(M1ArrayPower);
  Serial.print("; ");
  //Serial.print(" Time;");
  //Serial.print(currentTime);
  //Serial.print(";");
  */

  float diffPercent = ((tmp0-tmp1) / tmp0) * 100;
  float diffTarget = ((targetTemp-tmp1) / targetTemp) * 100;
  
  //Serial.print("diff%:");
  //Serial.print(diffPercent);
  //Serial.print(",");
  Serial.print("target%:");
  Serial.print(diffTarget);
  Serial.print(",");

  float displayOffset = (tmpOffset-1) * 100;
  
  Serial.print("Offset:");
  Serial.print(displayOffset);
  
  Serial.println();
  
  analogWrite(URC10_MOTOR_1_PWM, M1ArrayPower);   //send PWM value to magnet wire 

  if (digitalRead(buttonStop) == LOW){
    digitalWrite(ledPin, LOW);
    endHeat = true;
    Serial.println();
    Serial.print("END");
  }

  if (digitalRead(buttonUp) == LOW){
    tmpOffset = tmpOffset + 0.05;
  }

  if (digitalRead(buttonDown) == LOW){
    tmpOffset = tmpOffset - 0.05;
  }

  caliTargetTemp = targetTemp * tmpOffset;
  
  if (currentTime > assayTime){
    Serial.println();
    Serial.print("DONE;");
  }
  
  while (currentTime > assayTime){ //end program when assay length is done and hold in loop 
    analogWrite(URC10_MOTOR_1_PWM, 0); //turn wire off 
    digitalWrite(ledPin, LOW);
    delay (1000);
  }
}
