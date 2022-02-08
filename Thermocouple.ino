// tmp0 and tmp1 are outputs only
void read_thermocouple(double &tmp0, double &tmp1)
{
  static struct var_max31856 TC_CH0, TC_CH1;   
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
  }
}
