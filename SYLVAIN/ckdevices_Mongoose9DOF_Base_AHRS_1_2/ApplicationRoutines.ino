



//=================================
// Fill in the calibration values
// this could be expanded to allow the values to determined 
// using a calibration mode for example
void mongooseCalibrate(void)
{
 
    // See the calibration guide for more details on what each 
    // of these are for. These values are unique for each Mongoose
    // Magnetometer calibration values can also be dependent on
    // the sensor platform
    
    sen_offset.accel_offset[0]     = 0;
    sen_offset.accel_offset[1]     = 0;
    sen_offset.accel_offset[2]     = 0;
    
    sen_offset.gyro_offset[0]      = 1;
    sen_offset.gyro_offset[1]      = 0;
    sen_offset.gyro_offset[2]      = -0.5;
    
    sen_offset.magnetom_offset[0]  = -7.5;
    sen_offset.magnetom_offset[1]  = -116;
    sen_offset.magnetom_offset[2]  = 15.5;
    
    sen_offset.magnetom_XY_Theta   = ToRad(0);
    sen_offset.magnetom_XY_Scale   = 1.0;
    
    sen_offset.magnetom_YZ_Theta   = ToRad(0);
    sen_offset.magnetom_YZ_Scale   = 1;
    
}


// This function just gives a short flash of the LED to show a heart beat
#define LED_ON_TIME 1  // On time of the LED in loop times
#define LED_OFF_TIME 50  // On time of the LED in loop times

void UpdateLED()
{
  static unsigned int counter_on = 0;
  static unsigned int counter_off = 0;
  static char state = 0;
  
  if(state == OFF) 
  {
    if(counter_off > LED_OFF_TIME)
    {
      counter_off = 0;
      counter_on = 0;
      
      state = ON;
      digitalWrite(STATUS_LED,HIGH);
    }
    else
    {
      counter_off++;
    }
  }
  else // state must == ON
   {
    if(counter_on > LED_ON_TIME)
    {
      counter_off = 0;
      counter_on = 0;
      
      state = OFF;
      digitalWrite(STATUS_LED,LOW);
    }
    else
    {
      counter_on++;
    }
  }
  
  
  
}










