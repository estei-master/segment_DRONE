// ************************************************************************************************************
// Mongoose Base AHRS firmware v1.2
// Mongoose 9DoF IMU + Barometric pressure sensor
// http://www.ckdevices.com
// March 20, 2013
//
// Mongoose IMU Firmware by Cory Duce is licensed under a Creative Commons Attribution 3.0 Unported License.
// Modifications and additions by Cory Duce (cory@ckdevices.com) to allow it to work with the Mongoose hardware
// Based on code by Doug Weibel and Jose Julio which was based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel
//
// This program is provided as is without any guarantee or warranty.
// Although the author has attempted to find and correct any bugs in the program, 
// the author is not responsible for any damage or losses of any kind caused by the 
// use or misuse of this program.
//
// ************************************************************************************************************


// Axis definition: 
   // X axis pointing forward (to the battery connector)
   // Y axis pointing to the right 
   // Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise


// The calibration values for the sensors need to be set in the function "mongooseCalibrate" which
// is in the file "ApplicationRoutines.pde"
// The Mongoose Visualizer PC software can be used to determine the offset values by looking at the Avg value 
// for each sensor axis.


/* Mongoose Hardware version - v1.0 and v1.0b
	
	ATMega328@3.3V w/ external 8MHz resonator
	High Fuse DA
        Low Fuse FF
	
	ADXL345: Accelerometer
	HMC5843: Magnetometer
	IGT-3200: Gyro
        BMP085: Barometric Pressure sensor
        
        Programmer : 3.3v FTDI
        Arduino IDE : Select board  "Arduino Pro or Pro mini (3.3V, 8MHz) w/ Atmega328"
*/

#include <HMC58X3.h>
#include <EEPROM.h>
#include <Wire.h>

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

#define FALSE 0
#define TRUE 1

#define OFF 0
#define ON 1

// ADXL345 Sensitivity(from datasheet) => 4mg/LSB   1G => 1000mg/4mg = 256 steps
// Tested value : 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 
#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square


// IGT-3200 Sensitivity (from datasheet) => 14.375 LSBs/ï¿½/s
// Tested values : 
#define Gyro_Gain_X   14.375 //X axis Gyro gain
#define Gyro_Gain_Y   14.375 //Y axis Gyro gain
#define Gyro_Gain_Z   14.375 //Z axis Gyro gain


int SENSOR_SIGN[9] = { 1,1,1,1,1,1,1,1,1};  //Correct directions x,y,z - gyros, accels, magnetormeter

#define Kp_ROLLPITCH 0.0125
#define Ki_ROLLPITCH 0.000008
#define Kp_YAW 1.2
#define Ki_YAW 0.000008



//========================================
// Output Data Configuration
// Turn on/off the different data outputs
// For the Mongoose Visualization Software to work, the first 3 need to be turned on
// To achieve the max sample rate, you will need to only turn on PRINT_EULER

#define PRINT_EULER             1   //Will print the Euler angles Roll, Pitch and Yaw
#define PRINT_SENSOR_DATA       0   //Will print the Corrected Sensor Data
#define PRINT_SENSOR_DATA_RAW   0   //Will print the raw uncorrected Sensor Data
#define PRINT_DCM               0   //Will print the whole direction cosine matrix
#define PRINT_MagCal            0

//#define PRINT_GPS 0     //Will print GPS data
//#define PRINT_BINARY 0  //Will print binary message and suppress ASCII messages (above)

#define debugPin 6
#define STATUS_LED 4  //PD4 on the Atmega328. Red LED

float G_Dt=0.005;    // Integration time (DCM algorithm)  We will run the integration loop at 200Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 
float AN[9]; //array that store the 3 ADC filtered data

float AN_OFFSET[9] = {0,0,0,0,0,0,0,0,0}; //Array that stores the Offset of the sensors

//Structure for holding offsets and calibration values for the accel, gyro, and magnetom
struct s_sensor_offsets
{
    
    float gyro_offset[3];
    float accel_offset[3];
    float magnetom_offset[3];
    float magnetom_XY_Theta;
    float magnetom_XY_Scale;
    float magnetom_YZ_Theta;
    float magnetom_YZ_Scale;
    
};


//structure for holding raw and calibration corrected data from the sensors
struct s_sensor_data
{
    //raw data is uncorrected and corresponds to the
    //true sensor axis, not the redefined platform orientation
    int gyro_x_raw;
    int gyro_y_raw;
    int gyro_z_raw;
    int accel_x_raw;
    int accel_y_raw;
    int accel_z_raw;
    int magnetom_x_raw;
    int magnetom_y_raw;
    int magnetom_z_raw;
  
    //This data has been corrected based on the calibration values
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float magnetom_x;
    float magnetom_y;
    float magnetom_z;
    float magnetom_heading;
    short baro_temp;  
    long baro_pres;
};

s_sensor_offsets sen_offset = {0,0,0,0,0,0,0,0,0};
s_sensor_data sen_data = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


float Accel_Vector[3]= {0,0,0};    //Store the acceleration in a vector
float Mag_Vector[3]= {0,0,0};      //Store the magnetometer direction in a vector
float Gyro_Vector[3]= {0,0,0};     //Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0};    //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};         //Omega Proportional correction
float Omega_I[3]= {0,0,0};         //Omega Integrator
float Omega[3]= {0,0,0};


// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

//These counters allow us to sample some of the sensors at lower rates
unsigned int  Compass_counter=0;
unsigned int  Baro_counter=0;
unsigned int  GPS_counter=0;
unsigned int  Print_counter=0;



float DCM_Matrix[3][3]       = {{1,0,0},{0,1,0},{0,0,1}}; 
float Update_Matrix[3][3]    = {{0,1,2},{3,4,5},{6,7,8}}; 
float Temporary_Matrix[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
 


void setup()
{ 
  Serial.begin(115200);    //115200 du port serie
 
  pinMode (STATUS_LED,OUTPUT);  // Status LED
  pinMode (debugPin,OUTPUT);  // debug LED

  Wire.begin();    //Init the I2C
  delay(20);
  
  //================================
  // Initialize all the IMU sensors
  //
  Init_Accel();
  Init_Compass();
  Init_Gyro();
  Init_Baro();
  
 

  //===============================
  // Get the calibration value for the sensors. These are hard coded right now
  mongooseCalibrate();
  
  
  // All the offset values and calibration factors are now setup for the sensors
  
  digitalWrite(STATUS_LED,HIGH);
    

  timer=millis();
  delay(20);
  
  //init some counters
  Compass_counter=0;
  Baro_counter=0;
  GPS_counter=0;
  Print_counter=0; 
}




void loop() //Main Loop
{
  if((DIYmillis()-timer)>= (G_Dt*1000) )  // Main loop runs at 200Hz
  {
        digitalWrite(debugPin,HIGH);
        
        
        timer_old = timer;
        timer=DIYmillis();
        G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
        if(G_Dt > 1)
            G_Dt = 0;  //keeps dt from blowing up, goes to zero to keep gyros from departing
        
   
        Compass_counter++;
        Baro_counter++;
        GPS_counter++;
        Print_counter++;
        
        
        //=================================================================================//
        //=======================  Data adquisition of all sensors ========================//
        
        
        //======================= Read the Gyro and Accelerometer =======================//
        Read_Gyro();      // Read the data from the I2C Gyro
        Read_Accel();     // Read I2C accelerometer
      
      
        //=============================== Read the Compass ===============================//
        if (Compass_counter > 20)  // Read compass data at 10Hz... (5 loop runs)
        {
          Compass_counter=0;
          
          Read_Compass();    // Read I2C magnetometer     
          
        }
        
        
        //===================== Read the Temp and Pressure from Baro =====================//
        if (Baro_counter > 200)  // Read baro data at 1Hz... (50 loop runs)
        {
          Baro_counter=0; 
          
          sen_data.baro_temp = Read_Temperature();
          sen_data.baro_pres = Read_Pressure();  
        }
        
        
        //=============================== Read the GPS data ==============================//
        if (GPS_counter > 50)  // Read GPS data at 1Hz... (50 loop runs)
        {
          GPS_counter=0;
         
          //this is were it would go...  
        }
        
        
        
        
        
        //=================================================================================//
        //=======================  Calculations for DCM Algorithm  ========================//
        Matrix_update(); 
        Normalize();
        Drift_correction();
        Euler_angles();
        
       
       
       
       
        //=================================================================================//
        //============================= Data Display/User Code ============================//
        // Make sure you don't take too long here!
     
        //=============================== Read the GPS data ==============================//
        if (Print_counter > 4)  // The gyro integreation and DCM algorithm is running at 200Hhz, but we are only outputting data at 1/4th that rate
        {
          Print_counter=0;
         
           printdata(); 
        }
        UpdateLED();
        
        digitalWrite(debugPin,LOW);
 
  }
   
}







   

