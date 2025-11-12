//Prototype of a simple scheduler with 3 task frequencies (100hz - IMU + Kalman / 15hz - BAROMETER / 1hz - SDcard,GPS,etc.) 
#define ENDLINE_AFTER_IMU_LOG 0

#include <Arduino.h>  
#include <Wire.h>
//ForSDcard
#include <SPI.h>
#include <SD.h>
#define BUFFER_SIZE 100
#define Q_PARAM 10
#define R_PARAM 1
#define NO_CAL_SAMPLES 2000  //Number of samples taken per axis while calibrating the Gyro
//SD card SS pin
const int chipSelect = 11;

//Scheduler variables
unsigned long IMU_lasttime = 0;
unsigned long Baro_lasttime = 0;
unsigned long Misc_lasttime = 0;
//Intervals between executions [us]
const unsigned long IMU_dt = 10000;   //100 Hz -> 10 ms
const unsigned long Baro_dt = 66666;  // 15 Hz -> 66.666 ms
const unsigned long Misc_dt = 1000000;//  1 Hz -> 1000 ms

//Rotating array
int IMUindex = 0;//index that tells us to which row we are writing to
struct IMUReading {//Structure of 1 row of IMUTable array
  unsigned long timestamp; // Integer for micros() (4 bytes)
  float pitch;             // (4 bytes)
  float roll;              // (4 bytes)
};
//timestamp for last imu element saved to the SD
unsigned long last_IMU_SD_timestamp;
//Make a 100 element array called IMUTable of those records
volatile IMUReading IMUTable[100];


//Gyro variables
float RollRate, PitchRate, YawRate;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;

//Accelerometer variables
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;

//kalman filter variables
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;    //0 degrees - predicted initial angle
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;  //2 degrees - predicted initial uncertaininty

float Kalman_dt = 0.01;//This time is equal to the 1/frequency of the kalman filter and IMU

//initialize an array as an output of the Kalman filter
//{0,0} 0-angle prediction 0-uncertainty of the prediction
float Kalman1DOutput[] = { 0, 0 };

//Initialize funcion with Kalman equations
//that calculates predicted angle and uncertainty
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
//Initializes and calibrates the gyro
void gyro_init (void);
//Pulls data from IMU
void gyro_update (void);
unsigned long IMU_handler (void);//returns timestamp
void Baro_handler (void);
void MiscTasks();
File IMUlog;//file for logging pitch and roll

void setup() {

  SerialUSB.begin(57600);
  gyro_init();//declares 4 and 13 as output pins
  
  SerialUSB.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    SerialUSB.println("initialization failed. Things to check:");
    while (true);
  }

  SerialUSB.println("initialization done.");
  IMUlog = SD.open("imulog.csv", FILE_WRITE);
  IMUlog.println("timestamp[us];pitch[deg];roll[deg]");
  IMUlog.close();

}

void loop() {
  unsigned long now = micros();
  //Apogeedetectioncode
  //radioreccode

  if (now - IMU_lasttime >= IMU_dt) {
    IMU_lasttime = now;

    IMUTable[IMUindex].timestamp = IMU_handler ();
    IMUTable[IMUindex].pitch = KalmanAnglePitch;
    IMUTable[IMUindex].roll = KalmanAngleRoll;
    //SerialUSB.println(IMUindex);
    if(IMUindex<99)
    {
      IMUindex++;
    }
    else
    { SerialUSB.println();
      IMUindex = 0;
      for(int i = 0;i<100;i++)
      {
        SerialUSB.print(IMUTable[i].timestamp);
        SerialUSB.print(" - ");
        SerialUSB.println(IMUTable[i].pitch);
      } SerialUSB.println();
    }
  }//This gets called ~100 Hz

  if (now - Baro_lasttime >= Baro_dt) {
    Baro_lasttime = now;
    Baro_handler();
    //readBaro();
  }//This gets called ~15 Hz

  if (now - Misc_lasttime >= Misc_dt) {
    Misc_lasttime = now;
    MiscTasks();
  }//This gets called ~1Hz

}

void MiscTasks() {
  
  //Declaration of An Array that will be filled with data copied from IMUtable
  //We copy data and disable interrupts, to avoid writing data to SDcard while an array can be modified during the operation
  IMUReading SafeIMUTable[BUFFER_SIZE];

  //Atomic copy - fast copy of the data from volatile IMUtable
  int index_copy;
  //Block interrupts
  __disable_irq();
  //Copy the entire IMUtable buffer
  memcpy(SafeIMUTable, (const void*)IMUTable, sizeof(IMUReading) * BUFFER_SIZE);
  //Copy the index pointing at the next element that is going to be written
  index_copy = IMUindex;
  //Enable interrupts back
  __enable_irq();
  
  //Calculate start index
  int start_index = (index_copy) % BUFFER_SIZE;//redundant modulo operator is not needed here
  
  //Save data from our safe copy of the IMUtable - SafeImuTable
  IMUlog = SD.open("imulog.csv", FILE_WRITE);

  for(int i = 0; i < BUFFER_SIZE; i++)
  {
    int read_index = (start_index + i) % BUFFER_SIZE;

    //prevent zeros from array initialization from saving to SD
    if(!SafeIMUTable[read_index].timestamp>0)
    continue;
    //skip duplicate records
    if(last_IMU_SD_timestamp>=SafeIMUTable[read_index].timestamp)
    continue;
    
    // Write data to SD from SafeIMUTable
    IMUlog.print(SafeIMUTable[read_index].timestamp);
    IMUlog.print(";");
    IMUlog.print(SafeIMUTable[read_index].pitch);
    IMUlog.print(";");
    IMUlog.println(SafeIMUTable[read_index].roll);
  }
  //get the timestamp of the last written row
  last_IMU_SD_timestamp = SafeIMUTable[(start_index + BUFFER_SIZE-1) % BUFFER_SIZE].timestamp;

  #if ENDLINE_AFTER_IMU_LOG
  IMUlog.println(" ");
  #endif
  IMUlog.close();

}


unsigned long IMU_handler (void) {

  int time;
  gyro_update();

  //Get timestamp
  time = micros();//after that youcan calculate dynamic dt

  RollRate -= RateCalibrationRoll;
  PitchRate -= RateCalibrationPitch;
  YawRate -= RateCalibrationYaw;

  //1st Kalman filter for roll angle
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RollRate, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  //2nd Kalman filter for pitch angle
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, PitchRate, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  /*
  //Print Roll and Pitch in degrees
  SerialUSB.print("Roll = ");
  SerialUSB.print(KalmanAngleRoll);
  SerialUSB.print("Pitch = ");
  SerialUSB.println(KalmanAnglePitch);
  //*/
  return time;
}


void Baro_handler (void) {
}


void kalman_1d (float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + Kalman_dt * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + Kalman_dt * Kalman_dt * Q_PARAM * Q_PARAM;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + R_PARAM * R_PARAM);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  //Save Predicted State (angle) and uncertainty as an output
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
  //KalmanInput - rotation from Gyro
  //KalmanMeasurement - accelerometer angle
  //KalmanState - angle calculated by Kalman filter
}


void gyro_update (void) {

  //Start I2c gyro comms
  Wire.beginTransmission(0x68);

  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();  //Turn on low pass filter

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);        //Set the full scale range to +/-8G
  Wire.endTransmission();  //AFS CEL

  //Pull measurments from accelerometer
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);  //request data from accelerometer
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();  //Read data from accelerometer 3axis

  //Gyroscope output configuration
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);        //8 czy 08?
  Wire.endTransmission();  //Set sensitivity scale factor to 65.5 LSB/deg/s

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();  //access registers holding the measuremets

  //Pull rotation data from gyro
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();  //Read data from gyro 3 axis

  //Divide by LSB to convert to deg/sec
  RollRate = (float)GyroX / 65.5;
  PitchRate = (float)GyroY / 65.5;
  YawRate = (float)GyroZ / 65.5;

  //Convert acc data from LSB to physical values
  AccX = (float)AccXLSB / 4096 - 0.05;
  AccY = (float)AccYLSB / 4096 - 0.01;
  AccZ = (float)AccZLSB / 4096 - 0.12;  //with calibrated offsets

  //get angles and convert them from radians to degrees
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
}


void gyro_init (void) {

  pinMode(4,OUTPUT);//buzzer
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);  //Turn the diode on until calibration is complete

  Wire.setClock(400000);  //400khz clock
  Wire.begin();
  delay(250);  //give gyro time to start

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);  //Activate by writing to Power management register
  Wire.endTransmission();

  //Calibrate the Gyro
  for (int RateCalibrationNumber = 0; RateCalibrationNumber < NO_CAL_SAMPLES; RateCalibrationNumber++) {
    gyro_update();
    RateCalibrationRoll += RollRate;
    RateCalibrationPitch += PitchRate;
    RateCalibrationYaw += YawRate;
    delay(1);  //delay between calibration measurements
  }

  RateCalibrationRoll /= NO_CAL_SAMPLES;
  RateCalibrationPitch /= NO_CAL_SAMPLES;
  RateCalibrationYaw /= NO_CAL_SAMPLES;
  digitalWrite(13, LOW);  //calibration complete, turn off the diode
  LoopTimer = micros();
}