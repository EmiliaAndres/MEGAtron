
//Sketch based on https://github.com/CarbonAeronautics/Part-XIV-MeasureAngles

#include <Arduino.h>
#include <Wire.h>

#define NO_CAL_SAMPLES 2000 //Number of samples taken per axis while calibrating the Gyro
//Gyro variables
float RollRate,PitchRate,YawRate;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
//Accelerometer variables
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;


void gyro_signals(void){

  //Start I2c gyro comms
  Wire.beginTransmission(0x68);

  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission(); //Turn on low pass filter

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);      //Set the full scale range to +/-8G
  Wire.endTransmission();//AFS CEL

  //Pull measurments from accelerometer
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);//request data from accelerometer
  int16_t AccXLSB = Wire.read()<<8 | Wire.read();
  int16_t AccYLSB = Wire.read()<<8 | Wire.read();
  int16_t AccZLSB = Wire.read()<<8 | Wire.read();//Read data from accelerometer 3axis

  //Gyroscope output configuration
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();//Set sensitivity scale factor to 65.5 LSB/deg/s

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();//access registers holding the measuremets

  //Pull rotation data from gyro
  Wire.requestFrom(0x68,6);
  int16_t GyroX = Wire.read()<<8 | Wire.read();
  int16_t GyroY = Wire.read()<<8 | Wire.read();
  int16_t GyroZ = Wire.read()<<8 | Wire.read();//Read data from gyro 3 axis

  //Divide by LSB to convert to deg/sec
  RollRate = (float)GyroX/65.5;
  PitchRate = (float)GyroY/65.5;
  YawRate = (float)GyroZ/65.5; 

  //Convert acc data from LSB to physical values
  AccX=(float)AccXLSB/4096-0.05;
  AccY=(float)AccYLSB/4096-0.01;
  AccZ=(float)AccZLSB/4096-0.12;//with calibrated offsets(different for every accelerometer)

  //get angles and convert them from radians to degrees
  AngleRoll = atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch= -atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);

}

//void setup() {

  SerialUSB.begin(57600);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);//Turn the diode on until calibration is complete

  Wire.setClock(400000);//400khz clock
  Wire.begin();
  delay(250);//give gyro time to start

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00); //Activate MPU6050 by writing to Power management register
  Wire.endTransmission();

  //Calibrate the Gyro
  for(RateCalibrationNumber = 0; RateCalibrationNumber<NO_CAL_SAMPLES; RateCalibrationNumber++)
  {
    gyro_signals();
    RateCalibrationRoll += RollRate;
    RateCalibrationPitch += PitchRate;
    RateCalibrationYaw += YawRate;
    delay(1);//delay between calibration measurements
  }

  RateCalibrationRoll/=NO_CAL_SAMPLES;
  RateCalibrationPitch/=NO_CAL_SAMPLES;
  RateCalibrationYaw/=NO_CAL_SAMPLES;
  digitalWrite(13,LOW);//calibration complete, turn off the diode
}

void loop() {

  gyro_signals();

  /*
  //Print Gyro values in degrees
  RollRate-=RateCalibrationRoll;
  PitchRate-=RateCalibrationPitch;
  YawRate-= RateCalibrationYaw;
  SerialUSB.print("Rollrate = ");
  SerialUSB.print(RollRate);
  SerialUSB.print("Pitchrate = ");
  SerialUSB.print(PitchRate);
  SerialUSB.print("Yawrate = ");
  SerialUSB.println(YawRate);
  //*/

  /*
  //Print Accelerometer values in g's
  SerialUSB.print("Acc X = ");
  SerialUSB.print(AccX);
  SerialUSB.print("Acc Y = ");
  SerialUSB.print(AccY);
  SerialUSB.print("Acc Z = ");
  SerialUSB.println(AccZ);
  //*/

  //*
  //Print measured Pitch and Roll in degrees
  SerialUSB.print("Roll = ");
  SerialUSB.print(AngleRoll);
  SerialUSB.print("Pitch = ");
  SerialUSB.println(AnglePitch);
  //*/


  delay(50);
}
