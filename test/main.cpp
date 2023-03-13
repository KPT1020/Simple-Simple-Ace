#include <Arduino.h>
#include <PID_v1.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <Adafruit_ADS1X15.h>
#include "SHTSensor.h"
#include <Adafruit_BMP280.h>

#define pumpPin_1 27
#define pumpPin_2 32
#define colPin_1  26
#define colPin_2  13
#define Channel_1 1
#define Channel_2 2
#define Channel_3 3
#define Channel_4 4
#define Channel_5 5

double Setpoint, Input, Output;
SHTSensor sht(SHTSensor::SHT4X);

Adafruit_ADS1115 ads;
int16_t raw=0;
int16_t adc0 =0;

// double Kp=4, Ki=0.08, Kd=0.015;
double consKp = 64, consKi=2, consKd=8;
// double Kp=20, Ki=1, Kd=0.5;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, REVERSE);

short buffer = 0 ;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while ( !Serial ) delay(100); 
  // Serial.println(F("BMP280 Sensor event test"));
  Wire.begin(21,22);

  // if (sht.init()) {
  //     Serial.print("init(): success\n");
  // } else {
  //     Serial.print("init(): failed\n");
  // }
  // sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x

  if (!ads.begin(0x48)) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }



  pinMode(pumpPin_1,OUTPUT);
  pinMode(pumpPin_2,OUTPUT);
  pinMode(colPin_1,OUTPUT);
  pinMode(colPin_2,OUTPUT);
  pinMode(34,INPUT);
  pinMode(25,OUTPUT);

  ledcSetup(Channel_1, 5000, 8);
  ledcSetup(Channel_2, 5000, 8);
  ledcSetup(Channel_3, 15000, 8);
  ledcSetup(Channel_4, 15000, 8);
  ledcSetup(Channel_5, 5000, 8);
  ledcAttachPin(pumpPin_1,Channel_1);
  ledcAttachPin(pumpPin_2,Channel_2);
  ledcAttachPin(colPin_1,Channel_3);
  ledcAttachPin(colPin_2,Channel_4);

  ledcWrite(Channel_1,255);
  ledcWrite(Channel_2,0);
  ledcWrite(Channel_3,110);
  ledcWrite(Channel_4,0);

  // dacWrite(25, 210);

  Setpoint = 5500;
  myPID.SetMode(AUTOMATIC);  
  myPID.SetTunings(consKp, consKi, consKd);
}

long cycle_time =0 ;
long previoustime = millis();
int i =0;
long pwm_write;
void loop() {
  Input = ads.readADC_SingleEnded(3);
  myPID.Compute();
  ledcWrite(Channel_3, Output);

  
  // for(int i =0; i<255;i++){
  //   Output =i;
  //   ledcWrite(Channel_1, (int)Output);
  //   raw = ads.readADC_SingleEnded(1);
  //   adc0 = raw - (int)(150.0*((128.0-Output)/128));
  //   Serial.print(raw);Serial.print(",");Serial.print(adc0);Serial.print(",");Serial.print(Output);Serial.print(",");Serial.println(ads.readADC_SingleEnded(3));
  // }

  // for(int i =255; i>=0 ; i--){
  //   Output =i;
  //   ledcWrite(Channel_1, (int)Output);
  //   Input = ads.readADC_SingleEnded(3);
  //   myPID.Compute();
  //   raw = ads.readADC_SingleEnded(1);
  //   adc0 = raw - (int)(150*((128.0-Output)/128));
  //   Serial.print(raw);Serial.print(",");Serial.print(ads.readADC_SingleEnded(1));Serial.print(",");Serial.print(Output);Serial.print(",");Serial.println(ads.readADC_SingleEnded(3));
  // }
    Serial.print(Output);
    Serial.print(",");
    Serial.print(ads.readADC_SingleEnded(0));
    Serial.print(",");
    Serial.print(ads.readADC_SingleEnded(1));
    Serial.print(",");
    Serial.print(ads.readADC_SingleEnded(2));
    Serial.print(",");
    Serial.println(ads.readADC_SingleEnded(3));
  delay(10);
  // Serial.print(ads.readADC_SingleEnded(0));Serial.print(",");Serial.print(ads.readADC_SingleEnded(1));Serial.print(",");Serial.print(Output);Serial.print(",");Serial.println(Input);
 

  // ledcWrite(Channel_1, 0);
  //   raw = ads.readADC_SingleEnded(1);
  // adc0 = raw - (int)(150*((128.0-Output)/128));
  // delay(10);
  // Serial.print(raw);Serial.print(",");Serial.print(adc0);Serial.print(",");Serial.print("0");Serial.print(",");Serial.println(Input);
}