#include <Arduino.h>
#include <PID_v1.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <Adafruit_ADS1X15.h>
#include "SHTSensor.h"
#include <Adafruit_BMP280.h>

#define heaterPin_1 15
#define heaterPin_2 16
#define motor_EN 10
#define motor_ERR 11
#define NTCC 12
#define Channel_1 1
#define Channel_2 2
#define Channel_3 3
#define Channel_4 4
#define Channel_5 5

Adafruit_ADS1115 ads;
int16_t raw=0;
int16_t adc0 =0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while ( !Serial ) delay(100); 
  Wire.begin(21,22);

  if (!ads.begin(0x48)) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  pinMode(heaterPin_1,OUTPUT);
  pinMode(heaterPin_2,OUTPUT);
  pinMode(motor_EN,OUTPUT);
  pinMode(motor_ERR,INPUT);

  digitalWrite(motor_EN,255);
  ledcSetup(Channel_1, 5000, 8);
  ledcSetup(Channel_2, 5000, 8);
  ledcSetup(Channel_3, 15000, 8);
  ledcSetup(Channel_4, 15000, 8);
  ledcSetup(Channel_5, 5000, 8);
  ledcAttachPin(heaterPin_1,Channel_1);
  ledcAttachPin(heaterPin_2,Channel_2);
  // ledcAttachPin(25,Channel_5);

  ledcWrite(Channel_1,35);
  ledcWrite(Channel_2,0);

  dacWrite(25, 210);
}

void loop() {
    if((analogRead(motor_ERR)) == 0){
        Serial.println("ERROR, check circuit !");
        ledcWrite(Channel_1,0);
    }
    else{
        Serial.print("Temperature:");Serial.println(analogRead(NTCC));
    }
    delay(10);
}