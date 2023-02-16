#include <Arduino.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <Adafruit_ADS1X15.h>

// TaskHandle_t Task1;
TaskHandle_t Task2;
const int led1 = 33;
const int led2 = 25;

void Task1code(void * pvParameters ){

}

void Task2code(void * pvParameters ){
    
    Serial.print("Task2 running on core ");
    Serial.println(xPortGetCoreID());
    for(;;){
    vTaskSuspend(Task2); //suspend after each run
    Serial.println("Task2 ON");
    delay(700);
    Serial.println("Task2 OFF");
    delay(700);
    }
}

void setup(){
    Serial.begin(115200);
    Serial.println(xPortGetCoreID());
    pinMode(led1,OUTPUT);
    pinMode(led2,OUTPUT);

    ledcSetup(1, 5000, 8);
    ledcAttachPin(led1,1);
    ledcSetup(2, 5000, 8);
    ledcAttachPin(led2, 2);

    // xTaskCreatePinnedToCore(
    //     Task1code, /* Function to implement the task */
    //     "Task1", /* Name of the task */
    //     10000,  /* Stack size in words */
    //     NULL,  /* Task input parameter */
    //     0,  /* Priority of the task */
    //     &Task1,  /* Task handle. */
    //     0 /* Core where the task should run */
    // );

    xTaskCreatePinnedToCore(
        Task2code, /* Function to implement the task */
        "Task2", /* Name of the task */
        10000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        1,  /* Priority of the task */
        &Task2,  /* Task handle. */
        0 /* Core where the task should run */
    );
}

void loop(){
    Serial.print("Task1     running on core ");
    Serial.println(xPortGetCoreID());
    for(;;){
    vTaskResume(Task2);
    Serial.println("Task1 ON");
    delay(1000);
    Serial.println("Task1 OFF");
    delay(1000);
    }
}
