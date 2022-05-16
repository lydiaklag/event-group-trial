#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sdkconfig.h" //not sure if this is needed
#include "esp_system.h"
// #include "esp_heap_alloc_caps.h
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
// #include "freertos/heap_regions.h"

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <SPI.h>
#include <ADXL362.h>
#include "max30105.h"
#include "IIRFilter.h"
#include <Time.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

EventGroupHandle_t demo_eventgroup;
const int TX1_BIT = BIT0;
const int TX2_BIT = BIT1;
const int GROUPSYNC0_BIT = BIT0;
const int GROUPSYNC1_BIT = BIT1;
const int GROUPSYNC2_BIT = BIT2;
const int ALL_SYNC_BITS = ( BIT0 | BIT1 | BIT2 );
// #define BIT_0 (1 << 0)
// #define BIT_4 (1 << 4)
//*** 1 bit/flag for each thread
#define TASK_0_BIT        ( 1 << 0 )
#define TASK_1_BIT        ( 1 << 1 )
#define TASK_2_BIT        ( 1 << 2 )

#define ALL_SYNC_BITS ( TASK_0_BIT | TASK_1_BIT | TASK_2_BIT )

static int taskCoreHR = 1;
static int taskCoreAccel = 0;
EventGroupHandle_t EventGroupHandle = NULL; // for 3 tasks
// SemaphoreHandle_t baton; //semaphore for serial monitor printing
EventGroupHandle_t xEventBits;

void setup();
void loop();
void fun0(void *parameters); //accel
void fun1(void *parameters); //HR
void fun2(void *parameters); //SpO2
// void fun3(void *parameters); //firebase

void setup() {
  Serial.begin(9600);
  // xSemaphoreTake(baton, portMAX_DELAY);
  Serial.println("setting everything up\n");
  // xSemaphoreGive(baton);
  demo_eventgroup = xEventGroupCreate();

  if (demo_eventgroup != NULL){
    xTaskCreatePinnedToCore(
      fun0,
      "accel measurements 362",
      10000,
      NULL,
      1,
      NULL,
      taskCoreAccel);
    xTaskCreatePinnedToCore(
      fun1,
      "HR meas",
      10000,
      NULL,
      1,
      NULL,
      taskCoreHR
    );
    xTaskCreatePinnedToCore(
      fun2,
      "SpO2 meas",
      10000,
      NULL,
      1,
      NULL,
      taskCoreHR
    );
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10);
}

void fun0(void *parameters){
  // EventBits_t uxReturn;
  // TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
  uint32_t syncpos=0;
  EventBits_t bits;
  for(;;){
    vTaskDelay(1000/portTICK_PERIOD_MS);
    double t_accel = millis();
    // xSemaphoreTake(baton, portMAX_DELAY);
    Serial.println("accel has began");
    Serial.print("t_accel: "); Serial.print(t_accel); Serial.println();
    bits=xEventGroupSync(demo_eventgroup, GROUPSYNC0_BIT, ALL_SYNC_BITS, 60000 / portTICK_RATE_MS); // max wait 60s
		if(bits!=ALL_SYNC_BITS) {  // xWaitForAllBits == pdTRUE, so we wait for TX1_BIT and TX2_BIT so all other is timeout
			Serial.println("\tfail to receive synct eventgroup value");
		} else {
			Serial.println("\tgroupsync_task1 get sync eventgroup from all tasks"); Serial.print(syncpos); Serial.println();
		}
		syncpos++;
    // xSemaphoreGive(baton); //in order to write well in serial monitor, as it is a common resource
    
    // xEventGroupWaitBits(
    //       EventGroupHandle,
    //       BIT_0 | BIT_4,
    //       pdTRUE,
    //       pdFALSE,
    //       portMAX_DELAY);
    /* Perform task functionality here. */
        

    /* Set bit 0 in the event group to note this task has reached the
    sync point.  The other two tasks will set the other two bits defined
    by ALL_SYNC_BITS.  All three tasks have reached the synchronisation
    point when all the ALL_SYNC_BITS are set.  Wait a maximum of 100ms
    for this to happen. */
    // uxReturn = xEventGroupSync( xEventBits,
    //                                 TASK_0_BIT,
    //                                 ALL_SYNC_BITS,
    //                                 xTicksToWait );
    // if( ( uxReturn & ALL_SYNC_BITS ) == ALL_SYNC_BITS )
    // {
    //   Serial.print("ok");
    //         /* All three tasks reached the synchronisation point before the call
    //         to xEventGroupSync() timed out. */
    // }
    //i did all I wanted to do in this task, now I send the done_1_flag and wait for other tasks 
    // Serial.print("txpos_accel: "); Serial.print(syncpos); Serial.println();
    // xEventGroupSetBits(EventGroupHandle, TX1_BIT);
    // // xEventGroupSync( xEventBits, TASK_2_BIT, ALL_SYNC_BITS, portMAX_DELAY );
    // vTaskDelay(10000/portTICK_PERIOD_MS);
    // txpos++;
  }
}

void fun1(void *parameters){
  uint32_t syncpos=0;
  EventBits_t bits;
  for(;;){
    vTaskDelay(1000/portTICK_PERIOD_MS);
    double t_HR = millis();
    // xSemaphoreTake(baton, portMAX_DELAY);
    Serial.println("HR has began");
    Serial.print("t_HR: "); Serial.print(t_HR); Serial.println();
    // xSemaphoreGive(baton);
    bits=xEventGroupSync(demo_eventgroup, GROUPSYNC1_BIT, ALL_SYNC_BITS, 60000 / portTICK_RATE_MS); // max wait 60s
		if(bits!=ALL_SYNC_BITS) {  // xWaitForAllBits == pdTRUE, so we wait for TX1_BIT and TX2_BIT so all other is timeout
			Serial.println("\tfail to receive synct eventgroup value");
		} else {
			Serial.println("\tgroupsync_task1 get sync eventgroup from all tasks"); Serial.print(syncpos); Serial.println();
		}
		syncpos++;

    /* Set bit 1 in the event group to note this task has reached the
    synchronisation point.  The other two tasks will set the other two
    bits defined by ALL_SYNC_BITS.  All three tasks have reached the
    synchronisation point when all the ALL_SYNC_BITS are set.  Wait
    indefinitely for this to happen. */
    //   Serial.print("txpos_accel: "); Serial.print(txpos); Serial.println();
    // xEventGroupSetBits(EventGroupHandle, TX1_BIT);
    // xEventGroupSync( xEventBits, TASK_1_BIT, ALL_SYNC_BITS, portMAX_DELAY );
    /* xEventGroupSync() was called with an indefinite block time, so
    this task will only reach here if the syncrhonisation was made by all
    three tasks, so there is no need to test the return value. */
    
  }
}

void fun2(void *parameters){
  uint32_t syncpos=0;
  EventBits_t bits;
  for(;;){
    vTaskDelay(1000/portTICK_PERIOD_MS); //5 seconds
    double t_SpO2 = millis();  
    // xSemaphoreTake(baton, portMAX_DELAY);
    Serial.println("SpO2 has began");
    Serial.print("t_SpO2: "); Serial.print(t_SpO2); Serial.println();
    bits=xEventGroupSync(demo_eventgroup, GROUPSYNC2_BIT, ALL_SYNC_BITS, 60000 / portTICK_RATE_MS); // max wait 60s
		if(bits!=ALL_SYNC_BITS) {  // xWaitForAllBits == pdTRUE, so we wait for TX1_BIT and TX2_BIT so all other is timeout
			Serial.println("\tfail to receive synct eventgroup value");
		} else {
			Serial.println("\tgroupsync_task1 get sync eventgroup from all tasks"); Serial.print(syncpos); Serial.println();
		}
		syncpos++;
    // xSemaphoreGive(baton);
    /* Set bit 2 in the event group to note this task has reached the
    synchronisation point.  The other two tasks will set the other two
    bits defined by ALL_SYNC_BITS.  All three tasks have reached the
    synchronisation point when all the ALL_SYNC_BITS are set.  Wait
    indefinitely for this to happen. */
    // xEventGroupSync( xEventBits, TASK_2_BIT, ALL_SYNC_BITS, portMAX_DELAY );

    
  }
}

//end 

