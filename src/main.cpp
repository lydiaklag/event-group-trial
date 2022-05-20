#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sdkconfig.h"
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

#define WIFI_NETWORK "secret passage_plus" // the name of the wifi network //at lab it is ESP_Test
#define WIFI_PASSWORD "Afra!17p89#"        // at lab it is esp8266_test
#define WIFI_TIMEOUT_MS 20000
#define WIFI_SSID "secret passage_plus"

// Your Firebase Project Web API Key
#define API_KEY "AIzaSyAIE_5ozQRoAcZaprySgTDVu_YB7QJycik"
// Your Firebase Realtime database URL
#define DATABASE_URL "https://accel-42dfb-default-rtdb.europe-west1.firebasedatabase.app/"


TaskHandle_t myIntTaskHandle = NULL;
EventGroupHandle_t demo_eventgroup;
const int TX1_BIT = BIT0;
const int TX2_BIT = BIT1;
const int GROUPSYNC0_BIT = BIT0;
const int GROUPSYNC1_BIT = BIT1;
const int GROUPSYNC2_BIT = BIT2;
const int GROUPSYNC3_BIT = BIT3;
const int ALL_SYNC_BITS = (BIT0 | BIT1 | BIT2 | BIT3);
// #define BIT_0 (1 << 0)
// #define BIT_4 (1 << 4)
//*** 1 bit/flag for each thread
// #define TASK_0_BIT        ( 1 << 0 )
// #define TASK_1_BIT        ( 1 << 1 )
// #define TASK_2_BIT        ( 1 << 2 )
// #define ALL_SYNC_BITS ( TASK_0_BIT | TASK_1_BIT | TASK_2_BIT )
static int taskCoreHR = 1;
static int taskCoreAccel = 0;
EventGroupHandle_t EventGroupHandle = NULL; // for 3 tasks
// SemaphoreHandle_t baton; //semaphore for serial monitor printing
EventGroupHandle_t xEventBits;
// this section is for HR, SPO2
void *aaa; // to call ConnectToWifi function
MAX30105 Sensor;
// Define signal parameters
int samp_freq = 25;          // for each led
const int Num_Samples = 100; // it stores 4 sec
uint32_t gr_buffer[Num_Samples];
int filtered_gr_buffer[Num_Samples];
int ma_gr_buffer[Num_Samples];
const int points_pr = 4;
float PR[points_pr];
float Pulse_Rate_next = 0, Pulse_Rate_previous = 70;
int HR;
//***Butterworth band-pass (0.5Hz-5Hz) 2nd order
const double b[] = {0.175087643672101, 0, -0.350175287344202, 0, 0.175087643672101};
const double a[] = {1, -2.299055356038497, 1.967497759984451, -0.874805556449481, 0.219653983913695};
IIRFilter f(b, a);
double filtered_gr = 0;
int Moving_Average_Num = 2;
int Num_Points = 2 * Moving_Average_Num + 1; //***5-point moving average filter
int Sum_Points;
void *ax;
int Sampling_Time = 2400; // 2400ms = 4s
int flag_unplugged = 0;   // 0 means unplugged
// regarding SpO2, initialising global variables
const int points_spo2 = 4;
double SpO2_dc_ir[points_spo2]; // no need to initialise the arrays, that is done inside the SpO2 func
double SpO2_ac_ir[points_spo2];
double SpO2_dc_red[points_spo2];
double SpO2_ac_red[points_spo2];
double Oxy[points_spo2];
int ma_ir2_buffer[Num_Samples];
int ma_red_buffer[Num_Samples];
double SpO2_next; // maybe double
double SpO2_previous = 99;
double SpO2;
// this section is for FireBase
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;

// unsigned long t_accel;
// int Sampling_Time = 2400;  //(sampl_time =40ms-->25 samples/sec--> fs=25Hz
int Sampling_Time_accel = 500; // now I made the accel freq f=2hz or T = 0.5s) //because the HR task takes 5,03s to get a result
// that means that for each second, I take 2 samples (2 Hz = 2 samples/s)
int flag_movement = 0; // that means that there is no movement
// when we detect movement, the variable movement will become 1, then we will turn off the red, IR LED (HR task)

ADXL362 xl;
int16_t XValue, YValue, ZValue, Temperature;
// void *a;
static int taskCore = 1;
int distance;
const int num_samples_accel = 9;
int arr_x[num_samples_accel];
int arr_y[num_samples_accel];
int arr_z[num_samples_accel];
double final_x, final_y, final_z, final_temp;
// int i = 0;
double sumx = 0, sumy = 0, sumz = 0;
int count_accel = 0, count_HR = 0, count_SpO2 = 0; // for firebase
double t_HR = 0, t_accel = 0, t_SpO2 = 0;      // for firebase

// declaring functions
void iir_ma_filter_for_hr();
int Find_Peak(int p1, int p2, int *x);
int Find_Mean(int p1, int p2, int *x);
int find_min_negative(int p1, int p2, int *x);
void convert_signal_to_positive(int p1, int p2, int *x, int point);
void ComputeHeartRate();
int readSamples();
void loopHR(void *pvParameter);
void ComputeSpO2(void *pvParameter);
void measurementsAccel(void *parameter);
void connectToWiFi(void *parameters);
void setup();
void loop();
// void fun0(void *parameters); //accel
// void fun1(void *parameters); //HR
void loopSpO2(void *parameters); // SpO2
void loopFirebase(void *parameters); // firebase


void myIntTask(void *p){
  while(1){
    vTaskSuspend(NULL); //this interrupt task suspends itself
    //here I turn off the red,IR led
    Serial.print("\t i turn off the red, ir LED.\n");
    //
  }
}


void setup()
{
  Serial.begin(9600);
  // xSemaphoreTake(baton, portMAX_DELAY);
  // Serial.println("setting everything up\n");
  // xSemaphoreGive(baton);
  //***for Max30105 HR, SpO2
  if (Sensor.begin() == false)
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1)
      ;
  }
  byte ledBrightness = 0xDF;                                                             // Options: 0=Off to 255=50mA  --> DF=~44mA
  byte sampleAverage = 2;                                                                // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3;                                                                      // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 200;                                                                  // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;                                                                  // Options: 69, 118, 215, 411
  int adcRange = 2048;                                                                   // Options: 2048, 4096, 8192, 16384
  Sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configure sensor with these settings
  Sensor.setPulseAmplitudeRed(0xFF);                                                     // if the value was 0, here we basically turn off the red LED, so green and IR LEDs are active
  // now that I changed the value to 0xFF it means that all the LEDs are on at the same time
  //***Firebase section
    connectToWiFi(&aaa);
    // config.api_key = API_KEY;
    // config.database_url = DATABASE_URL;
    // if (Firebase.signUp(&config, &auth, "", "")){
    //   Serial.println("Firebase connection ok");
    //   signupOK = true;
    // }
    // else{
    //   Serial.printf("%s\n", config.signer.signupError.message.c_str());
    // }
    // config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
    // Firebase.begin(&config, &auth);
    // Firebase.reconnectWiFi(true);

  ///***for accelerometer
  xl.begin(5);
  xl.beginMeasure();

  EventGroupHandle = xEventGroupCreate();
  if (EventGroupHandle != NULL)
  {
    xTaskCreatePinnedToCore(
        measurementsAccel,
        "accel measurements 362",
        10000,
        NULL,
        1,
        NULL,
        taskCoreAccel);
    xTaskCreatePinnedToCore(
        loopHR,
        "HR meas",
        10000,
        NULL,
        1,
        NULL,
        taskCoreHR);
    xTaskCreatePinnedToCore(
        loopSpO2,
        "SpO2 meas",
        10000,
        NULL,
        1,
        NULL,
        taskCoreHR);
    xTaskCreatePinnedToCore(
        loopFirebase,
        "firebase upload",
        10000,
        NULL,
        1,
        NULL,
        taskCoreAccel);
  }
}

void loop()
{
  vTaskDelete(NULL); //delete this task to save resources
  delay(10);
}

void connectToWiFi(void *parameters)
{
  for (;;)
  {
    Serial.print("Connecting to Wifi");
    WiFi.mode(WIFI_STA); // station mode, to connect to an existing wifi
    // AP mode to create a wifi network with esp32, to let someone else configure it by connecting to it
    WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);
    Serial.print("\tafter t has begun");
    unsigned long startAttemptTime = millis(); // uptime of esp32

    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS)
    {
      Serial.print(".");
      delay(100);
    }

    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("Failed!");
      // break;
    }
    else
    {
      Serial.print("Connected!");
      Serial.print(WiFi.localIP());
      break;
    }
  }
}

void measurementsAccel(void *parameters)
{ // accel
  // EventBits_t uxReturn;
  // TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
  uint32_t syncpos = 0;
  EventBits_t bits;
  for (;;)
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // double t_accel = millis();
    t_accel = millis();
    // xSemaphoreTake(baton, portMAX_DELAY);
    // Serial.println("accel has began");
    Serial.print("t_accel: ");
    Serial.print(t_accel);
    Serial.println();

    sumx = sumy = sumz = 0; // resetting the sums
    flag_movement = 0;      // resetting the flag
    for (int i = 0; i < num_samples_accel; i++)
    {
      xl.readXYZTData(XValue, YValue, ZValue, Temperature);
      arr_x[i] = XValue;
      arr_y[i] = YValue;
      arr_z[i] = ZValue;
      sumx += arr_x[i];
      sumy += arr_y[i];
      sumz += arr_z[i];
      // Serial.print("i: "); Serial.print(i); Serial.println();
      if ((i > 0) && (arr_x[i] > arr_x[i - 1] + 50 || arr_y[i] > arr_y[i - 1] + 50 || arr_z[i] > arr_z[i - 1] + 50))
      {                    // 50 is a threshold I found about movement detection, can be modified
        flag_movement = 1; // a bit like a flag
        // at this point I go to the HR_loop and I turn on the green LED
        break;
      }
      // vTaskDelay(Sampling_Time_accel / portTICK_PERIOD_MS); //it causes the thread to be 2.5s delayed
      delay(Sampling_Time_accel); // to make sure I take enough samples with the right freq
      // if I dont use delay I will take all the samples super quickly, in an unknown freq
      // what if I use the t_accel
      //  making a for loop that takes samples only at t_accel + 0.5s, using an if
      // ask Maria
    }
    // now I am out of the loop
    // Serial.println("temp: "); Serial.print(Temperature); Serial.println();
    final_x = sumx / num_samples_accel;
    final_y = sumy / num_samples_accel;
    final_z = sumz / num_samples_accel;
    // distance = sqrt(pow(final_x, 2) + pow(final_y, 2) + pow(final_z,2));
    if (flag_movement == 1)
    { // do something in the HR loop
      Serial.println("there was some movement...");
      Serial.print("\tx: ");
      Serial.print(final_x);
      Serial.print(" ");
      Serial.print("\ty: ");
      Serial.print(final_y);
      Serial.print(" ");
      Serial.print("\tz: ");
      Serial.print(final_z);
      Serial.print(" \n");
    }
    else
    {
      Serial.println("no movement");
      Serial.print("\tx: ");
      Serial.print(final_x);
      Serial.print(" ");
      Serial.print("\ty: ");
      Serial.print(final_y);
      Serial.print(" ");
      Serial.print("\tz: ");
      Serial.print(final_z);
      Serial.print(" \n");
    }
    // until here it is the functional code of the thread, now I checkin with other threads
    bits = xEventGroupSync(EventGroupHandle, GROUPSYNC0_BIT, ALL_SYNC_BITS, 60000 / portTICK_RATE_MS); // max wait 60s
    if (bits != ALL_SYNC_BITS)
    { // xWaitForAllBits == pdTRUE, so we wait for TX1_BIT and TX2_BIT so all other is timeout
      Serial.println("\tfail to receive synct eventgroup value");
    }
    else
    {
      // Serial.print("\tgroupsync_task1 get sync eventgroup from all tasks");
      // Serial.print(syncpos);
      Serial.println();
    }
    syncpos++;
    count_accel++;
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
    // i did all I wanted to do in this task, now I send the done_1_flag and wait for other tasks
    // Serial.print("txpos_accel: "); Serial.print(syncpos); Serial.println();
    // xEventGroupSetBits(EventGroupHandle, TX1_BIT);
    // // xEventGroupSync( xEventBits, TASK_2_BIT, ALL_SYNC_BITS, portMAX_DELAY );
    // vTaskDelay(10000/portTICK_PERIOD_MS);
    // txpos++;
    // vTaskDelay(Sampling_Time / portTICK_PERIOD_MS); //maybe
  }
}

void loopHR(void *parameters)
{ // HR
  uint32_t syncpos = 0;
  EventBits_t bits;
  for (;;)
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // double t_HR = millis();
    t_HR = millis();
    // xSemaphoreTake(baton, portMAX_DELAY);
    // Serial.println("HR has began");
    Serial.print("t_HR: ");
    Serial.print(t_HR);
    Serial.println();
    if (flag_movement)
    {
      Sensor.setPulseAmplitudeRed(0); //I turn off the IR and the red LEDs
      Sensor.setPulseAmplitudeIR(0);    
    }
    else
    {
      Sensor.setPulseAmplitudeRed(0xFF); //turn on red
      Sensor.setPulseAmplitudeIR(0xFF); //turn on ir //green stays on in all occasions
    }
    int flag = readSamples();
    Serial.println();
    Serial.println("----------------------------------------------------------------------------------------------------");
    if (flag)
    { //***if sensor reads real data
      iir_ma_filter_for_hr();
      int neg = find_min_negative(Moving_Average_Num, Num_Samples - Moving_Average_Num, ma_gr_buffer);
      convert_signal_to_positive(Moving_Average_Num, Num_Samples - Moving_Average_Num, ma_gr_buffer, neg);
      ComputeHeartRate();
      Serial.print("NEW DATA--> ");
      Serial.print("HR: ");
      Serial.print(HR);
      Serial.println();
      flag_unplugged = 1;
    }
    else
    { //***else if sensor is unplugged
      flag_unplugged = 0;
      HR = 0;
      Serial.print("HR : unplugged");
      Serial.println();
    }
    // xSemaphoreGive(baton);
    bits = xEventGroupSync(EventGroupHandle, GROUPSYNC1_BIT, ALL_SYNC_BITS, 60000 / portTICK_RATE_MS); // max wait 60s
    if (bits != ALL_SYNC_BITS)
    { // xWaitForAllBits == pdTRUE, so we wait for TX1_BIT and TX2_BIT so all other is timeout
      Serial.println("\tfail to receive synct eventgroup value");
    }
    else
    {
      // Serial.print("\tgroupsync_task1 get sync eventgroup from all tasks"); Serial.print(syncpos); Serial.println();
    }
    syncpos++;
    count_HR++;
    // could we need a tiny delay here?
    //  delay(10);
    // or better vTaskDelay
    // vTaskDelay(10 / portTICK_PERIOD_MS); // just 10ms delay. no need, no reason
  }
}
/* SpO2 */
void loopSpO2(void *parameters)
{
  uint32_t syncpos = 0;
  EventBits_t bits;
  for (;;)
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1000 milliseconds
    t_SpO2 = millis();
    // xSemaphoreTake(baton, portMAX_DELAY);
    // Serial.println("SpO2 has began");
    Serial.print("t_SpO2: ");
    Serial.print(t_SpO2);
    Serial.println();
    if (flag_movement != 1)
    { // calculate the SpO2 only if the finger is plugged, name of the flag is counterintuitive
      int mean_ir = Find_Mean(0, Num_Samples, ma_ir2_buffer);
      int mean_red = Find_Mean(0, Num_Samples, ma_red_buffer);

      //***detect successive peaks and mins
      for (int i = 0; i < points_spo2; i++)
      {
        SpO2_dc_ir[i] = 0;
        SpO2_ac_ir[i] = 0;
        SpO2_dc_red[i] = 0;
        SpO2_ac_red[i] = 0;
        Oxy[i] = 0;
      }

      int p_ir = 0;
      int p_red = 0;
      int peak_ir = 0;
      int peak_red = 0;
      int m_ir = 0;
      int m_red = 0;

      for (int j = 101; j < 350; j++)
      {
        //***Find peaks and means for IR
        if (ma_ir2_buffer[j] > ma_ir2_buffer[j - 1] && ma_ir2_buffer[j] > ma_ir2_buffer[j + 1] && ma_ir2_buffer[j] > mean_ir && peak_ir == 0)
        {

          /*Serial.print("peak  ir: ");
          Serial.print(ma_ir2_buffer[j]);
          Serial.println();*/
          SpO2_dc_ir[p_ir] = ma_ir2_buffer[j];
          p_ir++;
          p_ir %= points_spo2; // Wrap variable
          peak_ir = 1;
        }

        if (ma_ir2_buffer[j] < ma_ir2_buffer[j - 1] && ma_ir2_buffer[j] < ma_ir2_buffer[j + 1] && ma_ir2_buffer[j] < mean_ir && peak_ir == 1)
        {
          /*Serial.print("min  ir: ");
          Serial.print(ma_ir2_buffer[j]);
          Serial.println();*/
          SpO2_ac_ir[m_ir] = SpO2_dc_ir[p_ir - 1] - ma_ir2_buffer[j];
          m_ir++;
          m_ir %= points_spo2; // Wrap variable
          peak_ir = 0;
        }

        //***Find peaks and means for RED
        if (ma_red_buffer[j] > ma_red_buffer[j - 1] && ma_red_buffer[j] > ma_red_buffer[j + 1] && ma_red_buffer[j] > mean_red && peak_red == 0)
        {

          SpO2_dc_red[p_red] = ma_red_buffer[j];
          p_red++;
          p_red %= points_spo2; // Wrap variable
          peak_red = 1;
        }

        if (ma_red_buffer[j] < ma_red_buffer[j - 1] && ma_red_buffer[j] < ma_red_buffer[j + 1] && ma_red_buffer[j] < mean_red && peak_red == 1)
        {

          SpO2_ac_red[m_red] = SpO2_dc_red[p_red - 1] - ma_red_buffer[j];
          m_red++;
          m_red %= points_spo2; // Wrap variable
          peak_red = 0;
        }
      }

      for (int i = 1; i < points_spo2; i++)
      {

        if (SpO2_ac_ir[i] != 0 && SpO2_ac_red[i] != 0)
        {
          float R = (float(SpO2_ac_red[i]) / float(SpO2_dc_red[i])) / (float(SpO2_ac_ir[i]) / float(SpO2_dc_ir[i]));
          Oxy[i] = -45.060 * R * R + 30.354 * R + 94.845;
          // Serial.print("oxy: ");
          // Serial.print(Oxy[i]);
          // Serial.println();
        }
      }

      float sumSP = 0;
      int cSP = 0;

      for (int i = 1; i < points_spo2; i++)
      {

        if (Oxy[i] > 84 && Oxy[i] <= 100)
        {
          sumSP = sumSP + Oxy[i];
          cSP = cSP + 1;
        }
      }

      if (cSP > 0)
      {
        SpO2_next = sumSP / cSP;
      }

      // Serial.print("SpO2 predicted: ");
      // Serial.print(SpO2_next);
      // Serial.println();

      if (SpO2_next > 84 && SpO2_next <= 100)
      {
        if (SpO2_next - SpO2_previous < -4)
        {
          SpO2_next = SpO2_previous;
        }
        if (SpO2_next - SpO2_previous < -3)
        {
          SpO2_next = SpO2_previous - 1;
        }
        SpO2_previous = SpO2_next;
      }
      else
      {
        SpO2_next = SpO2_previous;
      }

      SpO2 = int(round(SpO2_next));
      if (flag_unplugged == 0 || HR == 0)
      { // if unplugged show this
        SpO2 = 0; //to show at firebase
        Serial.println();
        Serial.println("SpO2 : unplugged");
      }
      else
      {
        Serial.println();
        Serial.print("NEW DATA--> ");
        Serial.print("SpO2 : ");
        Serial.print(SpO2);
        Serial.print(" %");
        Serial.println();
      }
      // Serial.println(); Serial.print("time to run the SpO2 task: "); Serial.print(millis() - t_SP); Serial.println(); // I want to measure how long does it take for SpO2 task to run
      // vTaskDelay(Sampling_Time / portTICK_PERIOD_MS);
    }
    else
    {
      Serial.print("\n \tcan't measure SpO2 because there is movement.\n");
      SpO2=0;
    }
    bits = xEventGroupSync(EventGroupHandle, GROUPSYNC2_BIT, ALL_SYNC_BITS, 60000 / portTICK_RATE_MS); // max wait 60s
    if (bits != ALL_SYNC_BITS)
    { // xWaitForAllBits == pdTRUE, so we wait for TX1_BIT and TX2_BIT so all other is timeout
      Serial.println("\tfail to receive synct eventgroup value");
    }
    else
    {
      // Serial.print("\tgroupsync_task1 get sync eventgroup from all tasks"); Serial.print(syncpos); Serial.println();
    }
    syncpos++;
    count_SpO2++;
  }
}
/* Firebase */
void loopFirebase(void *parameters)
{
  uint32_t syncpos = 0;
  EventBits_t bits;
  for (;;)
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1 second
    double t_firebase = millis();
    // xSemaphoreTake(baton, portMAX_DELAY);
    // Serial.println("Firebase has began");
    Serial.print("t_firebase: ");
    Serial.print(t_firebase);
    Serial.println();
    // // doublecheck if the names of the variables are ok
    //start commenting from here to speed up the program
    // if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 4000 || sendDataPrevMillis == 0))
    // { // it sends a new value to firebase every 15 seconds
    //   sendDataPrevMillis = millis();
    //   // Write an Float number on the database path test/float
    //   if (Firebase.RTDB.setFloat(&fbdo, "accel/XValue accel", XValue))
    //   {
    //     Serial.print(" ");
    //     // if (Firebase.RTDB.setFloat(&fbdo, "test/float", 0.01 + random(0,100))){
    //     //  Serial.println("PASSED");
    //     //  Serial.println("PATH: " + fbdo.dataPath());
    //     //  Serial.println("TYPE: " + fbdo.dataType());
    //     // if the x value has been sent successfully to firebase, then update the count_accel
    //     if (Firebase.RTDB.setInt(&fbdo, "accel/count", count_accel))
    //     {
    //       Serial.print(" ");
    //       // Serial.println("PASSED");
    //       // Serial.println("PATH: " + fbdo.dataPath());
    //       // Serial.println("TYPE: " + fbdo.dataType());
    //     }
    //     else
    //     {
    //       Serial.print(" ");
    //       // Serial.println("FAILED");
    //       // Serial.println("REASON: " + fbdo.errorReason());
    //     }
    //     count_accel++;
    //   }
    //   else
    //   {
    //     Serial.print(" ");
    //     // Serial.println("FAILED");
    //     // Serial.println("REASON: " + fbdo.errorReason());
    //   }
    //   // now for Y data accel
    //   if (Firebase.RTDB.setFloat(&fbdo, "accel/YValue accel", YValue))
    //   {
    //     Serial.print(" ");
    //     // Serial.println("PASSED");
    //     // Serial.println("PATH: " + fbdo.dataPath());
    //     // Serial.println("TYPE: " + fbdo.dataType());
    //   }
    //   else
    //   {
    //     Serial.print(" ");
    //     // Serial.println("FAILED");
    //     // Serial.println("REASON: " + fbdo.errorReason());
    //   }
    //   // now for Z data accel
    //   if (Firebase.RTDB.setFloat(&fbdo, "accel/ZValue accel", ZValue))
    //   {
    //     Serial.print(" ");
    //     // Serial.println("PASSED");
    //     // Serial.println("PATH: " + fbdo.dataPath());
    //     // Serial.println("TYPE: " + fbdo.dataType());
    //   }
    //   else
    //   {
    //     Serial.print(" ");
    //     // Serial.println("FAILED");
    //     // Serial.println("REASON: " + fbdo.errorReason());
    //   }
    //   // now for time data accel
    //   if (Firebase.RTDB.setFloat(&fbdo, "accel/t_accel", t_accel))
    //   {
    //     Serial.print(" ");
    //     // Serial.println("PASSED");
    //     // Serial.println("PATH: " + fbdo.dataPath());
    //     // Serial.println("TYPE: " + fbdo.dataType());
    //   }
    //   else
    //   {
    //     Serial.print(" ");
    //     // Serial.println("FAILED");
    //     // Serial.println("REASON: " + fbdo.errorReason());
    //   }
    //   // now the same for HR measurements
    //   // now for HR data
    //   if (Firebase.RTDB.setFloat(&fbdo, "HR/HR", HR))
    //   {
    //     Serial.print(" ");
    //     // Serial.println("PASSED");
    //     // Serial.println("PATH: " + fbdo.dataPath());
    //     // Serial.println("TYPE: " + fbdo.dataType());
    //     // if it is all good with sending the HR measurements in firebase, then update the count_HR
    //     if (Firebase.RTDB.setInt(&fbdo, "HR/count", count_HR))
    //     {
    //       Serial.print(" ");
    //       // Serial.println("PASSED");
    //       // Serial.println("PATH: " + fbdo.dataPath());
    //       // Serial.println("TYPE: " + fbdo.dataType());
    //     }
    //     else
    //     {
    //       Serial.print(" ");
    //       // Serial.println("FAILED");
    //       // Serial.println("REASON: " + fbdo.errorReason());
    //     }
    //     count_HR++;
    //   }
    //   else
    //   {
    //     Serial.print(" ");
    //     // Serial.println("FAILED");
    //     // Serial.println("REASON: " + fbdo.errorReason());
    //   }
    //   // now for time data HR
    //   if (Firebase.RTDB.setFloat(&fbdo, "HR/t_HR", t_HR))
    //   {
    //     Serial.print(" ");
    //     // Serial.println("PASSED");
    //     // Serial.println("PATH: " + fbdo.dataPath());
    //     // Serial.println("TYPE: " + fbdo.dataType());
    //   }
    //   else
    //   {
    //     Serial.print(" ");
    //     // Serial.println("FAILED");
    //     // Serial.println("REASON: " + fbdo.errorReason());
    //   }
    //   if (Firebase.RTDB.setFloat(&fbdo, "SpO2/SpO2", SpO2))
    //   {
    //     Serial.print(" ");
    //     // Serial.println("PASSED");
    //     // Serial.println("PATH: " + fbdo.dataPath());
    //     // Serial.println("TYPE: " + fbdo.dataType());
    //     // if it is all good with sending the HR measurements in firebase, then update the count_HR
    //     if (Firebase.RTDB.setInt(&fbdo, "SpO2/count", count_SpO2))
    //     {
    //       Serial.print(" ");
    //       // Serial.println("PASSED");
    //       // Serial.println("PATH: " + fbdo.dataPath());
    //       // Serial.println("TYPE: " + fbdo.dataType());
    //     }
    //     else
    //     {
    //       Serial.print(" ");
    //       // Serial.println("FAILED");
    //       // Serial.println("REASON: " + fbdo.errorReason());
    //     }
    //     count_SpO2++;
    //   }
    //   else
    //   {
    //     Serial.print(" ");
    //     // Serial.println("FAILED");
    //     // Serial.println("REASON: " + fbdo.errorReason());
    //   }
    //   // now for time data HR
    //   if (Firebase.RTDB.setFloat(&fbdo, "SpO2/t_SpO2", t_SpO2))
    //   {
    //     Serial.print(" ");
    //     // Serial.println("PASSED");
    //     // Serial.println("PATH: " + fbdo.dataPath());
    //     // Serial.println("TYPE: " + fbdo.dataType());
    //   }
    //   else
    //   {
    //     Serial.print(" ");
    //     // Serial.println("FAILED");
    //     // Serial.println("REASON: " + fbdo.errorReason());
    //   }
    // }
//comment until here :) 

      bits = xEventGroupSync(EventGroupHandle, GROUPSYNC3_BIT, ALL_SYNC_BITS, 60000 / portTICK_RATE_MS); // max wait 60s
      if (bits != ALL_SYNC_BITS)
      { // xWaitForAllBits == pdTRUE, so we wait for TX1_BIT and TX2_BIT so all other is timeout
        Serial.println("\tfail to receive synct eventgroup value");
      }
      else
      {
        Serial.print(" ");
        // Serial.print("\tgroupsync_task1 get sync eventgroup from all tasks"); Serial.print(syncpos); Serial.println();
      }
      syncpos++;
    
  }
}
/*the next functions are for the HR computation */
void iir_ma_filter_for_hr()
{

  for (int i = 0; i < Num_Samples; i++)
  {

    filtered_gr = f.filter(double(gr_buffer[i]));
    filtered_gr_buffer[i] = round(filtered_gr);
  }

  for (int i = Moving_Average_Num; i < Num_Samples - Moving_Average_Num; i++)
  {
    Sum_Points = 0;
    for (int k = 0; k < Num_Points; k++)
    {
      Sum_Points = Sum_Points + filtered_gr_buffer[i - Moving_Average_Num + k];
    }
    ma_gr_buffer[i] = Sum_Points / Num_Points;
  }
}

int Find_Peak(int p1, int p2, int *x)
{
  int Peak_Magnitude = 0;
  for (int m = p1; m < p2; m++)
  {
    if (Peak_Magnitude < x[m])
    {
      Peak_Magnitude = x[m];
    }
  }
  return Peak_Magnitude;
}

int Find_Mean(int p1, int p2, int *x)
{
  int Mean = 0;
  int c = 0;
  for (int m = p1; m < p2; m++)
  {
    if (x[m] > 0)
    {
      Mean = Mean + x[m];
      c++;
    }
  }
  if (c > 0)
  {
    return Mean / c;
  }
  else
  {
    return 1;
  }
}

int find_min_negative(int p1, int p2, int *x)
{
  int min_magnitude = 0;
  for (int m = p1; m < p2; m++)
  {
    if (min_magnitude > x[m])
    {
      min_magnitude = x[m];
    }
  }
  return min_magnitude;
}

void convert_signal_to_positive(int p1, int p2, int *x, int point)
{
  for (int m = p1; m < p2; m++)
  {
    x[m] = x[m] - point;
  }
}

void ComputeHeartRate()
{

  int Mean_Magnitude = Find_Mean(Moving_Average_Num, Num_Samples - Moving_Average_Num, ma_gr_buffer);

  //***detect successive peaks and compute PR
  for (int i = 0; i < points_pr; i++)
  {
    PR[i] = 0;
  }
  int Peak = 0;
  int Index = 0;
  int p = 0;

  for (int j = Moving_Average_Num; j < Num_Samples - Moving_Average_Num; j++)
  {
    //***Find first peak

    if (ma_gr_buffer[j] > ma_gr_buffer[j - 1] && ma_gr_buffer[j] > ma_gr_buffer[j + 1] && ma_gr_buffer[j] > Mean_Magnitude && Peak == 0)
    {
      Peak = ma_gr_buffer[j];
      Index = j;
    }

    //***Search for next peak

    if (Peak > 0)
    {
      if (ma_gr_buffer[j] > ma_gr_buffer[j - 1] && ma_gr_buffer[j] > ma_gr_buffer[j + 1] && ma_gr_buffer[j] > Mean_Magnitude)
      {
        float d = j - Index;
        float pulse = (float)samp_freq * 60 / d; // bpm for each PEAK interval
        PR[p] = pulse;
        p++;
        p %= points_pr; // Wrap variable
        Peak = ma_gr_buffer[j];
        Index = j;
      }
    }
  }

  float sum = 0;
  int c = 0;
  for (int i = 0; i < points_pr; i++)
  {
    if (PR[i] != 0 && PR[i] != INFINITY)
    {
      sum = sum + PR[i];
      c = c + 1;
    }
  }

  if (c != 0)
  {
    Pulse_Rate_next = sum / c;
    if (Pulse_Rate_next > 40 && Pulse_Rate_next < 200)
    {
      if (Pulse_Rate_next - Pulse_Rate_previous >= 5)
      {
        Pulse_Rate_next = Pulse_Rate_previous + 1;
      }
      if (Pulse_Rate_next - Pulse_Rate_previous <= -5)
      {
        Pulse_Rate_next = Pulse_Rate_previous - 1;
      }
      Pulse_Rate_previous = Pulse_Rate_next;
    }
    else
    {
      Pulse_Rate_next = Pulse_Rate_previous;
    }
  }
  else
  {
    Pulse_Rate_next = Pulse_Rate_previous;
  }

  HR = int(round(Pulse_Rate_next));
}

int readSamples()
{
  int flag = 1;
  for (uint32_t i = 0; i < Num_Samples; i++)
  {
    // read max30105
    gr_buffer[i] = Sensor.getGreen();
    // Sensor.nextSample();
    if (gr_buffer[i] < 10000)
    {
      flag = 0;
    }
    delay(40);
  }
  return flag;
}

// end
