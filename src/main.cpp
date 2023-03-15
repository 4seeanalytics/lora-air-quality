/*

  The onboard OLED display is SSD1306 driver and I2C interface. In order to make the
  OLED correctly operation, you should output a high-low-high(1-0-1) signal by soft-
  ware to OLED's reset pin, the low-level signal at least 5ms.

  OLED pins to ESP32 GPIOs via this connecthin:
  OLED_SDA -- GPIO4
  OLED_SCL -- GPIO15
  OLED_RST -- GPIO16


  The Writing diagram for ESP32

  I2C : Connected to GPIO04 and GPIO15
  Air Quality : Connected to GPIO18 & GPIO19

  Mic : Connected to GPIO 25, GPIO32, and GPIO33
  MIC_SCK --
  MIC_WS  --
  MIC_LR  --
  MIC_VDD --
  MIC_SD  --

*/

#include "heltec.h"
#include "images.h"

#include <WiFi.h>
 

#include <TaskScheduler.h>

#include "config.h"
 
#include "sensor.h"
#include "types.h"
 
#include "fx.h"

#include "common.h"

#include "lora_heltec.h"
 
/********************sagar created variables***************/

// on board led
extern int ledPin;
extern bool lora_data_sent;
int lora_counter = 0;

char data_lora[300];
int data_bytes = 0;

bool wifi_available = false;
bool wifi_Online = false;
bool mqtt_connected = false;

// extern data_config WM_config;
  
void task_scheduled_rebootCallback();

#define SCHEDULED_REBOOT_TIMER 1 * 25 * 60 * 60 * 1000 // Hours
#define SCHEDULED_CONNECTIVITY_TIMER 5 * 60 * 1000     // Minutes
#define SCHEDULED_HEARBEAT_TIMER 1 * 10 * 1000         // Seconds
 
Task task_scheduled_reboot(SCHEDULED_REBOOT_TIMER, TASK_FOREVER, &task_scheduled_rebootCallback);

#define BATTERY_POWERED true

Scheduler runner;

String device_mac = "";

extern pms5003data data;

ENV_Sensor Env_Sensor;

extern unsigned long loopsPM;     // Counter to track executions
extern unsigned long dataLoopsPM; // Loops with data

extern int previousDiagnosticsMillis;
extern int previousDataMillis;
extern int previousSystemMillis;

/**********************************************************/

/******************** LORA*********************/
#define BAND 868E6 // For Heltech OLED
// unsigned int counter = 0;
String rssi = "RSSI --";
String packSize = "--";
String packet;

void read_sensors_data()
{
  int readBytes = Env_Sensor.read();
  if (readBytes > 0)
  {
    String sensorData = Env_Sensor.sensor_data_buffer;

    // memcpy(data_lora, Env_Sensor.sensor_data_buffer, readBytes);
    memcpy(data_lora, Env_Sensor.compressed_data.c_str(), readBytes);
    data_bytes = Env_Sensor.compressed_bytes;

    debug_string("Sensor Read " + String(readBytes) + " bytes", true);
    
  }
  else
  {
    debug_string("Error reading sensor");
  }
}

/******************************************
 * setup_hardawre()
 * Setup the hardware, buttons, Device, Display, etc.
 * Author : Kaushlesh Chandel
 * Last Modified : Build 21/07/06
 *******************************************/
void setup_hardawre()
{
    Heltec.begin(false /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
    delay(1500);
 
}

/**************************************************************************/
/*
    Calculates the absolute Humidity based on the current temperature
*/
/**************************************************************************/
uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                // [mg/m^3]
  return absoluteHumidityScaled;
}

/**************************************************************************/
/*
    Draw the Logo bitmap. Need to change this to Secure
*/
/**************************************************************************/
void logo()
{
 
}

/************************************************/
/************************************************/
void setup()
{
  Serial.begin(115200); // Start the Serial Port
 
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);  

  debug_string("Setup hardawre");
  setup_hardawre();

 // serial_print_config(); // Print Config to Serial port

  init_lora();

  // Initalize the sensors
  check_sensors();

  delay(1000);
}

bool fresh_boot = true;

void loop()
{
  // Serial.println("*********** main loop ***********");

  loopsPM++;
   

  // lora loop.....
  lora_loop();

  unsigned long currentMillis = millis(); // Check current time
 
  // Send data based on data frequency. default is every 10 seconds
  if (currentMillis - previousDataMillis >= DEFAULT_SCANNING_FREQUENCY*1000)
  {
    dataLoopsPM++; // Count how many times data loop was executed
    previousDataMillis = currentMillis;
    read_sensors_data();
    lora_counter++;
    Serial.println("Lora Counter :" + String(lora_counter));

    if (lora_data_sent && lora_counter >= 18)
    {
      lora_counter = 0;
      lora_data_sent = false;
      send_data_over_lora(data_lora, data_bytes);
    }
    if (!BATTERY_POWERED)
    {
      display_data_oled();
    }
  }

   delay(5000);
}
