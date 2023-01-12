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
#include <WiFiClientSecure.h>


#include <TaskScheduler.h>


#include "config.h"
#include "networking.h"
#include "sensor.h"
#include "types.h"
#include "mqtt.h"
#include "ota.h"
#include "fx.h"

#include "common.h"

/********************sagar created variables***************/
bool wifi_available = false;
bool wifi_Online = false;
bool mqtt_connected = false;

// extern data_config WM_config;


void task_send_mqtt_hearbeatCallback();
void task_check_connectivityCallback();
void task_scheduled_rebootCallback();


#define SCHEDULED_REBOOT_TIMER 1 * 25 * 60 * 60 * 1000 // Hours
#define SCHEDULED_CONNECTIVITY_TIMER 5 * 60 * 1000     // Minutes
#define SCHEDULED_HEARBEAT_TIMER 1 * 10 * 1000         // Seconds

Task task_send_mqtt_hearbeat(SCHEDULED_HEARBEAT_TIMER, TASK_FOREVER, &task_send_mqtt_hearbeatCallback);
Task task_check_connectivity(SCHEDULED_CONNECTIVITY_TIMER, TASK_FOREVER, &task_check_connectivityCallback);
Task task_scheduled_reboot(SCHEDULED_REBOOT_TIMER, TASK_FOREVER, &task_scheduled_rebootCallback);

Scheduler runner;



String device_mac = "";


extern pms5003data data;

/**********************************************************/

/******************** LORA*********************/
#define BAND 868E6 // For Heltech OLED
// unsigned int counter = 0;
String rssi = "RSSI --";
String packSize = "--";
String packet;

/******************************************
 * setup_hardawre()
 * Setup the hardware, buttons, Device, Display, etc.
 * Author : Kaushlesh Chandel
 * Last Modified : Build 21/07/06
 *******************************************/
void setup_hardawre()
{
  // WIFI Kit series V1 not support Vext control
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  Heltec.display->init();
  Heltec.display->setFont(ArialMT_Plain_10);
  delay(1500);
  Heltec.display->clear();

  device_mac = WiFi.macAddress();
  device_mac.replace(":", "");
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
  Heltec.display->clear();
  Heltec.display->drawXbm(0, 5, logo_width, logo_height, logo_bits);
  Heltec.display->display();
}

/********************************/
/********************************/
void setup()
{
  Serial.begin(115200); // Start the Serial Port

  debug_string("Reading config");
  check_configurations();

  debug_string("Setup hardawre");
  setup_hardawre();

  serial_print_config(); // Print Config to Serial port

  wifi_available = wifi_scan(WM_config.WiFi_Creds.wifi_ssid);

  if (wifi_available)
  {
    wifi_Online = wifi_connect();
  }

  if (wifi_Online)
  {
    // OTA....
    if (checkUpdateFirmware(SW_VERSION, HW_VERSION) == false)
    {
      debug_string("No OTA updates");
    }

    // mqtt....
    debug_string("Initialize MQTT ");
    if (init_mqtt() == true)
    {
      mqtt_connected = true;
      debug_string("MQTT Connected");
    }

    debug_string("IP Address : " + WiFi.localIP().toString());
    debug_string("Device mac : " + device_mac);
  }

  // Initalize the sensors
  check_sensors();

  // Runner has list of tasks that must be run at a set period
  runner.init();

  // Add task to Runner.
  runner.addTask(task_send_mqtt_hearbeat);
  runner.addTask(task_check_connectivity);
  runner.addTask(task_scheduled_reboot);

  // Enable the tasks. tasks start after 10 seconds
  task_send_mqtt_hearbeat.enableDelayed(10000);
  task_check_connectivity.enableDelayed(10000);
  task_scheduled_reboot.enableDelayed(10000);

  if (wifi_Online)
  {
    debug_string("All Good! Proceed to runway with WiFi");
  }
  else
  {
    debug_string("ERROR! Proceed to runway without WiFI");
  }

  delay(1000);
}


void loop()
{
  Serial.println("**********************");

  read_pm();
  read_voc();
  read_aht();
  read_lumen();
  // read_mic(); // Need to move to a seperate thread for Microphone DB Sensor
  display_data_oled();
  delay(1000);
  // counter++;
}