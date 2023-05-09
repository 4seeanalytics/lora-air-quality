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

// #include <WiFi.h>

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
  

#define BATTERY_POWERED true

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


extern bool GOTO_DEEPSLEEP;

// Schedule TX every this many seconds
// Respect Fair Access Policy and Maximum Duty Cycle!
// https://www.thethingsnetwork.org/docs/lorawan/duty-cycle.html
// https://www.loratools.nl/#/airtime
const unsigned TX_INTERVAL = 60 * 3;

void read_sensors_data()
{
  Serial.println("read start");
  int readBytes = Env_Sensor.read();
  Serial.println("read end");
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
    // Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
    // Heltec.display->init();
    // Heltec.display->setFont(ArialMT_Plain_10);
    // delay(1500);
    // Heltec.display->clear();
  
  // on board led...
  if (!BATTERY_POWERED)
  {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
  }
 
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
  // Heltec.display->clear();
  // Heltec.display->drawXbm(0, 5, logo_width, logo_height, logo_bits);
  // Heltec.display->display();
}

/************************************************/
/************************************************/
void setup()
{
  Serial.begin(115200); // Start the Serial Port
 
  // debug_string("Setup hardawre");
  setup_hardawre();

 // serial_print_config(); // Print Config to Serial port

  // Initalize the sensors
  check_sensors();

  init_lora();
  // delay(1000);
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
    // if (!BATTERY_POWERED)
    // {
    //   display_data_oled();
    // }
  }

    
  // static unsigned long lastPrintTime = 0;

  // const bool timeCriticalJobs = os_queryTimeCriticalJobs(ms2osticksRound((TX_INTERVAL * 1000)));
  //   if (!timeCriticalJobs && GOTO_DEEPSLEEP == true && !(LMIC.opmode & OP_TXRXPEND))
  //   {
  //       Serial.print(F("Can go sleep "));
  //       LoraWANPrintLMICOpmode();
  //       SaveLMICToRTC(TX_INTERVAL);
  //       GoDeepSleep();
  //   }
  //   else if (lastPrintTime + 2000 < millis())
  //   {
  //       Serial.print(F("Cannot sleep "));
  //       Serial.print(F("TimeCriticalJobs: "));
  //       Serial.print(timeCriticalJobs);
  //       Serial.print(" ");

  //       LoraWANPrintLMICOpmode();
  //       PrintRuntime();
  //       lastPrintTime = millis();
  //   }

}
