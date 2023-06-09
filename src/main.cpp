#include "sensor.h"
#include "config.h"
#include "heltec_lora.h"


#define BATTERY_POWERED false
#define DEFAULT_SCANNING_FREQUENCY 10

/******************** LORA*********************/
#define BAND 868E6 // For Heltech OLED
// unsigned int counter = 0;
// String rssi = "RSSI --";
// String packSize = "--";
// String packet;


int ledPin = 25;
extern bool lora_data_sent;
int lora_counter = 0;

char data_lora[300];
int data_bytes = 0;


int previousDataMillis;



ENV_Sensor Env_Sensor;


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
    Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
    Heltec.display->init();
    Heltec.display->setFont(ArialMT_Plain_10);
    delay(1500);
    Heltec.display->clear();
  
  // on board led...
  if (!BATTERY_POWERED)
  {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
  }

  // Turn-off the 'brownout detector'
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
 
}


/********************************/
/********************************/
void setup()
{

  // debug_string("Setup hardawre");
  setup_hardawre();

  // Initalize the sensors
  check_sensors();

  init_lora();
}




void loop()
{   
  // lora loop.....
  lora_loop();

  air_process();
  // read_pm();

  unsigned long currentMillis = millis(); // Check current time
 
  // Send data based on data frequency. default is every 10 seconds
  if (currentMillis - previousDataMillis >= DEFAULT_SCANNING_FREQUENCY*1000)
  {
    previousDataMillis = currentMillis;
    read_sensors_data();
    lora_counter++;
    Serial.println("Lora Counter :" + String(lora_counter));

    if (lora_data_sent && lora_counter >= 60)
    {
      lora_counter = 0;
      lora_data_sent = false;
      send_data_over_lora(data_lora, data_bytes);
    }


    // if ((!lora_data_sent) && lora_counter < 60)
    // {
    //   Serial.println("!!Lora send error!!");
    //   esp_restart();
    // }

    if (!BATTERY_POWERED)
    {
      display_data_oled();
    }
  }
}
