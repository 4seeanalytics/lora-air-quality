#ifndef FX_h
#define FX_h

#include "config.h"
 
#include "types.h"
#include "common.h"

bool shouldreboot = false;
 
extern bool mqtt_connected;

extern bool deviceHasWifiCreds;

extern data_config WM_config;

 

void task_scheduled_rebootCallback()
{
  if (shouldreboot == false) // Variable used to avoid rebooting for the first time callback is done
  {
      shouldreboot = true;
  }
  else
  {
    debug_string("Scheduled reboot");
    esp_restart();
  }
}

#endif