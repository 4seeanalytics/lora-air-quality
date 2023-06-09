#include "config.h"

#define ESP_getChipId() ((uint32_t)ESP.getEfuseMac())

String chipID = "Secure-" + String(ESP_getChipId(), HEX);



/******************************************
 * debug_string(string,bool)
 * Prints the message to Serial port and sends to MQTT server when device in debug mode
 * Author : Kaushlesh Chandel
 * Last Modified : Build 21-07-08
 *******************************************/
void debug_string(String msg, bool debug_msg/* = false*/)
{
    if (debug_msg == true)
    {
        Serial.print("##");
        Serial.println(msg);
    }
    else
    {
        Serial.print(">>");
        Serial.println(msg);
    }
 
}


void debug_string(String msg)
{
    Serial.print(">>");
    Serial.println(msg);
}
