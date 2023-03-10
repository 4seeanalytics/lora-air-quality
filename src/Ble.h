#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"



#include "EEPROM.h"
#define EEPROM_SIZE 128

#define modeAddr 0
#define wifiAddr 0

void erase_eeprom(void);
String Get_BLE_Name(void);
// String Get_Device_code(void);
void BLE_init(void);
void BLE_Stop(void);

String EEPROM_read_String(int addr);
