#include "Ble.h"
#include "common.h"

extern volatile bool deviceConnected;
extern volatile bool code_recieved;
extern String bid;



BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;

/*when connected or disconnected to BLE this function is called internally*/
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = Ok;
        // credentails_recieved = false;
        Serial.println("BLE Connected");
        BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = Fail;
        // credentails_recieved = false;
        Serial.println("BLE Disconnect");
    }
};

/* whenever data recived form the device to hardware this fucnction is called internally */
class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string value = pCharacteristic->getValue();

        if (value.length() > 0)
        {
            Serial.print("Value : ");
            Serial.println(value.c_str());
            writeString(wifiAddr, value.c_str());

            // credentails_recieved = true;
            code_recieved = true;
            pCharacteristic->setValue("SUCCESS");
        }
    }


    /* writing the recieved data to eeprom */
    void writeString(int addr, String data)
    {
        int _size = data.length();
        for (int i = 0; i < _size; i++)
        {
            EEPROM.write(addr + i, data[i]);
        }
        EEPROM.write(addr + _size, '\0');
        EEPROM.commit();
    }
};

/******************************************
 * void EEPROM_writeString(int add, String data)
 * Writes the data to the EEPROM on location passed on the addr variable
 * Author : Sagar Kota
 * Last Modified : 22-03-01
 *******************************************/
void EEPROM_writeString(int addr, String data)
{
    int _size = data.length();
    for (int i = 0; i < _size; i++)
    {
        EEPROM.write(addr + i, data[i]);
    }
    EEPROM.write(addr + _size, '\0');
    EEPROM.commit();
}

/******************************************
 * void EEPROM_read_String(int add, String data)
 * Reads the Data from EEPROM form the loaction passed as arg addr till '\0' char
 * Author : Sagar Kota
 * Last Modified : 22-03-01
 *******************************************/
String EEPROM_read_String(int addr)
{
    char data[100];
    int len = 0;
    unsigned char k;
    k = EEPROM.read(addr);
    while (k != '\0' && len < 500)
    {
        k = EEPROM.read(addr + len);
        data[len] = k;
        len++;
    }
    data[len] = '\0';
    return String(data);
}

void erase_eeprom(void)
{
    EEPROM.begin(512); // Initialasing EEPROM
    Serial.println("Erasing...");
    Serial.println("clearing eeprom");
    for (int i = 0; i < 128; ++i)
    {
        EEPROM.write(i, 0);
    }
    EEPROM.commit();
}

/******************************************
 * String getValue(String data, char separator, int index)
 * Returns the String based the index passed
 * for e.g : String is sagar,kota , pass 0 for 'sagar' , and 1 for 'kota' and separator as ','
 * Author : Sagar Kota
 * Last Modified : 22-03-01
 *******************************************/
String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++)
    {
        if (data.charAt(i) == separator || i == maxIndex)
        {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

String Get_BLE_Name(void)
{
    bid.replace(":", "");
    return "Secure AQM-" + bid.substring(8, 12);

}

// String Get_Device_code(void)
// {
//     return _device_code;
// }

/******************************************
 * void WiFi_Setup_Using_BLE(void)
 * BLE server is opened up for wifi configuration
 * Author : Sagar Kota
 * Last Modified : 22-03-01
 *******************************************/
// void WiFi_Setup_Using_BLE(void)
void BLE_init(void)
{
    Serial.println("BLE MODE");
    String BLE_Name = Get_BLE_Name();
    Serial.print("BLE Server : ");
    Serial.println(BLE_Name);

    // Create the BLE Device
    BLEDevice::init(BLE_Name.c_str());

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_INDICATE);

    pCharacteristic->setCallbacks(new MyCallbacks());
    // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
    // Create a BLE Descriptor
    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setValue(bid.c_str()); // sending mac address at the start
    // Start the service
    pService->start();

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
    Serial.println("Waiting a client connection to notify...");
}

/******************************************
 * void BLE_Stop(void)
 * Stops the BLE services
 * Author : Sagar Kota
 * Last Modified : 22-03-01
 *******************************************/
void BLE_Stop(void)
{
    BLEDevice::stopAdvertising();
    // pService->stop();
    BLEDevice::deinit(true);
}
