#include <Arduino.h>


// Connections to I2S microphone
#define I2S_WS 25
#define I2S_SD 32
#define I2S_SCK 33

esp_err_t init_i2s_mic(void);
uint32_t read_i2s_mic(void);
