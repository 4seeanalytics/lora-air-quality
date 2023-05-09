
#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>


void init_lora(void);
void lora_loop(void);

void ledFLash(int flashes);
void onEvent(ev_t ev);

void do_send(osjob_t *j , char * data, int sizebytes);
void send_data_over_lora(char * L_dta , int bytes);



void LoraWANPrintLMICOpmode(void);
void LoraWANDebug(lmic_t lmic_check);
void PrintRuntime();
void PrintLMICVersion();
void do_send(osjob_t *j);
void SaveLMICToRTC(int deepsleep_sec);
void LoadLMICFromRTC();
void GoDeepSleep();
