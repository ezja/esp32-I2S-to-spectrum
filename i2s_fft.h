
// *************************************************************
// ESP32 & I2S mic --> FFT --> compression --> SPIFFS / LoRaWAN
// FFT via ESP-DSP lib, LoRaWAN compression via Wouter Brok
// *************************************************************

// bins in LoRaWAN payload
#define SELECTEDBINS ((ENDBIN-STARTBIN)/NR_OF_BINS_TO_COMBINE) 
// bins per byte in LoRaWAN payload
#define NR_OF_BINS_TO_COMBINE 5 
// 186 Hz @ BLOCK_SIZE 2048 & SAMPLERATE 6250 kHz
#define STARTBIN 32 
// 510 Hz @ BLOCK_SIZE 2048 & SAMPLERATE 6250 kHz
#define ENDBIN 82 

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2s.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_dsp.h"
#include "dsp_platform.h"
#include "dsps_view.h"
#include "dsps_fft2r.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"

// ***********************************
// s_spectrum is de LoRaWAN payload!
//
// momenteel 1 byte per freq. bin,
// amplitude gecomprimeerd naar uint8
// ***********************************
extern uint8_t s_spectrum[SELECTEDBINS];

void init_i2s();
void i2s_samplesread_ringbuf();
void i2s_samplesread();	 
void fft();
void startDeepSleep();
void heapdebug();
void spiffswrite();
void startTask_i2s_samplesread();
void startTask_fft();
void startTask_spiffswrite();
