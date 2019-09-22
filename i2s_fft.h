
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
