
// *************************************************************
// ESP32 & I2S mic --> FFT --> compression --> SPIFFS / LoRaWAN
// FFT via ESP-DSP lib, LoRaWAN compression via Wouter Brok
// *************************************************************

// I2S sampling rate
#define SAMPLERATE 6250 

// FFT window size
#define BLOCK_SIZE 2048

// I2S ringbuffer size
#define I2S_BUFFER_SIZE 2048      
#define I2S_BUFFER_TYPE RINGBUF_TYPE_BYTEBUF

// FFTs to average per pass
#define AVG_FFT_NR 2 

// number of bins in LoRaWAN payload
#define SELECTEDBINS ((ENDBIN-STARTBIN)/NR_OF_BINS_TO_COMBINE) 

// bins per byte in LoRaWAN payload
#define NR_OF_BINS_TO_COMBINE 5 

// 98 Hz @ BLOCK_SIZE 2048 & SAMPLERATE 6250 kHz
#define STARTBIN 16 

// 586 Hz @ BLOCK_SIZE 2048 & SAMPLERATE 6250 kHz
#define ENDBIN 96 


// ***********************************
// s_spectrum is de LoRaWAN payload!
//
// momenteel 1 byte per freq. bin,
// amplitude gecomprimeerd naar uint8
// ***********************************

extern uint8_t s_spectrum[SELECTEDBINS];

extern int16_t fft_cnt;

// extern void init_i2s();

extern void i2s_samplesread_ringbuf();

extern void i2s_samplesread();	 

extern void fft();

extern void startDeepSleep();

extern void heapdebug();

extern void spiffswrite();

extern void startTask_i2s_samplesread();

extern void startTask_fft();

extern void startTask_spiffswrite();


