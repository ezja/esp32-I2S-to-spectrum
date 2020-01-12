#include <arduinoFFT.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "soc/i2s_reg.h"
#include "driver/i2s.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_sleep.h"

#define SAMPLERATE 6250 
#define BLOCK_SIZE 1024
// ~166ms per FFT block @ 6250KHz 

#define I2S_BUFFER_SIZE 2048      

// FFTs to average per pass
#define AVG_FFT_NR 5 

// number of bins in LoRaWAN payload
#define SELECTEDBINS ((ENDBIN-STARTBIN)/NR_OF_BINS_TO_COMBINE) 

// bins per byte in LoRaWAN payload
#define NR_OF_BINS_TO_COMBINE 5 

// 98 Hz @ BLOCK_SIZE 2048 & SAMPLERATE 6250 kHz
#define STARTBIN 16 

// 586 Hz @ BLOCK_SIZE 2048 & SAMPLERATE 6250 kHz
#define ENDBIN 96 

int32_t samples[BLOCK_SIZE];

double vReal[BLOCK_SIZE];
double vImag[BLOCK_SIZE];

static float spectrumsum_log[BLOCK_SIZE];
static float spectrumsum_lin[BLOCK_SIZE];
static float sortedspectrum[BLOCK_SIZE];

static float averagespectrum_log[BLOCK_SIZE];
static float averagespectrum_lin[BLOCK_SIZE];
static double cspctrm[SELECTEDBINS];

static uint16_t output_data_int_lin[BLOCK_SIZE];
static uint16_t averagespectrum_lin_int[BLOCK_SIZE];

uint8_t s_spectrum[SELECTEDBINS];

int16_t fft_cnt = 1;

// reference amplitude at -26db sensitivity (INMP441 datasheet) 
constexpr double MIC_REF_AMPL = pow(10, double(-26)/20) * ((1<<(24-1))-1);

arduinoFFT FFT = arduinoFFT(); /* c++ arduinoFFT FFT object */

static const char* TAG = "I2S TO FFT *** DEBUG OUTPUT:";


// ******************
// I2S config struct
// evt anders implementeren, zie https://forum.arduino.cc/index.php?topic=637667.0
// eerst memory area clearen?
// ******************

static void init_i2s()
{
         i2s_config_t i2s_config = {

              .mode = (i2s_mode_t)( I2S_MODE_MASTER | I2S_MODE_RX),

              .sample_rate = 6250,

              .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,

              // L/RSEL 2-ch: 
              // .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
              // L/RSEL 1-ch -- pin "SEL" @ SPH6045 en "L/R" @ INMP441 --> GND:
              .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,

              // ESP32 technical reference p.306: MSB/LSB first VS "Philips mode"
              .communication_format = I2S_COMM_FORMAT_I2S_MSB,

              .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,        
              
              .dma_buf_count = 32,                       
              .dma_buf_len = 64,  

              .use_apll = 1

    };

      // pin config 
      i2s_pin_config_t pin_config = {
              .bck_io_num = GPIO_NUM_21, 
              .ws_io_num = GPIO_NUM_22,
              .data_out_num = -1,
              .data_in_num = GPIO_NUM_23
    };

/*     gpio_set_direction(GPIO_NUM_21, GPIO_MODE_OUTPUT);*/
/*     gpio_set_direction(GPIO_NUM_22, GPIO_MODE_OUTPUT);*/
/*     gpio_set_direction(GPIO_NUM_23, GPIO_MODE_INPUT);*/

/*     gpio_set_pull_mode(GPIO_NUM_21, GPIO_PULLDOWN_ONLY);*/
/*     gpio_set_pull_mode(GPIO_NUM_22, GPIO_PULLDOWN_ONLY);*/
/*     gpio_set_pull_mode(GPIO_NUM_23, GPIO_PULLUP_ONLY);*/
  
     i2s_driver_install(I2S_NUM_1, &i2s_config, 0, NULL);
     // #ifdef SPH6045
     // REG_SET_BIT(  I2S_TIMING_REG(0),BIT(9));
     
     i2s_set_pin(I2S_NUM_1, &pin_config);
       
     i2s_stop(I2S_NUM_1); 
     i2s_start(I2S_NUM_1);
}


// *******************************
//
// FFT met arduinoFFT 1.5 library
//
// TODO: -- logaritmische spectra 
//       -- place buffers in IRAM
//       -- exp. moving averages
//
// *******************************

void fft(int times){ 

    int16_t N = BLOCK_SIZE; float sum = 0.; float ms = 0.; float rms = 0.; int32_t sum_samples; int32_t ms_samples; int32_t rms_samples;
    float avg_sum = 0.; float avg_ms = 0.; float avg_rms = 0.; int32_t avg_sum_samples; int32_t avg_ms_samples; int32_t avg_rms_samples;

    init_i2s(); // initialize I2S subsystem and read the I2S DMA buffer once to let the microphone boot up 
    
    Serial.print(F("I2S initialised"));
    
    //    while(1){     
      for (uint16_t j=0; j<times; j++) {
            // zero all buffers            
            memset(averagespectrum_lin,0,sizeof(averagespectrum_lin)); memset(averagespectrum_log,0,sizeof(averagespectrum_log));
            memset(spectrumsum_log,0,sizeof(spectrumsum_log)); memset(spectrumsum_lin,0,sizeof(spectrumsum_lin));
            memset(vReal,0,sizeof(vReal)); memset(vImag,0,sizeof(vImag));
          Serial.print(F("Recording..."));
      for (uint16_t i=0; i<AVG_FFT_NR; i++)
                  { Serial.print(i);  
                     int num_bytes_read = i2s_read_bytes(I2S_NUM_1, 
                                                      &samples, 
                                                      BLOCK_SIZE * sizeof(int32_t),     
                                                      portMAX_DELAY); 

                     for (int i = 0; i < BLOCK_SIZE; i++) 
                              {
                                vReal[i] = samples[i] >> 8; vImag[i] = 0.0; 
                              }

                     FFT.Windowing(vReal, BLOCK_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
                     FFT.Compute(vReal, vImag, BLOCK_SIZE, FFT_FORWARD);
                     FFT.ComplexToMagnitude(vReal, vImag, BLOCK_SIZE);

                     // cast output_data_float_lin --> int16 
                     memcpy(output_data_int_lin, vReal, N); 

                     for (int i=0; i < N / 2; i++)
                             {   
                spectrumsum_lin[i] += vReal[i]; 
                                spectrumsum_log[i] += vReal[i];
                             }

                     for (int i=0; i < N; i++)
                             {   
                                sum_samples +=  samples[i] << 8;
                             }

                     // RMS of (time-domain) audio signal
                     ms = pow((sum_samples / BLOCK_SIZE), 2); 
                     rms_samples = sqrt(ms_samples);  

                     for (int i=0; i < N / 2; i++)
                             {   
                                sum +=  vReal[i];
                             }
 
                     // RMS per spectrum of FFT data  
                     ms = powf((sum / (BLOCK_SIZE/2)), 2); 
                     rms = sqrtf(ms);
 
                   // ESP_LOGW(TAG,"----- SPECTUM #%d --> RMS = %f ----- \n \n", i, rms);
                     ms = 0; sum = 0; rms = 0; ms_samples = 0; sum_samples = 0; rms_samples = 0; 
                 }
         Serial.print(F("calculate moving average, lin spectrum"));
         // moving average, lin spectrum
                 for (int i=0; i < N / 2; i++)
                             { 
                                averagespectrum_lin[i] = (spectrumsum_lin[i] / AVG_FFT_NR); // * 100. -- extra headrooM?
                             }

        // moving average, log spectrum
                 for (int i=0; i < N / 2; i++)
                             {         
                                averagespectrum_log[i] = spectrumsum_log[i];
                             }

        // RMS van moving average
                 for (int i=0; i < N / 2; i++)
                     {         
                      avg_sum += averagespectrum_lin[i]; // ESP_LOGW(TAG,"averagespectrum_lin[%i]= %f \n", i, averagespectrum_lin[i]);       
                     }

                 // calculate RMS of averaged spectrums
                 avg_ms = powf((avg_sum / AVG_FFT_NR), 2); avg_ms_samples = pow((avg_sum_samples / AVG_FFT_NR), 2);
                 avg_rms = sqrtf(avg_ms); avg_rms_samples = sqrt(avg_ms / AVG_FFT_NR); 
                 ESP_LOGW(TAG,"\n -------- \n \n avg RMS = %f \n \n -------- \n \n", avg_rms);     


                 // calculate their SPL in dB  
                 const uint8_t FULL_SCALE_DBSPL = 120; 
                 const uint32_t FULL_SCALE_DBFS = 20*log10(pow(2,24)); 
                 
                 const float avg_spl = FULL_SCALE_DBSPL-(FULL_SCALE_DBFS-20*log10(sqrtf(2) * (avg_rms / MIC_REF_AMPL)));  
                 
                 const uint32_t avg_spl_samples = FULL_SCALE_DBSPL-(FULL_SCALE_DBFS-20*log10f(sqrt(2) * (avg_rms_samples / MIC_REF_AMPL)));  
                 
                 
               ESP_LOGW(TAG,"avg SPL FFT = %f \n -------- \n avg SPL samples = %i \n -------- \n ", avg_spl, avg_spl_samples); 

               // calculate which three bins have highest amplitude using bubble sort
                 
               memcpy(sortedspectrum, averagespectrum_lin, sizeof(averagespectrum_lin));        
               int fmax1bin, fmax2bin, fmax3bin; float fmax1, fmax2, fmax3; float maxval = 0.;
               
               for (int i = 0; i < BLOCK_SIZE; i++)          
                   {for (int j = 0; j < BLOCK_SIZE; j++)           
                    {if (sortedspectrum[j] < sortedspectrum[i])               
                      {float tmp = sortedspectrum[i]; sortedspectrum[i] = sortedspectrum[j]; sortedspectrum[j] = tmp;}}}
                 
                 fmax1 = sortedspectrum[0]; fmax2 = sortedspectrum[1]; fmax3 = sortedspectrum[2];     

                 for (int i = 0; i < BLOCK_SIZE; i++)                  
                        {if(averagespectrum_lin[i] == fmax1)
                                 {fmax1bin = i;} // ESP_LOGW(TAG,"fmax1 bin = %i \n \n -------- \n \n", i);
                        else if(averagespectrum_lin[i] == fmax2)
                                 {fmax2bin = i;} // ESP_LOGW(TAG,"fmax2 bin = %i \n \n -------- \n \n", i);
                        else if(averagespectrum_lin[i] == fmax3)
                                 {fmax3bin = i;}} // ESP_LOGW(TAG,"fmax3 bin = %i \n \n -------- \n \n", i);
                                  
                     //ESP_LOGW(TAG,"fmax1 = %f in bin #%i \n \n -------- \n \n", sortedspectrum[0], fmax1bin); 
                     //ESP_LOGW(TAG,"fmax2 = %f in bin #%i \n \n -------- \n \n", sortedspectrum[1], fmax2bin); 
                     //ESP_LOGW(TAG,"fmax3 = %f in bin #%i \n \n -------- \n \n", sortedspectrum[2], fmax3bin); 
              
                 // cast averaged spectrum from float to 16-bit integers
                 memcpy(averagespectrum_lin_int, averagespectrum_lin, N); 

                 // reset running averages
                 avg_sum = 0; avg_ms = 0; avg_rms = 0; avg_sum_samples = 0; avg_ms_samples = 0; avg_rms_samples = 0;

                 // select bins for LoRaWAN payload
                 for (uint16_t i=0; i<SELECTEDBINS; i++) 
                        { 
                             for (uint16_t j=0; j<NR_OF_BINS_TO_COMBINE; j++) 
                                 {
                                    cspctrm[i] += (vReal[(STARTBIN+i*NR_OF_BINS_TO_COMBINE+j)]);
                                 }
                             // cspctrm[i] /= fft_cnt;} average over fft_cnt
                             if (cspctrm[i]>maxval) maxval=cspctrm[i];
                        } 

                 for (uint16_t i=0; i<SELECTEDBINS; i++) 
                          {   
                              // scale amplitude values to 8-bit 
                              s_spectrum[i] =+ 255*(cspctrm[i] / maxval);  
                              //ESP_LOGW(TAG,"s_spectrum[%d]= %d \n", i, s_spectrum[i]); 
                              //ESP_LOGW(TAG,"cspctrm[%d]= %f \n", i, cspctrm[i]);
                          }

                 // increment counter of passes done
                 fft_cnt++; ESP_LOGW(TAG,"\n -------- \n FFT_CNT = %d @ AVG_FFT_NR = %d \n", fft_cnt, AVG_FFT_NR);
                 }
              }
