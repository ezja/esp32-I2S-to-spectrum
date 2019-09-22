
// ***********************************************************
// ESP32 & I2S mic --> FFT --> averaging --> SPIFFS / LoRaWAN
//
// FFT routines uit ESP-DSP library:
// https://github.com/espressif/esp-dsp
//
// LoRaWAN compression via Wouter Brok:
// https://github.com/wjmb/FeatherM0_BEEP
// ***********************************************************

// ***************************************************************
// @ BLOCK_SIZE = 8192: 
// Used static DRAM:  143392 bytes (  37344 available, 79.3% used) 
// Used static IRAM:  105392 bytes (  25680 available, 80.4% used) 
//
// @ BLOCK_SIZE = 2048: 
// Used static DRAM:  114996 bytes (  65740 available, 63.6% used)
// Used static IRAM:   46424 bytes (  84648 available, 35.4% used)
// ***************************************************************

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
#include "i2s_fft.h"

// ****************
// I2S mic select: 
// #define SPH6045
// #define INMP441
// ****************

#define SAMPLERATE 6250 
// FFT window size
#define BLOCK_SIZE 2048
// I2S ringbuffer size
#define I2S_BUFFER_SIZE 2048      
#define I2S_BUFFER_TYPE RINGBUF_TYPE_BYTEBUF
// FFTs per LoRaWAN payload
#define AVG_FFT_NR 2 

size_t I2S_size = 2048;
static int32_t samples[2048];
static RingbufHandle_t i2s_buffer = NULL;

static float y_cf[BLOCK_SIZE*2]; 
static float x1[BLOCK_SIZE];
static float wind[BLOCK_SIZE*2];
static float output_data_float_log[BLOCK_SIZE];
static float output_data_float_lin[BLOCK_SIZE];
static float averagespectrum_log[BLOCK_SIZE];
static float averagespectrum_lin[BLOCK_SIZE];
static uint16_t averagespectrum_lin_int[BLOCK_SIZE];
static float spectrumsum_log[BLOCK_SIZE];
static float spectrumsum_lin[BLOCK_SIZE];
static float cspctrm[SELECTEDBINS];

static uint16_t s_codehistogram[64]; 
static uint8_t s_code[64]; 

// deep sleep interval
int64_t sleep_us = 50000000;

// ***********************************
// s_spectrum is de LoRaWAN payload!
//
// momenteel 1 byte per freq. bin,
// amplitude gecomprimeerd naar uint8
// ***********************************
uint8_t s_spectrum[SELECTEDBINS];

static const char* TAG = "I2S --> FFT";


// ******************
// I2S config struct
// ******************

static void init_i2s()
{
   	    i2s_config_t i2s_config_rx = {

		.mode = I2S_MODE_MASTER | I2S_MODE_RX,

		.sample_rate = SAMPLERATE,

		.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,

		// L/RSEL voor 2-kanaals: 
        // .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,

		// L/RSEL voor 1-kanaals, met de pin ("SEL" bij SPH6045 en "L/R" bij INMP441) naar GND:
		.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
	
		// ESP32 technical reference p.306 voor MSB/LSB first VS "Phillips mode"
        // NB! voor INMP441 zie datasheet p. 11 -- eerste bit van elk woord moet worden verschoven:
        // "MSB of each word is delayed by one SCK cycle from the start of each half frame"
		.communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_LSB),
		
		// NB! 2048 samples per DMA write @ 256 samples:
        // 8 buffers, +1 want eerste buffer wordt niet gebruikt door I2S!  
		.dma_buf_count = 9,                       
		.dma_buf_len = 256,  

		.use_apll = 1, 
		.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1        
		};

	    // pin config 
	    i2s_pin_config_t pin_config_rx = {
		    .bck_io_num = GPIO_NUM_21, 
		    .ws_io_num = GPIO_NUM_18,
		    .data_out_num = -1,
		    .data_in_num = GPIO_NUM_19
		};

	i2s_driver_install(I2S_NUM_1, &i2s_config_rx, 0, NULL);
	i2s_set_pin(I2S_NUM_1, &pin_config_rx);
	i2s_stop(I2S_NUM_1);	
	i2s_start(I2S_NUM_1);
}


// *****************************************
// I2S input -> ringbuffer *samples
// met moving average -- TODO: exponentieel?
// *****************************************

void i2s_samplesread_ringbuf()	 
{   
    // init_i2s();
    // BaseType_t i2s_samples_ring = xRingbufferSend(i2s_buffer, (void *)samples, 2048, (portTickType)portMAX_DELAY);

    uint64_t sum = 0; uint64_t ms = 0; uint64_t rms = 0; uint64_t maxval = 0;
    uint64_t avg_rms = 0; uint64_t sum_rms = 0; uint64_t sum_rms_mov = 0; uint64_t avg_cnt = 50;
    uint64_t avg_running = 1;  uint64_t moving_avg = 0; uint64_t moving_spl = 0;

    // *****************************************************************************************************************************
    // evt. static ringbuffer in IRAM 
    // *****************************************************************************************************************************
    // !! CONFIG_FREERTOS_SUPPORT_STATIC_ALLOCATION=y in sdkconfig !!
    //
    // StaticRingbuffer_t *buffer_struct = (StaticRingbuffer_t *)heap_caps_malloc(sizeof(StaticRingbuffer_t), MALLOC_CAP_INTERNAL);
    // uint8_t *buffer_storage = (uint8_t *)heap_caps_malloc(sizeof(uint8_t)*I2S_BUFFER_SIZE, MALLOC_CAP_INTERNAL);
    // RingbufHandle_t i2s_buffer = xRingbufferCreateStatic(I2S_BUFFER_SIZE, I2S_BUFFER_TYPE, buffer_storage, buffer_struct);
    // *****************************************************************************************************************************

    while(1){
              for (int i=0; i < avg_cnt; i++)
                             {   
                                 i2s_read_bytes(
                                    I2S_NUM_1, 
                                    (char *)samples, 
                                    I2S_BUFFER_SIZE,   
                                    //&bytes_read,  
                                    portMAX_DELAY); 

                                 sum = 0; maxval = 0; ms = 0; rms = 0; avg_rms = 0; sum_rms = 0;

                                 for (int i=0; i < I2S_BUFFER_SIZE; i++)
                                         {       
                                             sum +=  samples[i] / I2S_BUFFER_SIZE;
                                         }

                                 for (int i=0; i < I2S_BUFFER_SIZE; i++)
                                         {       
                                             if (samples[i]>maxval) maxval=samples[i];
                                         }

                                  sum = sum / I2S_BUFFER_SIZE;
                                  rms = (sqrt(pow(sum, 2))) / I2S_BUFFER_SIZE;
                                  rms = rms/maxval;
                                  sum_rms += rms;
                                  sum = 0; maxval = 0; ms = 0; rms = 0; 
                              }
                                         
                 avg_rms = sum_rms / avg_cnt;
                 sum_rms_mov += sum_rms;
                 avg_running++;
                 moving_avg += avg_rms;
                 moving_avg = moving_avg / avg_running;
                 
                 ESP_LOGW(TAG,"\n -------- \n \n");      
	             ESP_LOGW(TAG,"I2S avg RMS = %llu \n \n -------- \n \n", avg_rms);
	             ESP_LOGW(TAG,"I2S moving_avg RMS = %llu \n \n -------- \n \n", moving_avg);

                 const uint8_t FULL_SCALE_DBSPL = 120; // FULL SCALE dBSPL (AOP = 116dB SPL)
                 const uint32_t FULL_SCALE_DBFS = 20*log10(pow(2,24));
                 const uint32_t spl = FULL_SCALE_DBSPL-(FULL_SCALE_DBFS-20*log10(sqrt(2) * avg_rms)); 
                 const uint32_t moving_spl = FULL_SCALE_DBSPL-(FULL_SCALE_DBFS-20*log10(sqrt(2) * moving_avg)); 

                 ESP_LOGW(TAG,"\n -------- \n \n");      
                 ESP_LOGW(TAG,"I2S avg SPL = %d \n \n -------- \n \n", spl); 
                 ESP_LOGW(TAG,"I2S moving_avg SPL = %d \n \n -------- \n \n", moving_spl); 

                 avg_rms = 0; sum_rms = 0; 
         }   
}


// *************************
// I2S input zonder ringbuf
// *************************

void i2s_samplesread()	 
{   
	    uint32_t *samples_ptr = samples;

        while(1){
                   i2s_read_bytes(
                         I2S_NUM_1, 
                         (uint32_t *)samples, 
                         // read size * 4?
                         I2S_size*4,   
                         //&bytes_read,  
                         portMAX_DELAY); 
                }
}


// **********************************************************
// FFT radix-2 + windowing, uit ESP-DSP lib 
// aangepaste versie van /modules/support/dsps_view.cpp
// **********************************************************

void fft(){

    int16_t N = BLOCK_SIZE;

    float sum = 0.; float ms = 0.; float rms = 0.;
    float avg_sum = 0.; float avg_ms = 0.; float avg_rms = 0.;

        while(1){

            memset(y_cf,0,sizeof(y_cf)); memset(x1,0,sizeof(x1)); memset(wind,0,sizeof(x1));
            
            memset(s_codehistogram,0,sizeof(s_codehistogram));
            memset(output_data_float_lin,0,sizeof(output_data_float_lin));
            memset(output_data_float_log,0,sizeof(output_data_float_log));

            memset(averagespectrum_lin,0,sizeof(averagespectrum_lin));
            memset(averagespectrum_log,0,sizeof(averagespectrum_log));
            memset(spectrumsum_log,0,sizeof(spectrumsum_log));
            memset(spectrumsum_lin,0,sizeof(spectrumsum_lin));

            dsps_fft2r_deinit_fc32(); dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);

			for (uint16_t i=0; i<AVG_FFT_NR; i++)
            {
                   dsps_fft2r_deinit_fc32();
                   dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);

                   // Hanning window 
                   dsps_wind_hann_f32(wind, N);

                   // test signaal, sinusbank met random amp & freq
                   dsps_tone_gen_f32(x1, N, (esp_random() / (float) RAND_MAX), ((esp_random() / ((float) RAND_MAX)) / 2.) - 1., 0);            
                        
				   for (uint16_t i = 0; i < BLOCK_SIZE ; i++) 
                       { 					   

                       // ****************************************************
                       // test signaal sinusbank als input
					   // y_cf[i * 2 + 0] = x1[i] * wind[i];                    
                       // ****************************************************

                       // ****************************************************
				       // IMNP401 mic als input -- shift van int32 naar int24
					   // #ifdef INMP441
					   // y_cf[i * 2 + 0] = ((samples[i] <<= 8) + 0.0000000000001) * wind[i]; 
					   // #endif                     
                       // ****************************************************		
		  
                       // ****************************************************
				       // SPH6045 mic als input -- shift van int32 naar int18 
					   // #ifdef SPH6045
                          y_cf[i * 2 + 0] = ((samples[i] >>= 14) + 0.0000001) * wind[i];
					   // #endif
                       // ****************************************************

                       // y_cf[] is complex, maar input is reeël:
                       // y_cf[i * 2 + 0] = reeël component
                       // y_cf[i * 2 + 1] = imaginair component = 0
					      y_cf[i * 2 + 1] = 0;               

                       }

                     dsps_fft2r_deinit_fc32();
                     dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);

                     // FFT op array y_cf "in-place" 
                     dsps_fft2r_fc32(y_cf, N);

                     // bit-reverse "in place"
                     dsps_bit_rev_fc32(y_cf, N);
                     dsps_cplx2reC_fc32(y_cf, N);
			
                     // en terugvlechten naar 1 array
                     for (int i = 0 ; i < N/2 ; i++) 
                             {
                               output_data_float_log[i] = 10 * log10f((y_cf[i * 2 + 0] * y_cf[i * 2 + 0] + y_cf[i * 2 + 1] * y_cf[i * 2 + 1])/N);
                               output_data_float_lin[i] = ((y_cf[i * 2 + 0] * y_cf[i * 2 + 0] + y_cf[i * 2 + 1] * y_cf[i * 2 + 1])/N);		 
							   // ESP_LOGW(TAG,"FFT bin[%i] = %i db \n" , i, output_data[i]);
                             }


                     for (int i=0; i < N / 2; i++)
                             {   
								spectrumsum_lin[i] += output_data_float_lin[i];
                                spectrumsum_log[i] += output_data_float_log[i];
                             }


                     for (int i=0; i < N / 2; i++)
                             {   
                                sum +=  output_data_float_lin[i];
                             }

					 // RMS per spectrum
                     ms = powf((sum / BLOCK_SIZE), 2);
                     rms = sqrtf(ms);   
	                 // ESP_LOGW(TAG,"----- SPECTUM #%d --> RMS = %f ----- \n \n", i, rms);

                     ms = 0; sum = 0; rms = 0;

                     dsps_fft2r_deinit_fc32();
                 }


				 // moving average, lin spectrum
                 for (int i=0; i < N / 2; i++)
                             { 
                             // "headroom" voor extra precisie later bij cast van float naar int:        
                             // averagespectrum_lin[i] = (spectrumsum_lin[i] / AVG_FFT_NR) * 100.;
                                averagespectrum_lin[i] = (spectrumsum_lin[i] / AVG_FFT_NR);
                             }
							 
				// moving average, log spectrum
                 for (int i=0; i < N / 2; i++)
                             {         
                              averagespectrum_log[i] = spectrumsum_log[i];
                             }

				// RMS van moving average
                 for (int i=0; i < N / 2; i++)
                     {         
                      avg_sum += averagespectrum_lin[i];
                      // ESP_LOGW(TAG,"averagespectrum_lin[%i]= %f \n", i, averagespectrum_lin[i]);       
                     }

                 avg_ms = powf((avg_sum), 2);
                 avg_rms = sqrtf(avg_ms); 

                 ESP_LOGW(TAG,"\n -------- \n \n");      
	             ESP_LOGW(TAG,"avg RMS = %f \n \n -------- \n \n", avg_rms);     
                 const uint8_t FULL_SCALE_DBSPL = 120;  // SPL in dB 
                 const uint32_t FULL_SCALE_DBFS = 20*log10(pow(2,24)); // 24 bit @ INMP441, 18 bit @ SPH6045
                 const uint32_t avg_spl = FULL_SCALE_DBSPL-(FULL_SCALE_DBFS-20*log10(sqrt(2) * avg_rms));  
	             ESP_LOGW(TAG,"avg SPL = %i \n \n -------- \n \n", avg_spl);
                 avg_sum = 0; avg_ms = 0; avg_rms = 0;

                 // avg spectrum in float cast --> int
                 memcpy(averagespectrum_lin_int, averagespectrum_lin, N);

                 // fmax --  max amplitude bins
                 int fmax1bin, fmax2bin, fmax3bin;
                 float fmax1, fmax2, fmax3;
                 float sortedspectrum[BLOCK_SIZE];

                 memcpy(sortedspectrum, averagespectrum_lin, sizeof(averagespectrum_lin));

	             for (int i = 0; i < BLOCK_SIZE; i++)                     
	                 {
		                for (int j = 0; j < BLOCK_SIZE; j++)           
		                {
			                if (sortedspectrum[j] < sortedspectrum[i])               
			                {
				                float tmp = sortedspectrum[i];         
				                sortedspectrum[i] = sortedspectrum[j];           
				                sortedspectrum[j] = tmp;        
			                }
		                }
                     }
                 
                 fmax1 = sortedspectrum[0]; fmax2 = sortedspectrum[1]; fmax3 = sortedspectrum[2];     
    
                 for (int i = 0; i < BLOCK_SIZE; i++)
                        { 

                        if(averagespectrum_lin[i] == fmax1)
                                 {
                                      // ESP_LOGW(TAG,"fmax1 bin = %i \n \n -------- \n \n", i);
                                      fmax1bin = i;
                                 }

                        else if(averagespectrum_lin[i] == fmax2)
                                 {
                                      // ESP_LOGW(TAG,"fmax2 bin = %i \n \n -------- \n \n", i);
                                      fmax2bin = i;
                                 }

                        else if(averagespectrum_lin[i] == fmax3)
                                 {
                                      // ESP_LOGW(TAG,"fmax3 bin = %i \n \n -------- \n \n", i);
                                      fmax3bin = i;
                                 }
                         }         

                     ESP_LOGW(TAG,"fmax1 = %f in bin #%i \n \n -------- \n \n", sortedspectrum[0], fmax1bin); 
                     ESP_LOGW(TAG,"fmax2 = %f in bin #%i \n \n -------- \n \n", sortedspectrum[1], fmax2bin); 
                     ESP_LOGW(TAG,"fmax3 = %f in bin #%i \n \n -------- \n \n", sortedspectrum[2], fmax3bin); 

				 			
                     // LoRa prep, spectrum compression
                     memset(cspctrm,0,sizeof(cspctrm));
                     float maxval=0.;

                     ESP_LOGW(TAG,"\n -------- \n \n");

                     for (uint16_t i=0; i<SELECTEDBINS; i++) 
                          {
                             for (uint16_t j=0; j<NR_OF_BINS_TO_COMBINE; j++) 
                                 {
                                     cspctrm[i] += (averagespectrum_lin[(STARTBIN+i*NR_OF_BINS_TO_COMBINE+j)]);
                                     
                                 }
                          if (cspctrm[i]>maxval) maxval=cspctrm[i];
                          }

                     for (uint16_t i=0; i<SELECTEDBINS; i++) 
                          {
                              s_spectrum[i] = 255*(cspctrm[i]/maxval);
                              ESP_LOGW(TAG,"s_spectrum[%d]= %d \n", i, s_spectrum[i]);   
                              ESP_LOGW(TAG,"cspctrm[%d]= %f \n", i, cspctrm[i]);     
                          }

				      // linear plot
                      dsps_view(averagespectrum_lin, BLOCK_SIZE / 2, 160, 40, 0, 1500, '|');
	
                      dsps_fft2r_deinit_fc32(); 
        }
}


// ***********
// heap debug 
// ***********

void heapdebug()
/*      {
/*                ESP_LOGW(TAG,"\n -------- \n \n");*/
/*                ESP_LOGW(TAG,"\n -------- \n 8BIT\n");*/
/*                heap_caps_print_heap_info(MALLOC_CAP_8BIT);*/
/*                ESP_LOGW(TAG,"\n -------- \n \n");*/
/*                ESP_LOGW(TAG,"\n -------- \n 32 BIT\n");*/
/*                heap_caps_print_heap_info(MALLOC_CAP_32BIT);*/
/*                ESP_LOGW(TAG,"\n -------- \n \n");*/
/*                ESP_LOGW(TAG,"\n -------- \n EXEC \n");*/
/*                heap_caps_print_heap_info(MALLOC_CAP_EXEC);*/
/*                ESP_LOGW(TAG,"\n -------- \n \n");*/
/*                ESP_LOGW(TAG,"\n -------- \n SPIRAM \n");*/
/*                heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);*/
/*                ESP_LOGW(TAG,"\n -------- \n \n");*/
/*                ESP_LOGW(TAG,"\n -------- \n DMA \n");*/
/*                heap_caps_print_heap_info(MALLOC_CAP_DMA);*/
/*                ESP_LOGW(TAG,"\n -------- \n \n");*/
/*                ESP_LOGW(TAG,"\n -------- \n INTERNAL \n");*/
/*                heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);*/
/*                ESP_LOGW(TAG,"\n -------- \n \n");*/
/*                ESP_LOGW(TAG,"\n -------- \n \n");*/
/*      }*/


// **********************
// bubble sort, aflopend
// **********************

void sort(uint8_t *index, float *output_data_float_lin, uint8_t nr)
{

  for (uint8_t i=0; i<nr; i++)
  {
    index[i] = i;
  }
  bool swapped; 
  for (uint8_t i=0; i<nr-1; i++) 
  { 
    swapped = false; 
    for (uint8_t j=0; j<nr-1-i; j++) 
    { 
      if (output_data_float_lin[j]<output_data_float_lin[j+1])
      { 
        const float tmp = output_data_float_lin[j];
        output_data_float_lin[j] = output_data_float_lin[j+1];
        output_data_float_lin[j+1] = tmp;
        const float tmpi = index[j];
        index[j] = index[j+1];
        index[j+1] = tmpi;
        swapped=true;
      } 
    }
    if (swapped == false) break; 

  }  
}


// **************************************
// codering van spectrum via Wouter Brok 
// TTN decodering mij nog niet duidelijk!
// **************************************

uint8_t calc_spectralcode(uint32_t *output_data_float_lin)
{

  uint32_t u1 = 0;
  uint32_t u2 = 0;
  uint32_t u3 = 0;
  uint32_t u4 = 0;

  for (uint8_t i=0; i<24; i++) 
            { 
             s_codehistogram[i]=255*((float)s_codehistogram[i]/BLOCK_SIZE); 
            }

  // frequentie gebieden? 
  // W.Brok noemt: A.F. Rybochkin et al, Moshen2015_methods, Moshen2015_avtoreferat 

  // @ BLOCK_SIZE == 2048:                                                
    u1 = output_data_float_lin[33]+output_data_float_lin[34]+
         output_data_float_lin[35]+output_data_float_lin[36]; // 205-225 Hz
    u2 = output_data_float_lin[46]+output_data_float_lin[47]+
         output_data_float_lin[48]+output_data_float_lin[49]; // 280-300 Hz
    u3 = output_data_float_lin[53]+output_data_float_lin[54]+
         output_data_float_lin[55]+output_data_float_lin[56]; // 325-345 Hz
    u4 = output_data_float_lin[64]+output_data_float_lin[65]+
         output_data_float_lin[66]+output_data_float_lin[67]; // 390-410 Hz

  uint8_t code = 0;

  code+=(u1>u2)?(1<<5):0;
  code+=(u1>u3)?(1<<4):0;
  code+=(u1>u4)?(1<<3):0;
  code+=(u2>u3)?(1<<2):0;
  code+=(u2>u4)?(1<<1):0;
  code+=(u3>u4)?(1<<0):0;

  /*  const uint8_t code = calc_spectralcode(averagespectrum_lin);*/
  /*  s_codehistogram[code]++;*/
  /*  sort(s_code, s_codehistogram, 64);*/
  
  return code;
}


// ***********
// deep sleep 
// ***********

void startDeepSleep();
{
    i2s_stop(I2S_NUM_1);
    esp_sleep_enable_timer_wakeup(sleep_us);	
    esp_deep_sleep_start ();
}


// *****************************
// avg spectrum --> SPIFFS
// I2S mic samples --> SPIFFS
// *****************************

void spiffswrite()
{

    while(1){

        // *********************************************************************************************************
        // via ringbuffer
        // *********************************************************************************************************
        // uint32_t *i2s_samples = (uint32_t *)xRingbufferReceive(i2s_buffer, &I2S_size, (portTickType)portMAX_DELAY);
        // vRingbufferReturnItem(i2s_buffer, (void *)i2s_samples);
        // *********************************************************************************************************

        int16_t total_i2s = 0;
        int16_t total_fft = 0;

        int16_t i2s_cnt_spiffs = 0;
		int16_t fft_cnt_spiffs = 0;

        // 2.25 MB voor I2S = 68 sec (32KB p/s @ 8192kHz, twee-kanaals)
        int32_t SPIFFS_size_i2s = 2250000; 
        // FFT is 2KB per spectrum: 1024 bins @ 16-bit depth 
        int16_t SPIFFS_size_fft = 250000;

	    FILE* fp_i2s = fopen("/spiffs/i2s", "a");
	    FILE* fp_fft_output_data = fopen("/spiffs/fft_output_data", "a");

        // i2s rec --> SPIFFS (append)
    	for(int i = 0; i < SPIFFS_size_i2s; i++)
            {   
				i2s_cnt_spiffs = fwrite(samples, 2, 2, fp_i2s); 
                total_i2s += i2s_cnt_spiffs; // counter 
				ESP_LOGW(TAG,"%d curr counter i2s write -> spiffs \n", i2s_cnt_spiffs);
				ESP_LOGW(TAG,"%d totaal i2s \n", total_i2s);

                if(total_i2s == SPIFFS_size_i2s)
                 {
                       fclose(fp_i2s);
                   //  memcpy(dataSPIFFS, y_cf, sizeof(dataSPIFFS));
			           ESP_LOGW(TAG,"\n \n i2s --> spiffs .. \n klaar  \n");
                       total_i2s = 0;
			     }
	        }	

        // FFT output data --> SPIFFS (append)
    	for(int i = 0; i < SPIFFS_size_fft; i++)
            {   
				fft_cnt_spiffs = fwrite(averagespectrum_lin_int, 2, 2, fp_fft_output_data); 
                total_fft += fft_cnt_spiffs; // counter 
				ESP_LOGW(TAG,"%d curr counter i2s write -> spiffs \n", fft_cnt_spiffs);
				ESP_LOGW(TAG,"%d totaal fft \n", total_fft);

                if(total_fft == SPIFFS_size_fft)
                 {
                       fclose(fp_fft_output_data);
			           ESP_LOGW(TAG,"\n \n i2s --> spiffs \n klaar \n \n");
                   //  memcpy(dataSPIFFS, y_cf, sizeof(dataSPIFFS));
                       total_fft = 0;
			     }
	         }	
        }
}


// *******************
// I2S task --> CPU 1
// *******************

void startTask_i2s_samplesread(){

       init_i2s();
       // i2s_buffer = xRingbufferCreate(2048, RINGBUF_TYPE_BYTEBUF); 

       xTaskCreatePinnedToCore(&i2s_samplesread, "i2s_samplesread", 8192, NULL, configMAX_PRIORITIES - 1, NULL, 1);

                    }

// *******************
// FFT task --> CPU 0
// *******************

void startTask_fft(){

       xTaskCreatePinnedToCore(&fft, "FFT", 32768, NULL, configMAX_PRIORITIES - 1, NULL, 0);

                    }

// **********************
// SPIFFS task --> CPU 1
// **********************

void startTask_spiffswrite(){

        // *************
        // SPIFFS setup    
        // *************

        /*    esp_vfs_spiffs_conf_t conf = {*/
        /*      .base_path = "/spiffs",*/
        /*      .partition_label = NULL,*/
        /*      .max_files = 5,*/
        /*      .format_if_mount_failed = true*/
        /*        };*/

        /*    esp_vfs_spiffs_register(&conf); */

        /*    struct stat st;*/
        /*    if (stat("/spiffs/rec", &st) == 0) {*/
        /*        unlink("/spiffs/rec");*/
        /*        }; */

       xTaskCreatePinnedToCore(&spiffswrite, "spiffswrite", 32768, NULL, configMAX_PRIORITIES - 1, NULL, 1);
                            
                    }


void app_main()
{

       startTask_i2s_samplesread(); 

       startTask_fft();

       startTask_spiffswrite(); 

}


