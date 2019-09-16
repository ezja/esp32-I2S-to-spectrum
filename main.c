
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
//#include "esp_wifi.h"
//#include "esp_event_loop.h"
//#include "nvs_flash.h"

#define BLOCK_SIZE 2048
#define AVG_NR 10

static float y_cf[BLOCK_SIZE*2]; 
static float x1[BLOCK_SIZE];
static float output_data[2048*2];
static float output_data_float[BLOCK_SIZE];
static float wind[BLOCK_SIZE*2];
/*int16_t* y1_cf = &y_cf[0];*/
/*int16_t* y2_cf = &y_cf[BLOCK_SIZE];*/

IRAM_ATTR static int32_t samples[2048];

/* BLOCK_SIZE 8192: */
/* Used static DRAM:  143392 bytes (  37344 available, 79.3% used) */
/* Used static IRAM:  105392 bytes (  25680 available, 80.4% used) */


static void init_i2s()
{
	const int sample_rate = 8012;

   	 i2s_config_t i2s_config_rx = {
		.mode = I2S_MODE_MASTER | I2S_MODE_RX,

		.sample_rate = sample_rate,

		 // 32 bit depth, maar datasheet INMP441 zegt 24 bit?
		.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,

		// L/RSEL voor 2-kanaals:
		// .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,

		// L/RSEL voor 1-kanaals, verbonden aan GND:
		.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
	
		// MSB of LSB? LSB heeft weinig dynamisch bereik met INMP441?
		.communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
	
		// DMA latency
		// 64 buffers  
		.dma_buf_count = 9,        
		// 128 * 64 = 8192 samples per DMA write                     
		.dma_buf_len = 256,            
		.use_apll = 0, 
		.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1        
		};

	// PINOUT
	i2s_pin_config_t pin_config_rx = {
		.bck_io_num = GPIO_NUM_26, 
		.ws_io_num = GPIO_NUM_32,
		.data_out_num = -1,
		.data_in_num = GPIO_NUM_33
					};

	i2s_driver_install(I2S_NUM_1, &i2s_config_rx, 0, NULL);
	i2s_set_pin(I2S_NUM_1, &pin_config_rx);

	i2s_stop(I2S_NUM_1);
	i2s_start(I2S_NUM_1);

}


void samplesread()	 
{   
   i2s_read_bytes(
            I2S_NUM_1, 
            (char *)samples, 
            BLOCK_SIZE,   
            //&bytes_read,  
            portMAX_DELAY); 

/*       for (int i=0 ; i< N ; i++)*/
/*              {*/
/*               printf("I2S samples [%i] %i\n", i/2, samples[i]);*/
/*              }*/

}


void i2s_loop()
{

    // int N = sizeof(y_cf) / sizeof(float) / 2;
    int16_t N = 2048;

	init_i2s();

         while(1){

            samplesread(*samples);

            dsps_tone_gen_f32(x1, N, 1., 0.16, 0);            

            // 32 bit float window functie naar array "wind"
            dsps_wind_hann_f32(wind, N);

		    for (uint16_t i = 0; i < BLOCK_SIZE ; i++) 
                     {
/*                     y_cf[i * 2 + 0] = samples[i] << 8; // Re */
                       y_cf[i * 2 + 0] = x1[i] * wind[i];
					   y_cf[i * 2 + 1] = 0; // Im =  0
/*                     y_cf[i * 2 + 0] + 0.0000000000001;*/
/*                     y_cf[i] = y_cf[i] * wind[i];*/
                     }

/*         for (int i=0 ; i< N ; i++)*/
/*                 {*/
/*                   printf("Data samples complex array[%i] %i\n", i/2, y_cf[i]);*/
/*                 }*/

            // FFT init
			dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);

            // FFT op array y_cf "in-place" 
            dsps_fft2r_fc32(y_cf, N);

            // bit-reverse "in place"
            dsps_bit_rev_fc32(y_cf, N);

            // array met amps en fases omzetten naar twee losse arrays
            dsps_cplx2reC_fc32(y_cf, N);


/*             for (int i = 0 ; i < (N*2) ; i++)*/
/*                   {*/
/*                      output_data[i] = y_cf[i];*/
/*                      // output_data[i] = output_data[i]/INT16_MAX;*/
/*                   }*/

             for (int i = 0 ; i < N/2 ; i++) 
                   {
                     // output_data wordt float en logaritmisch/decibel geschaald
                     output_data_float[i] = 10 * log10f(((y_cf[i * 2 + 0] * y_cf[i * 2 + 0] 
                                                                  + y_cf[i * 2 + 1] * y_cf[i * 2 + 1])/N) + 0.0000001);

                    // printf("FFT bin[%i] = %i db \n" , i, output_data[i]);
                   }


             for (uint16_t j=0; j<AVG_NR; j++)
                     {

                     float averagespectrum[BLOCK_SIZE];
                     memset(averagespectrum,0,sizeof(averagespectrum));

                     float meanspectrum[BLOCK_SIZE];
                     memset(meanspectrum,0,sizeof(meanspectrum));

                     for (uint16_t j=0; j < BLOCK_SIZE; j++) 
                           {
                             meanspectrum[j] += samples[j];
                           }

                     for (int k=0; k < BLOCK_SIZE; k++)
                           {         
                             averagespectrum[k] += meanspectrum[k];
                           }

                     }

		   // printf("\n \n \n \n DONE \n \n \n \n ");

           dsps_view_spectrum(output_data_float, N/2, -100, 0);
           
           // dsps_view(output_data_float, N/2, 64, 10,  -100, 40, '|');
            
           dsps_fft2r_deinit_fc32();

        }
}

void spiffswrite()
{

        int16_t total_i2s = 0;
        int16_t total_fft = 0;

        int16_t i2s_cnt_spiffs = 0;
		int16_t fft_cnt_spiffs = 0;

        int32_t SPIFFS_size_i2s = 500000;
        int32_t SPIFFS_size_fft = 100000;


	    FILE* fp_i2s = fopen("/spiffs/i2s", "a");
	    FILE* fp_fft_output_data = fopen("/spiffs/fft_output_data", "a");


        // i2s rec --> SPIFFS (append)
    	for(int i = 0; i < SPIFFS_size_i2s; i++)
            {   
				i2s_cnt_spiffs = fwrite(samples, 2, 2, fp_i2s); 
                total_i2s += i2s_cnt_spiffs; // counter 
				printf("%d curr counter i2s write -> spiffs \n", i2s_cnt_spiffs);
				printf("%d totaal i2s \n", total_i2s);

                if(total_i2s == SPIFFS_size_i2s)
                 {
                       fclose(fp_i2s);
                   //  memcpy(dataSPIFFS, y_cf, sizeof(dataSPIFFS));
			           printf("\n \n i2s --> spiffs .. \n klaar  \n");
                       total_i2s = 0;
			     }
	         }	


        // FFT output data --> SPIFFS (append)
    	for(int i = 0; i < SPIFFS_size_fft; i++)
            {   
				fft_cnt_spiffs = fwrite(output_data, 2, 2, fp_fft_output_data); 
                total_fft += fft_cnt_spiffs; // counter 
				printf("%d curr counter i2s write -> spiffs \n", fft_cnt_spiffs);
				printf("%d totaal fft \n", total_fft);

                if(total_fft == SPIFFS_size_fft)
                 {
                       fclose(fp_fft_output_data);
			           printf("\n \n i2s --> spiffs \n klaar \n \n");
                   //  memcpy(dataSPIFFS, y_cf, sizeof(dataSPIFFS));
                       total_fft = 0;
			     }
	         }	
}


void app_main()
{

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
        };


    esp_vfs_spiffs_register(&conf);

/*    struct stat st;*/
/*    if (stat("/spiffs/rec", &st) == 0) {*/
/*        // Delete it if it exists*/
/*        unlink("/spiffs/rec")}; */


    xTaskCreatePinnedToCore(&i2s_loop, "i2s_loop", 32768, NULL, configMAX_PRIORITIES - 1, NULL, 0);

}
