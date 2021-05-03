//// This tab holds the logic for measuring the sound frequencies
/**  Based upon:  https://revspace.nl/EspAudioSensor 
 * ESP32 I2S Sound / Frequency Level Measurement with INMP.
 * 
 * INMP441 GND to ESP32 GND
 * INMP441 VDD to ESP32 3.3V
 * INMP441 SD  to ESP32 (see PIN structure)
 * INMP441 SCK to ESP32 (see PIN structure)
 * INMP441 WS  to ESP32 (see PIN structure)
 * INMP441 L/R to ESP32 GND  (TO use the left channel only) 

 */

#include <Arduino.h>
#include <driver/i2s.h>
#include "arduinoFFT.h"   //local library adjusted for ESP32

// size of noise sample
#define SAMPLES      1024   // aka FFT size 
                            // 1024 FFT size = 512 bins
                            
                            
#define SAMPLE_RATE  8192 /*
                          // BIN BANDBREEDTE = SAMPLING RATE / FFT SIZE
                          // BIJ 2048 sample rate --> 1024 band fft
                          // 1024 / 1024 = 1 Hz per bin
                          // 
                          // BIJ 8192 sample rate --> 4096 band fft
                          // 4096 / 1024 = 4 Hz per bin
                          */
                          
#define OCTAVES 9 
#define BINS 10

// The frequency resolution of each spectral line is equal to the Sampling Rate divided by the FFT size.
int bandbreedte = SAMPLES / (SAMPLE_RATE / 2);

const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = SAMPLES;

// our FFT data
static float real[SAMPLES];
static float imag[SAMPLES];
static arduinoFFT fft(real, imag, SAMPLES, SAMPLES);
static float energy[BINS];


// 150 - 200 - 250 - 275 - 300 - 325 - 350 - 375 - 400 - 450 Hz
static const int bin_freq[]     = {    150,    200,   250 ,  275,  300,   325,   350,   375,   400,   450 };  

//static const float aweighting[] = { -39.4, -32.8, -21.2 ,-12.4, -5.9, -1.6,  0.6,  1.1,  1.1 }; 
static const float aweighting[] = { 1  , 1  , 1  , 1  , 1  , 1  , 1 , 1  , 1 }; 

static void print(const char *fmt, ...)
{
    // format it
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // send it to serial
    Serial.write(buf);
}

static void integerToFloat(int32_t * integer, float *vReal, float *vImag, uint16_t samples)
{
    for (uint16_t i = 0; i < samples; i++) {
        vReal[i] = (integer[i] >> 16) / 10.0;
        vImag[i] = 0.0;
    }
}

// calculates energy from Re and Im parts and places it back in the Re part (Im part is zeroed)
static void calculateEnergy(float *vReal, float *vImag, uint16_t samples)
{
    for (uint16_t i = 0; i < samples; i++) {
        vReal[i] = sq(vReal[i]) + sq(vImag[i]);
        vImag[i] = 0.0;
    }
}

// sums up energy in bins per octave
static void sumEnergy(const float *bins, float *energies, int bin_size, int num_octaves)
{
    // skip the first bin
    int bin = bin_size;
    
    // non-octave version
    for (int octave = 0; octave < num_octaves; octave++) {
        float sum = 0.0;
        
        for (int i = 0; i < bin_size; i++) {
            sum += real[bin++];
        }
        
        energies[octave] = sum;
        bin_size *= 2;
    }
}

static float decibel(float v)
{
    return 10.0 * log(v) / log(10);
}

// converts energy to logaritmic, returns A-weighted sum
static float calculateLoudness(float *energies, const float *weights, int num_octaves, float scale)
{
    float sum = 0.0;
    for (int i = 0; i < num_octaves; i++) {
        float energy = scale * energies[i];
        sum += energy * pow(10, weights[i] / 10.0);
        //energies[i] = decibel(energy);              // do not turn levels per bin into decibels, but in leave in power
    }
    return decibel(sum);
}

bool init_sensor_INMP441(void)
{

    DPRINT("Configuring I2S...");
    esp_err_t err;

    // The I2S config as per the example
    const i2s_config_t i2s_config = {
        .mode                 = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),      // Receive, not transfer
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT, // LEFT Channel (L/R is connected to GRND)
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,       // Interrupt level 1
        .dma_buf_count        = 8,                          // number of buffers
        .dma_buf_len          = BLOCK_SIZE,                 // samples per buffer
        .use_apll = true
    };

    // The pin config as per the setup
    const i2s_pin_config_t pin_config = {
        .bck_io_num   = PIN.I2S_SCK,       // BCKL
        .ws_io_num    = PIN.I2S_WS,        // LRCL
        .data_out_num = -1,                // not used (only for speakers)
        .data_in_num  = PIN.I2S_SD         // DOUT
    };

    // Configuring the I2S driver and pins.
    // This function must be called before any I2S driver read/write operations.
    err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        DPRINT(F("Failed installing driver: ")); DPRINTLN(err);
        return false;
    }
    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        DPRINT(F("Failed setting pin: ")); DPRINTLN(err);
        return false;
    }
    DPRINTLN("I2S driver installed.");
    return true;
}

void measure_sound_freq()
{   DPRINTLN(F("---measure_sound_freq()"));
    wait_for_stabilization(10);     
    int   no_measures = 1;     //number of measurements
    float loudness_avg = 0;
    int energy_avg[OCTAVES];
    int count  = 0;
    for (count = 0; count < no_measures; count++)  // initialize array
    { energy_avg[count] = 0; }
 
    for ( count = 0; count < no_measures; count++) 
    {
      static int32_t samples[BLOCK_SIZE];
      // Read multiple samples at once and calculate the sound pressure
      size_t num_bytes_read;
      esp_err_t err = i2s_read(I2S_PORT,
                      (char *) samples,
                      BLOCK_SIZE,        // the doc says bytes, but its elements.
                      &num_bytes_read,
                      portMAX_DELAY);    // no timeout
      int samples_read = num_bytes_read / 8;    // PB: samples_read ??not used??

      // integer to float
      // waarom? FFT wil int16 samples
      integerToFloat(samples, real, imag, SAMPLES);

      // apply HANNING ipv flat top window
      fft.Windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);
      fft.Compute(FFT_FORWARD);
      
      // remove DC component
      fft.DCRemoval();

      /* SKIP DB BEREKENING EN MIDDELING
            
            // calculate energy in each bin
            //calculateEnergy(real, imag, SAMPLES);

            // sum up energy in bin for each octave
            //sumEnergy(real, energy, 1, BINS);

            // calculate loudness (in dB) per octave + A weighted loudness
            //float loudness = calculateLoudness(energy, aweighting, OCTAVES, 1.0);

            //loudness_avg = loudness_avg + loudness;
            //energy_avg[OCTAVES];
          
            /* store values and show 
            for (int i = 0; i < BINS; i++)
            {
              energy_avg[i] = energy_avg[i] + energy[i];
            }
            */
            
      */
    }
    
      // show result on screen
      for (int i = 0; i < BINS; i++)
      { 
        DPRINT(" B"); DPRINT(i); 
        DPRINT(" ("); DPRINT(bin_freq[i]); DPRINT("Hz)");DPRINT(":\t");
        for (int j = 1; j < real[i]/(count*100); j=j+4)
        {
          DPRINT("|");
        }
        DPRINT(" (");
        DPRINT(real[i]/count);
        DPRINTLN(")");
        
      }
        
      double x = FFT.MajorPeak(vReal, samples, SAMPLE_RATE);
      // print freq band nr met hoogste amplitude waarde
      DPRINTLN("PEAK FREQ:");
      DPRINT(fft.MajorPeak());
        
      //    print(" => Loudness %6.1f", loudness_avg/count);
      //    print("\n"
    
    // if < 0  microphone not connected and return 0
    if ((vReal[150 / bandbreedte]/count >= 0) and             
        (vReal[200 / bandbreedte]/count >= 0) and
        (vReal[250 / bandbreedte]/count >= 0) and
        (vReal[275 / bandbreedte]/count >= 0) and
        (vReal[300 / bandbreedte]/count >= 0) and
        (vReal[325 / bandbreedte]/count >= 0) and
        (vReal[350 / bandbreedte]/count >= 0) and
        (vReal[375 / bandbreedte]/count >= 0) and
        (vReal[400 / bandbreedte]/count >= 0) and
        (vReal[450 / bandbreedte]/count >= 0)
        )
        
    {   

       // format for lora payload
       int data;
       
       */
       // 10 frequency bins
       //
       // 2 bytes per bin --> 20 payload bytes
       // 
       // 150 - 200 - 250 - 275 - 300 - 325 - 350 - 375 - 400 - 450 Hz
       */
       
       // 150Hz bij 
       if (vReal[150 / bandbreedte] > 32767 ) 
       { data = 32767; }
       else
       { data = vReal[150 / bandbreedte]; }     
       DPRINT("data 150Hz: ");
       DPRINTLN(data);
       sensor_payload[PayLoad.INMP441]   = highByte(data);   // high byte
       sensor_payload[PayLoad.INMP441+1]   = lowByte(data);    // low byte

       // 200 Hz  
       if (vReal[200 / bandbreedte]) > 32767 )
       { data = 32767; }
       else
       { data = vReal[200 / bandbreedte]; }     
       DPRINT("data 200Hz: ");DPRINTLN(data);
       sensor_payload[PayLoad.INMP441+2]   = highByte(data);   // high byte
       sensor_payload[PayLoad.INMP441+3]   = lowByte(data);    // low byte

       // 250 Hz
       if (vReal[250  / bandbreedte] > 32767 )
       { data = 32767; }
       else
       { data = vReal[250 / bandbreedte]; }      
       DPRINT("data 250Hz: ");DPRINTLN(data);
       sensor_payload[PayLoad.INMP441+4]   = highByte(data);   // high byte
       sensor_payload[PayLoad.INMP441+5]   = lowByte(data);    // low byte
       
       // 275 Hz
       if (vReal[275 / bandbreedte] > 32767 )
       { data = 32767; }
       else
       { data = vReal[275 / bandbreedte]); }      
       DPRINT("data 275Hz: ");DPRINTLN(data);

       sensor_payload[PayLoad.INMP441+6]   = highByte(data);   // high byte
       sensor_payload[PayLoad.INMP441+7]   = lowByte(data);    // low byte
       
       // 300 Hz
       if (vReal[300 / bandbreedte] > 32767 )
       { data = 32767; }
       else
       { data = vReal[300 / bandbreedte]; }      
       DPRINT("data 300Hz: ");DPRINTLN(data);
       sensor_payload[PayLoad.INMP441+8]   = highByte(data);   // high byte
       sensor_payload[PayLoad.INMP441+9]   = lowByte(data);    // low byte
       
       // 325 Hz
       if (vReal[325 / bandbreedte] > 32767 )
       { data = 32767; }
       else
       { data = vReal[325 / bandbreedte]; }      
       DPRINT("data 325Hz: ");DPRINTLN(data);
       sensor_payload[PayLoad.INMP441+10]   = highByte(data);   // high byte
       sensor_payload[PayLoad.INMP441+11]   = lowByte(data);    // low byte
       
       // 350 Hz
       if (vReal[350 / bandbreedte] > 32767 )
       { data = 32767; }
       else
       { data = vReal[350  / bandbreedte]; }      
       DPRINT("data 350Hz: ");DPRINTLN(data);
       sensor_payload[PayLoad.INMP441+12]   = highByte(data);   // high byte
       sensor_payload[PayLoad.INMP441+13]   = lowByte(data);    // low byte
       
       // 375 Hz
       if (vReal[375 / bandbreedte] > 32767 )
       { data = 32767; }
       else
       { data = vReal[375 / bandbreedte]; }      
       DPRINT("data 375Hz: ");DPRINTLN(data);
       sensor_payload[PayLoad.INMP441+14]   = highByte(data);   // high byte
       sensor_payload[PayLoad.INMP441+15]   = lowByte(data);    // low byte
       
       // 400 Hz
       if (vReal[400 / bandbreedte] > 32767 )
       { data = 32767; }
       else
       { data = vReal[400 / bandbreedte]; }      
       DPRINT("data 400Hz: ");DPRINTLN(data);
       sensor_payload[PayLoad.INMP441+16]   = highByte(data);   // high byte
       sensor_payload[PayLoad.INMP441+17]   = lowByte(data);    // low byte
       
       // 450 Hz
       if (vReal[450 / bandbreedte]) > 32767 )
       { data = 32767; }
       else
       { data = vReal[450 / bandbreedte]; }      
       DPRINT("data 450Hz: ");DPRINTLN(data);
       sensor_payload[PayLoad.INMP441+18]   = highByte(data);   // high byte
       sensor_payload[PayLoad.INMP441+19]   = lowByte(data);    // low byte
      
    }
}
