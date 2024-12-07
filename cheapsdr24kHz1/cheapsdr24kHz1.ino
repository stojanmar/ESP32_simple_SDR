/* 
    With reference to:JF3HZB / T.UEBO
    Modified by S52UV / S.Markic
    Created on Dec, 2024
*/
//prilagojen na esp32 dev module
//Ver01 bere i2s na analog pinu
//ver02 dodam AM and FM demod
//ver04 output audio sampling with 24kHz
//ver05 zamenjani filtri
// Based on All_mode_SDR from T.Uebo
// Modified for solo ESP32 by S.Markic S52UV Dec. 2024

// mck pin has to be 0 I think?
#include <Arduino.h>
//#include "esp_task_wdt.h"
#include <esp_dsp.h>
#include <driver/i2s.h>
//#include "DacESP32.h"
#include <driver/adc.h>
#include <driver/dac.h>
#include <soc/syscon_reg.h>
#include "esp_adc_cal.h"
//#include <include/esp_intr_types.h>
#include "serialcommand.h"

#include "functions.hpp"

/*#define fsample 48000
#define DOWN_SAMPLE 6
#define BLOCK_SAMPLES (DOWN_SAMPLE*16)*/
#define ADC_CHANNEL   ADC1_CHANNEL_5  // GPIO33

// flags
int f_MODE = USB; //0 je USB
bool TXEN = false;
volatile int MUTE_Timer = 0;  //bilo je 300
volatile float G_adj = 1.0;

//buffers
int rxbuf[BLOCK_SAMPLES*2], txbuf[BLOCK_SAMPLES*2];
float Lch_in[BLOCK_SAMPLES], Rch_in[BLOCK_SAMPLES];
float Lch_out[BLOCK_SAMPLES], Rch_out[BLOCK_SAMPLES];
float RXsig[BLOCK_SAMPLES], I_RXsig[BLOCK_SAMPLES], Q_RXsig[BLOCK_SAMPLES]; 

float I_RXsig0[BLOCK_SAMPLES/DOWN_SAMPLE],  I_RXsig1[BLOCK_SAMPLES/DOWN_SAMPLE];
float Q_RXsig0[BLOCK_SAMPLES/DOWN_SAMPLE],  Q_RXsig1[BLOCK_SAMPLES/DOWN_SAMPLE];
/*// Decimation Filter
#define Ndec 512
float dec_fir_coeff[Ndec]; // Filter coefficients
float z_decL[Ndec], z_decR[Ndec];
fir_f32_t sfir_decL, sfir_decR;*/

#include "filter_coeffs/RX_DECIMATION_LPF.h"
// Ndec
float z_RXdecRe[Ndec], z_RXdecIm[Ndec], z_TXdec[Ndec];
fir_f32_t TXfir_dec, RXfir_decRe, RXfir_decIm;

#include "filter_coeffs/CPLX_BPF.h"
#include "filter_coeffs/AM_LPF.h"
// Ncmplx
float RX_CPLX_coeffs_Re[Ncmplx];
float RX_CPLX_coeffs_Im[Ncmplx];

float z_firRe[Ncmplx], z_firIm[Ncmplx];
float z_firRe0[Ncmplx], z_firIm0[Ncmplx];
float z_firRe1[Ncmplx], z_firIm1[Ncmplx];
//fir_f32_t TXfirRe, TXfirIm;
fir_f32_t RXfirRe, RXfirIm;
fir_f32_t AMfirRe0, AMfirIm0;
fir_f32_t AMfirRe1, AMfirIm1;

//#include "filter_coeffs/TX_IIR_LPF.h"
#include "filter_coeffs/RX_IIR_LPF.h"
float z_IIR0[2], z_IIR1[2], z_IIR2[2], z_IIR3[2];
float z_IIR4[2], z_IIR5[2], z_IIR6[2], z_IIR7[2];

float zRe=0.0f, zIm=0.0f, zDC=0.0f, zPH=0.0f;
float zMET=0.0f, zAGC=0.0f, zN=0.0f, zRX=0.0f;
float g_lim=0;

//float offsetdc = 1.74e9f;
#define offsetdc 1.468e9
// 24kHz or 6kHz Local OSC.
#define N_LO 4
float LO_I[N_LO]={1.0f, 0, -1.0f, 0};
float LO_Q[N_LO]={0, -1.0f, 0, 1.0f};
int pt_LO = 0;

//Ring Buffer
#define Ring_Buf_Size (Nfft+256)
volatile float d_Lch[Ring_Buf_Size];
volatile int Aubuf[Ring_Buf_Size];  //used it for audio DAC output  used only 2*BLOCKSAMPLES
volatile int pt_Au = 0;
volatile float d_Rch[Ring_Buf_Size];
volatile float Level[Ring_Buf_Size];
volatile int pt_Lvl = 0;

volatile int newring = 0;
volatile int pt = 0;
volatile int printing = 0;
volatile int index1 = 0;
volatile int uout = 128;
volatile int corenr;
volatile int filled = 0;

float glasnost = 0.8;
float *tmpRe;
float *tmpIm;
TaskHandle_t H_Task[2];


/*------------------------------------------------ -----------------------------------------------
  Setup
-------------------------------------------------- -----------------------------------------------*/
void setup(void) {
  //Serial.begin(115200);
  delay(50);

  digitalWrite(16, LOW);
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);

 
  xTaskCreatePinnedToCore(alt_task, "alt_task", 4096, NULL, 10, &H_Task[0], 0); 
  xTaskCreatePinnedToCore(SerialCom, "SerialCom", 4096, NULL, 5, &H_Task[1], 0);
  // I2S setup (Lables are defined in i2s_types.h, esp_intr_alloc.h)-------------------------------- -------
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),  // I2S receive mode with ADC
    .sample_rate = fsample,                                                          // sample rate
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,                                 // 16 bit I2S
    //.channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    //.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),   // I2S format
    .communication_format = I2S_COMM_FORMAT_I2S_LSB,
    //.communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,                                                        // none
    .dma_buf_count = 6,                                                           // number of DMA buffers
    .dma_buf_len = BLOCK_SAMPLES*2, //1000, //NUM_SAMPLES,                                                   // number of samples
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_11db);
  adc1_config_width(ADC_WIDTH_12Bit);
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

  i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);
  SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);
  i2s_adc_enable(I2S_NUM_0);

  //delay(1000);
  //second I2S config
  /*i2s_config_t i2s_config2 = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),  // I2S tx mode 
    .sample_rate = fsample,                                                          // sample rate
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,                                 // 16 bit I2S
    //.channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    //.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),   // I2S format
    .communication_format = I2S_COMM_FORMAT_I2S_LSB,
    //.communication_format = I2S_COMM_FORMAT_I2S_MSB,
    //.communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,                                                        // none
    .dma_buf_count = 6,                                                           // number of DMA buffers
    .dma_buf_len = BLOCK_SAMPLES*2, //1000, //NUM_SAMPLES,                                                   // number of samples
    .use_apll = false,
    //.tx_desc_auto_clear = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_driver_install( I2S_NUM_1, &i2s_config2, 0, NULL);

    i2s_pin_config_t pin_config2 = {
    .bck_io_num = -1, //this is BCK pin
    .ws_io_num = -1, // this is LRCK pin
    .data_out_num = -1, // this is DATA output pin
    .data_in_num = -1   //Not used
};
i2s_set_pin( I2S_NUM_1, &pin_config2);*/

  // set up FIR ------------------------------------------------------------------
  // Decimation  & FM LPF
  
  dsps_fird_init_f32(&RXfir_decRe, RX_DEC_FIR_coeffs, z_RXdecRe, Ndec, DOWN_SAMPLE, 0);
  dsps_fird_init_f32(&RXfir_decIm, RX_DEC_FIR_coeffs, z_RXdecIm, Ndec, DOWN_SAMPLE, 0);

  /*// Complex BPF for SSB/CW RX
  dsps_fir_init_f32(&RXfirRe, RX_CPLX_coeffs_Re, z_firRe, Ncmplx);
  dsps_fir_init_f32(&RXfirIm, RX_CPLX_coeffs_Im, z_firIm, Ncmplx);*/

  // Complex BPF for SSB/CW RX
  dsps_fir_init_f32(&RXfirRe, CPLX_coeffs_Re, z_firRe, Ncmplx);
  dsps_fir_init_f32(&RXfirIm, CPLX_coeffs_Im, z_firIm, Ncmplx);

  // Complex BPF for AM RX
  //dsps_fir_init_f32(&sfirAMRe, AM_FIR_coeffs, z_firRe, Nam);
  //dsps_fir_init_f32(&sfirAMIm, AM_FIR_coeffs, z_firIm, Nam);
  /*dsps_fir_init_f32(&AMfirRe0, RX_CPLX_coeffs_Re, z_firRe0, Ncmplx);
  dsps_fir_init_f32(&AMfirRe1, RX_CPLX_coeffs_Re, z_firRe1, Ncmplx);
  dsps_fir_init_f32(&AMfirIm0, RX_CPLX_coeffs_Im, z_firIm0, Ncmplx);
  dsps_fir_init_f32(&AMfirIm1, RX_CPLX_coeffs_Im, z_firIm1, Ncmplx);*/

  dsps_fir_init_f32(&AMfirRe0, AM_FIR_coeffs, z_firRe0, Nam); //Ncmplx);
  dsps_fir_init_f32(&AMfirRe1, AM_FIR_coeffs, z_firRe1, Nam); //Ncmplx);
  dsps_fir_init_f32(&AMfirIm0, AM_FIR_coeffs, z_firIm0, Nam); //Ncmplx);
  dsps_fir_init_f32(&AMfirIm1, AM_FIR_coeffs, z_firIm1, Nam); //Ncmplx);

  
  delay(1000);
  

}


/*------------------------------------------------ -----------------------------------------------
  Signal Process Loop
-------------------------------------------------- -----------------------------------------------*/
void loop(void) {
  MUTE_Timer--;
  if(MUTE_Timer<0) MUTE_Timer = 0;

  size_t readsize = 0;
  //Input from I2S codec
  esp_err_t rxfb = i2s_read(I2S_NUM_0, &rxbuf[0], BLOCK_SAMPLES*2*4, &readsize, portMAX_DELAY);
  if (rxfb == ESP_OK && readsize==BLOCK_SAMPLES*2*4)
   {
    digitalWrite(16, HIGH);
    int j;
    j=0;
    for (int i=0; i<BLOCK_SAMPLES; i++) {
      Lch_in[i] = ((float) rxbuf[j]) - offsetdc;
      Rch_in[i] = ((float) rxbuf[j+1]) - offsetdc;
      j+=2;

      /*d_Lch[pt] = Lch_in[i];
      //d_Rch[pt] = Rch_in[i];
      pt++;
      if( pt == Ring_Buf_Size ) {
        pt=0;
        newring = 1;*/
      }
    
    //-------Signal process ----------------------------
    
     /*for (int i=0; i<BLOCK_SAMPLES; i++) {
     Lch_out[i] =  6.0f*Lch_in[i];
     //Rch_out[i] = Rch_in[i];    
     }*/
    // Down conversion  fsample/4 to DC(base band)
      for(int i=0; i<BLOCK_SAMPLES; i++)
      {
        I_RXsig[i] = 6.0f*Lch_in[i]*LO_I[pt_LO];   //bilo je 10.0f
        Q_RXsig[i] = 6.0f*Lch_in[i]*LO_Q[pt_LO];
        pt_LO++; if(pt_LO==N_LO) pt_LO=0;
      }

    // Decimation (Down sampling) 
      dsps_fird_f32(&RXfir_decRe, I_RXsig, I_RXsig, BLOCK_SAMPLES);
      dsps_fird_f32(&RXfir_decIm, Q_RXsig, Q_RXsig, BLOCK_SAMPLES);

   float Re, Im;
      // RX SSB/CW --------------------------------
      if(f_MODE==USB || f_MODE==CWU || f_MODE==LSB || f_MODE==CWL)
      {
        dsps_fir_f32(&RXfirRe, I_RXsig, I_RXsig, BLOCK_SAMPLES/DOWN_SAMPLE);
        dsps_fir_f32(&RXfirIm, Q_RXsig, Q_RXsig, BLOCK_SAMPLES/DOWN_SAMPLE);

        if(f_MODE == USB || f_MODE == CWU)
          for(int i=0; i<BLOCK_SAMPLES/DOWN_SAMPLE; i++) RXsig[i] = (I_RXsig[i] - Q_RXsig[i])*G_adj;
        else
          for(int i=0; i<BLOCK_SAMPLES/DOWN_SAMPLE; i++) RXsig[i] = (I_RXsig[i] + Q_RXsig[i])*G_adj;

        //for S-Meter
        /*for(int i=0; i<BLOCK_SAMPLES/DOWN_SAMPLE; i++)
        {
          float s = RXsig[i]; 
          Level[pt_Lvl] = s*s;
          pt_Lvl++; if(pt_Lvl == Ring_Buf_Size) pt_Lvl = 0;
        }*/
        //Limitter(AGC)
        float T_R_AGC;
        if(f_MODE == USB || f_MODE == LSB) T_R_AGC = T_slowAGC;
        else T_R_AGC = T_fastAGC;
        for(int i=0; i<BLOCK_SAMPLES/DOWN_SAMPLE; i++)
        {
          RXsig[i] *=Gain_SSB_CW;
          float absl = fabs(RXsig[i])-lim_lvl;
          if( absl<0 ) absl = 0;

          if(MUTE_Timer == 0)
          { 
            if(absl>g_lim)
              g_lim = g_lim*T_At_AGC + absl*(1.0f-T_At_AGC);
            else
              g_lim*=T_R_AGC;
          }
          else
          {
            g_lim = 0;
          }
          RXsig[i] = RXsig[i] * (lim_lvl/(Comp_rate * g_lim + lim_lvl) );
        }
      }
      // RX AM ------------------------------------
      else if(f_MODE==AM)
      {
        dsps_fir_f32(&AMfirRe0, I_RXsig, I_RXsig0, BLOCK_SAMPLES/DOWN_SAMPLE);
        dsps_fir_f32(&AMfirRe1, Q_RXsig, Q_RXsig0, BLOCK_SAMPLES/DOWN_SAMPLE);
        //dsps_fir_f32(&AMfirIm0, I_RXsig, I_RXsig1, BLOCK_SAMPLES/DOWN_SAMPLE);
        //dsps_fir_f32(&AMfirIm1, Q_RXsig, Q_RXsig1, BLOCK_SAMPLES/DOWN_SAMPLE);

        for(int i=0; i<BLOCK_SAMPLES/DOWN_SAMPLE; i++){
          Re=I_RXsig0[i]*G_adj;
          Im=Q_RXsig0[i]*G_adj;
          float Demod_AM = sqrt(Re*Re+Im*Im); 
          zDC = Demod_AM*0.01 + zDC*0.99; //LPF for getting DC component
          RXsig[i] = Demod_AM - zDC;
          //for S-Meter
          Level[pt_Lvl] = zDC*zDC;
          pt_Lvl++; if(pt_Lvl == Ring_Buf_Size) pt_Lvl = 0;
          //Limitter(AGC)
          RXsig[i] *=Gain_AM;
          float absl = fabs(zDC*Gain_AM)-lim_lvl;
          if( absl<0 ) absl = 0;

          if(MUTE_Timer == 0)
          {
            if(absl>g_lim)
              g_lim = g_lim*T_At_AGC + absl*(1.0f-T_At_AGC);
            else
              g_lim*=T_normAGC;
          }
          else
          {
            g_lim = 0;
          }
          RXsig[i] = RXsig[i] * (lim_lvl/(Comp_rate * g_lim+lim_lvl) );
        }
      }

     // RX FM ---------------------------------
      else if(f_MODE==FM)
      {
        for(int i=0; i<BLOCK_SAMPLES/DOWN_SAMPLE; i++)
        {
          Re = I_RXsig[i]*zRe + Q_RXsig[i]*zIm;
          Im = Q_RXsig[i]*zRe - I_RXsig[i]*zIm;
          zRe = I_RXsig[i];
          zIm = Q_RXsig[i];
          float Demod_FM = atan2(Im, Re) * (float)(fsample/DOWN_SAMPLE); 
          zDC = Demod_FM*0.003 + zDC*0.997; // LPF for getting DC component
          RXsig[i] = 5000.0f*(Demod_FM - zDC); // Reject DC
          //for S-Meter
          Re = I_RXsig[i]*G_adj;
          Im = Q_RXsig[i]*G_adj;
          Level[pt_Lvl] = Re*Re + Im*Im;
          pt_Lvl++; if(pt_Lvl == Ring_Buf_Size) pt_Lvl = 0;
          // Limitter
          RXsig[i] *=Gain_FM;
          float absl = fabs(RXsig[i])-lim_lvl;
          if( absl<0 ) absl = 0; 

          if(MUTE_Timer == 0)
          {
            if(absl>g_lim)
              g_lim = g_lim*T_At_AGC + absl*(1.0f-T_At_AGC);
            else
              g_lim*=0.999;
          }
          else
          {
            g_lim = 0;
          }
          RXsig[i] = RXsig[i] * (lim_lvl/(Comp_rate * g_lim+lim_lvl) );
        }
        //for SQL
        /*dsps_biquad_f32(RXsig,  HPFout, BLOCK_SAMPLES/DOWN_SAMPLE, HPF4_biquad0, z_HPF0);
        dsps_biquad_f32(HPFout, HPFout, BLOCK_SAMPLES/DOWN_SAMPLE, HPF4_biquad1, z_HPF1);
        for(int i=0; i<BLOCK_SAMPLES/DOWN_SAMPLE; i++)
        {
          Re = HPFout[i];
          Noise[pt_Noise] = Re*Re;
          pt_Noise++; if(pt_Noise == Ring_Buf_Size) pt_Noise = 0;
        }*/
      }
      // Demod. output is Rxsig[]


    // Up sampLing (Interpolation)
      for(int i=BLOCK_SAMPLES/DOWN_SAMPLE-1; i>=0;  i--)
      {
        Rch_out[i*DOWN_SAMPLE]=DOWN_SAMPLE*RXsig[i];
        Lch_out[i*DOWN_SAMPLE]=0;
        for(int j=1; j<DOWN_SAMPLE; j++)
        {
          Rch_out[i*DOWN_SAMPLE+j]=0;
          Lch_out[i*DOWN_SAMPLE+j]=0;
        }
      }
    // Anti-aliasing Filter
      dsps_biquad_f32(Rch_out, Rch_out, BLOCK_SAMPLES, RX_biquad0, z_IIR0);
      dsps_biquad_f32(Rch_out, Rch_out, BLOCK_SAMPLES, RX_biquad1, z_IIR1);
      dsps_biquad_f32(Rch_out, Rch_out, BLOCK_SAMPLES, RX_biquad2, z_IIR2);
      dsps_biquad_f32(Rch_out, Rch_out, BLOCK_SAMPLES, RX_biquad3, z_IIR3);
//Output to I2S codec
   
    j=0;
    float m = 1.0f;
    for (int i=0; i<BLOCK_SAMPLES; i++) {
      zRX=Rch_out[i]*m*0.05f + zRX*0.95f; //LPF for AF output
      //zRX=Lch_out[i]*m*0.035f + zRX*0.965f; //LPF for AF output
      //zRX=Lch_out[i];
      txbuf[j] = ((int) zRX)>>22;  //filtriran
      txbuf[j+1] = ((int) Rch_out[i])>>22; //nefiltriran
      //Aubuf[pt_Au]=txbuf[j];
      //if (index1 > BLOCK_SAMPLES) index1 = 0;
      j+=2;
      
    }
    //Fill audio buffer
    for (int i=0; i<BLOCK_SAMPLES; i++)
      {
        Aubuf[pt_Au]=txbuf[i*2];
        pt_Au++;
        //if( pt_Au == Ring_Buf_Size ) pt_Au=0;
        if( pt_Au >= BLOCK_SAMPLES*2){
        pt_Au=0;
        index1 = 0;
        } 
      }
    filled = 1;
    //i2s_write( I2S_NUM_1, &txbuf[0], BLOCK_SAMPLES*2*4, &readsize, portMAX_DELAY);
    
  
  digitalWrite(16, LOW);
  }
}  // end of main loop

#define interruptPin 4
//int uout;
//uint64_t number_of_microseconds = 32; //delay v mikrosekundah
// Timer Handle for the high-frequency task
hw_timer_t *Timer0_Cfg = NULL;

// Queue Handle for ISR-to-task communication
//QueueHandle_t isrQueue;

// Interrupt Service Routine (Runs on Core 0)?
void IRAM_ATTR Timer0_ISR() {
    //static uint32_t isrCount = 0;
    //isrCount++;
    /*digitalWrite(interruptPin, HIGH);
    if(filled){
      index1 = 0;
      filled = 0;
    }*/
    // Toggle GPIO pin
    //digitalWrite(interruptPin, !digitalRead(interruptPin));
    uout = 128 + (Aubuf[index1]); // 
    //uout = index1; // sixthth sample back to sample fsample/4
    //sensVal = constrain(sensVal, 10, 150);  // limits range of sensor values to between 10 and 150
    uout = constrain (uout,0,255);
    dac_output_voltage(DAC_CHANNEL_2, uout);
    //corenr = xPortGetCoreID();
    //index1 += 6*2;
    index1 = (index1 + 1);  //3 for 16 khz audio 2 for 24k audio
    //index1 = (index1 + 2);  //2 za 24 khz audio
    if(index1 >= BLOCK_SAMPLES*2) index1 = 0;
    
    //digitalWrite(interruptPin, LOW);
}
void alt_task(void *args)
{ 
  digitalWrite(interruptPin, LOW);
  pinMode(interruptPin, OUTPUT);
  digitalWrite(interruptPin, LOW);
  
  // Configure Timer0 Interrupt
  //Timer0_Cfg = timerBegin(0, 80, true);  // 80 je 1Mhz 
  //Timer0_Cfg = timerBegin(0, 20, true);  // 80 je 1Mhz  20 je 4Mhz je točno za 16kHz audio
  Timer0_Cfg = timerBegin(0, 33, true);  // 80 je 1Mhz  40 je 2Mhz je dobro za 24kHz audio
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  //timerAlarmWrite(Timer0_Cfg, 41, true);  //dobro za za 24khz audio
  //timerAlarmWrite(Timer0_Cfg, 125, true);  //25 za 120kHz sdr  125 za 48khz 62,5 x 2 za 48k
  //timerAlarmWrite(Timer0_Cfg, 250, true);  //je točno za 16kHz
  timerAlarmWrite(Timer0_Cfg, 101, true);  //dobro za za 24khz audio
  timerAlarmEnable(Timer0_Cfg); // Enable the alarm

    // Force ISR to run on Core 0 (optional; ESP32 does this by default)
   /* intr_handle_t timerInterruptHandle;
    esp_intr_alloc(ESP_INTR_FLAG_IRAM, NULL, onTimer1, NULL, &timerInterruptHandle);
    esp_intr_set_affinity(timerInterruptHandle, 0); // Set interrupt affinity to Core 0*/

  //esp_task_wdt_add(nullptr);
  delay(1000);
  
  dac_output_enable(DAC_CHANNEL_2);  //pin 26
  //dac_output_voltage(DAC_CHANNEL_1, 128);
  dac_output_voltage(DAC_CHANNEL_2, 128);
    delay(100);
  while(1)
  {  
     
    //vTaskSuspend(&H_Task[0]);
    //vTaskSuspend( xHandle );
    delay(10);
    //esp_task_wdt_reset();
    /*if (0 != number_of_microseconds)
    {
        while (((uint64_t) esp_timer_get_time() - microseconds) <=
               number_of_microseconds)
        {
            // Wait
        }
    }*/
    //yield();
  }
}




/*----------------------------------------------------------------------
   Serial Com
------------------------------------------------------------------------*/
#define com_len 128
unsigned char com_buf[com_len];
int cp = 0;
int everg = 0;
void SerialCom(void *args)
{ 
  //esp_task_wdt_add(nullptr);
  Serial.begin(115200);
  delay(50);

  while (1)
  {  
    //esp_task_wdt_reset();
    //vTaskResume(&H_Task[0]);
    while (Serial.available()) {
      char tmp[4];
      char c = Serial.read();  // get a character
      // echo back
      if (c == 0x08)  //BS
      {
        tmp[0] = c;
        tmp[1] = 0x20;
        tmp[2] = 0x08;
        tmp[3] = '\0';
        Serial.print(tmp);
      } else {
        tmp[0] = c;
        tmp[1] = '\0';
        Serial.print(tmp);
      }

      //
      if (c == 0x08)  //BS
      {
        if (cp > 0) cp--;
        com_buf[cp] = '\0';
      }
      else if ((c == 0x0d) || (c == 0x0a))
      {
        com_buf[cp] = '\0';
        exe_com(com_buf);
        cp = 0;
      } else {
        com_buf[cp] = c;
        cp++;
        if (cp >= com_len) cp = 0;
      }
    }


    //Serial.println (uout);
    //Serial.println (corenr);
    delay(10);   
  }
}
