/* 
    With reference to:JF3HZB / T.UEBO
    Modified by S52UV / S.Markic
    Created on Dec, 2024
*/
#ifndef FUNCTIONS_H
#define	FUNCTIONS_H

#include <esp_dsp.h>
#include "prm.h"

#define lim_lvl 0.3e9 //0.25e9
#define Th_AGC 0.536871e9
#define Mag_max 2.14758e9
#define S1_Level (142.0f + S1_Level_adj)

#define USB 0
#define LSB 1
#define AM 2
#define FM 3
#define CWU 4
#define CWL 5

#define fsample 24000
#define DOWN_SAMPLE 4 //6
#define BLOCK_SAMPLES (DOWN_SAMPLE*16)  //16 or 32?

#define BW 8000.0f
#define G_fm 0.4f

#define NfftMAX 4096
#define Nfft 1024


void calc_CMPLX_coeff(float *Re, float *Im, int N, float fL, float fH, float nL, float nH, bool Notch);


#endif	/* FUNCTIONS_H */