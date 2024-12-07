/* 
    With reference to:JF3HZB / T.UEBO
    Modified by S52UV / S.Markic
    Created on Dec, 2024
*/

#include "functions.hpp"



extern float *wf_HANN;
extern float *wf_HAMMING;
extern float *wf_BLACKMAN;
extern float *wf_BLACKMANHARRIS;
extern float *dat;
extern float *ejwt;
extern float *inv_ejwt;
extern float *tmpRe;
extern float *tmpIm;
extern int f_MODE;
void calc_CMPLX_coeff(float *coeffRe, float *coeffIm, int N, float fL, float fH, float nL, float nH, bool Notch)
{
  float fs = fsample/DOWN_SAMPLE;
  int kL, kH;
  float g=1.0f/(float)N;
  float fnyq = 0.5*fs;
  float inv_df= (float)N/fs;
  float df= fs/(float)N;

  if(f_MODE == USB || f_MODE == LSB || f_MODE == AM)
  {
    kL = (int)( (fL + fnyq)*inv_df );
    kH = (int)( (fH + fnyq)*inv_df );
    for(int i=0; i<N; i++) tmpRe[i]=0;

    for(int i=kL+1; i<=kH; i++) tmpRe[i]=1;
    tmpRe[kL]= tmpRe[kL+1] * ( 1.0f-( (fL + fnyq)*inv_df - (float)kL ) );
    tmpRe[kH+1]= tmpRe[kH] * ( (fH + fnyq)*inv_df - (float)kH );
  }
  else if(f_MODE == CWU || f_MODE == CWL )
  {
    
  }


  if(Notch == true)
  {
    kL = (int)( (nL + fnyq)*inv_df );
    kH = (int)( (nH + fnyq)*inv_df );
    for(int i=0; i<N; i++) tmpIm[i]=0;

    for(int i=kL+1; i<=kH; i++) tmpIm[i]=1;
    tmpIm[kL]= tmpIm[kL+1] * ( 1.0f-( (nL + fnyq)*inv_df - (float)kL ) );
    tmpIm[kH+1]= tmpIm[kH] * ( (nH + fnyq)*inv_df - (float)kH );

    for (int i = 0; i < N; i++)
    {
      tmpRe[i] -= tmpIm[i];
      if (tmpRe[i] < 0) tmpRe[i] = 0;
    }
  }

  for(int i=0; i<N; i++) tmpIm[i]=0;

  for(int i=0; i<(N>>1); i++)
  {
    dat[2*i]  = tmpRe[(N>>1) + i];
    dat[2*i+1]= tmpIm[(N>>1) + i];
    dat[2*( (N>>1) + i )    ] = tmpRe[i];
    dat[2*( (N>>1) + i ) + 1] = tmpIm[i];
  }

  dsps_fft2r_fc32_ae32_(dat, N, inv_ejwt);
  dsps_bit_rev2r_fc32(dat, N);

  for(int i=0; i<(N>>1); i++)
  {
    tmpRe[(N>>1) + i] = g*dat[2*i];
    tmpIm[(N>>1) + i] = g*dat[2*i+1];
    tmpRe[i] = g*dat[2*( (N>>1) + i )   ];
    tmpIm[i] = g*dat[2*( (N>>1) + i ) +1];
  }

  for(int i=0; i<N; i++)
  {
    //coeffRe[i] = tmpRe[i] * wf_BLACKMANHARRIS[i*(NfftMAX/N)];
    //coeffIm[i] = tmpIm[i] * wf_BLACKMANHARRIS[i*(NfftMAX/N)];
    coeffRe[i] = tmpRe[i] * wf_HAMMING[i*(NfftMAX/N)];
    coeffIm[i] = tmpIm[i] * wf_HAMMING[i*(NfftMAX/N)];
  }

}

