// FFT_IQ.h
#ifndef FFT_IQ_H
#define FFT_IQ_H

#include "Config.h"
#include <math.h>

// umbral: si la energía de Q es < 1% de la de I, lo consideramos "real"
#define IQ_REAL_LIKE_THRESHOLD 0.01

// flag para saber si esta FFT salió "casi real"
static bool g_fft_real_like = false;

// ======================================
// 1) Ventana a los buffers IQ
// ======================================
inline void fft_applyWindowIQ()
{
  static bool   init = false;
  static double w[NUM_SAMPLES];

  if (!init) {
    for (int n = 0; n < NUM_SAMPLES; n++) {
      w[n] = 0.54 - 0.46 * cos((2.0 * M_PI * n) / (NUM_SAMPLES - 1)); // Hamming
    }
    init = true;
  }

  for (int n = 0; n < NUM_SAMPLES; n++) {
    vReal[n] *= w[n];
    vImag[n] *= w[n];
  }
}

// ======================================
// 2) FFT compleja radix-2 in-place
//    (la misma que ya traías)
// ======================================
inline void fft_runComplex()
{
  int N = NUM_SAMPLES;
  int j = 0;

  // bit-reversal
  for (int i = 1; i < N - 1; i++) {
    int bit = N >> 1;
    for ( ; j & bit; bit >>= 1) j &= ~bit;
    j |= bit;
    if (i < j) {
      double tr = vReal[i]; vReal[i] = vReal[j]; vReal[j] = tr;
      double ti = vImag[i]; vImag[i] = vImag[j]; vImag[j] = ti;
    }
  }

  // etapas
  for (int len = 2; len <= N; len <<= 1) {
    double ang = -2.0 * M_PI / (double)len;
    double wlen_r = cos(ang);
    double wlen_i = sin(ang);
    for (int i = 0; i < N; i += len) {
      double w_r = 1.0;
      double w_i = 0.0;
      for (int j = 0; j < len/2; j++) {
        int u = i + j;
        int v = i + j + len/2;
        double vr = vReal[v] * w_r - vImag[v] * w_i;
        double vi = vReal[v] * w_i + vImag[v] * w_r;
        vReal[v]  = vReal[u] - vr;
        vImag[v]  = vImag[u] - vi;
        vReal[u] += vr;
        vImag[u] += vi;
        double next_w_r = w_r * wlen_r - w_i * wlen_i;
        double next_w_i = w_r * wlen_i + w_i * wlen_r;
        w_r = next_w_r;
        w_i = next_w_i;
      }
    }
  }

  // detección de "casi real": energía de Q muy baja
  double eI = 0.0, eQ = 0.0;
  for (int k = 0; k < N; k++) {
    eI += vReal[k] * vReal[k];
    eQ += vImag[k] * vImag[k];
  }
  g_fft_real_like = (eQ < IQ_REAL_LIKE_THRESHOLD * eI);
}

// ======================================
// 3) Empaquetar a pantalla: DC al centro
// ======================================
inline void fft_packToDisplay()
{
  static double magShift[NUM_SAMPLES];

  // 1) magnitud y shift (-Fs/2 .. +Fs/2)
  for (int k = 0; k < NUM_SAMPLES; ++k) {
    double re = vReal[k], im = vImag[k];
    double mag = sqrt(re * re + im * im);
    int s = (k + NUM_SAMPLES/2) % NUM_SAMPLES;
    magShift[s] = mag;
  }

  // 2) si la FFT parecía "real", anulamos la mitad espejo
  if (g_fft_real_like) {
    for (int k = NUM_SAMPLES/2 + 1; k < NUM_SAMPLES; k++) {
      magShift[k] = 0.0;
    }
  }

  // 3) reescalar a DRAW_WIDTH
  const double scale = (double)NUM_SAMPLES / (double)DRAW_WIDTH;
  for (int x = 0; x < DRAW_WIDTH; ++x) {
    double src = x * scale;
    int k0 = (int)floor(src);
    int k1 = (k0 + 1 < NUM_SAMPLES) ? (k0 + 1) : (NUM_SAMPLES - 1);
    double frac = src - (double)k0;
    g_magDisp[x] = magShift[k0] + frac * (magShift[k1] - magShift[k0]);
  }
}

#endif
