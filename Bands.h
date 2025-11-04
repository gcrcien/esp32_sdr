// Bands.h
#ifndef BANDS_H
#define BANDS_H

#include <Arduino.h>
#include "Globals.h"   // aquí ya vienen: BandDef, g_bands[], g_bandCount, g_bandIndex, g_loHz

// Selecciona la banda cuyo LO esté más cerca de "loHz"
// Esto es lo que tu Encoder.h llama en:
//   bands_select_from_lo(g_loHz);
inline void bands_select_from_lo(uint32_t loHz)
{
  int bestIdx = 0;
  uint32_t bestDiff = 0xFFFFFFFFUL;

  for (int i = 0; i < g_bandCount; i++) {
    uint32_t b = g_bands[i].loHz;
    uint32_t diff = (loHz > b) ? (loHz - b) : (b - loHz);
    if (diff < bestDiff) {
      bestDiff = diff;
      bestIdx = i;
    }
  }

  g_bandIndex = bestIdx;
}

// Avanza a la siguiente banda del arreglo global
// Esto es lo que tu Encoder.h llama en:
//   bands_next_band();
inline void bands_next_band()
{
  g_bandIndex++;
  if (g_bandIndex >= g_bandCount)
    g_bandIndex = 0;

  // al cambiar de banda, mueve el LO al centro de esa banda
  g_loHz = g_bands[g_bandIndex].loHz;
}

// (opcional) banda anterior, por si luego la quieres desde un botón
inline void bands_prev_band()
{
  if (g_bandIndex == 0) g_bandIndex = g_bandCount - 1;
  else g_bandIndex--;

  g_loHz = g_bands[g_bandIndex].loHz;
}

#endif // BANDS_H
