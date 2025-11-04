// Layout.h
#ifndef LAYOUT_H
#define LAYOUT_H

#include "Config.h"
#include "DisplayUI.h"

// Ajusta posiciones sin recrear el sprite del waterfall
inline void layout_setWaterfallTop(int /*requested_top*/)
{
  // 1) pegamos el WF al fondo usando la altura fija del sprite
  int scr_h = tft.height();              // 320 en tu ILI9488
  waterfall_top = scr_h - WATERFALL_HEIGHT;   // p.ej. 320 - 80 = 240

  // 2) calculamos dónde termina la FFT
  int fft_bottom = waterfall_top - SCALEBAR_H - FFT_WF_GAP;

  // altura mínima de la zona HUD (topbar + banda)
  int hud_top = TOPBAR_H + BANDBAR_H;    // en tu código eran 52 + 14 = 66

  // altura deseada de FFT
  int fft_h = FFT_FIXED_H;               // el que ya usabas (100 aprox)

  // si no cabe la FFT fija, la recortamos
  if (fft_bottom - fft_h < hud_top) {
    // ui_max está en DisplayUI.h en tus fuentes
    fft_h = ui_max(10, fft_bottom - hud_top);
  }

  // 3) limpiar zona FFT
  if (fft_bottom > hud_top) {
    tft.fillRect(
      0,
      hud_top,
      tft.width(),
      fft_bottom - hud_top,
      TFT_BLACK
    );
  }

  // 4) actualizar parámetros de UI
  ui_setFFTBottom(fft_bottom);
  ui_setFFTHeight(fft_h);
  ui_setResetFFT(true);

  // 5) redibujar estáticos que dependen de la posición
  ui_drawTopStaticTicks();
  ui_drawBandBar();
  ui_drawMidFreqScale(fft_bottom);

  // 6) empujar el sprite de WF en su nuevo top (pegado abajo)
  wf.pushSprite(0, waterfall_top);
}

#endif
