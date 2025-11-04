// DisplayUI.h
#ifndef DISPLAY_UI_H
#define DISPLAY_UI_H

#include "Config.h"
#include "Bands.h"

// ====== COLORES CONFIGURABLES PARA FFT ======
#ifndef FFT_BG_COLOR
#define FFT_BG_COLOR   (0, 0, 8)     // fondo de la zona FFT
#endif

#ifndef FFT_BAR_COLOR
#define FFT_BAR_COLOR  0x3646
#endif

// ====== primero: defines que usa Layout.h ======
#ifndef TOPBAR_LINE_Y
#define TOPBAR_LINE_Y   (TOPBAR_H + 4)
#endif

#ifndef BANDBAR_Y
#define BANDBAR_Y       (TOPBAR_LINE_Y + 1)
#endif
// Suavizado del waterfall
#define WF_SMOOTH_NEW   2   // peso del valor nuevo
#define WF_SMOOTH_OLD   1   // peso del valor anterior


// -------- helpers locales (Layout.h también los usa) --------
static inline int ui_min(int a, int b) {
  return (a < b) ? a : b;
}
static inline int ui_max(int a, int b) {
  return (a > b) ? a : b;
}

// convierte magnitud lineal -> [0..1] usando top + span
static inline float ui_mag_to_norm_log(double mag)
{
  float ref = (float)g_refMag;
  if (ref <= 0.0f) return 0.0f;

  float ratio = (float)mag / ref;
  if (ratio < 1e-8f) ratio = 1e-8f;

  float dB = 20.0f * log10f(ratio);  // dB relativos a REF (<= UI_DB_TOP normalmente)

  const float top   = UI_DB_TOP;            // ej. 0 dB
  const float floor = top - UI_DB_SPAN;     // ej. -50 dB

  if (dB > top)   dB = top;
  if (dB < floor) dB = floor;

  // mapear [floor, top] -> [0,1]
  return (dB - floor) / (top - floor);
}


// ====== forward declarations para que Layout.h no truene ======
inline void ui_setFFTBottom(int y);
inline void ui_setFFTHeight(int h);
inline void ui_setResetFFT(bool v);
inline void ui_drawTopStaticTicks();
inline void ui_drawTopDynamic();
inline void ui_drawBandBar();
inline void ui_drawMidFreqScale(int fft_y_bottom);

// ⬇️ AHORA sí podemos incluir el layout
#include "Layout.h"

// -------- estado interno de UI --------
static bool  g_wfNewestAtTop   = true;
static int   g_fft_y_bottom    = 0;
static int   g_fft_height      = FFT_FIXED_H;
static bool  g_resetFFTHeights = false;

// este es el nuevo flag: pinta el rectángulo de fondo de la FFT solo una vez
static bool  g_fftBgDrawn      = false;

// LUT para gamma del waterfall
static uint8_t g_wfGammaLUT[256];
static bool    g_wfGammaInit = false;
static uint8_t g_wfPrevLine[WATERFALL_WIDTH] = {0};


// === 7-seg dibujada por software (mini) =====================
inline void ui_draw7seg_digit(int x, int y, int s, int d, uint16_t col, uint16_t bg)
{
  int th = s;        // thickness
  int len = 4 * s;   // largo horizontal
  int hgt = 4 * s;   // alto vertical

  static const uint8_t seg[10] = {
    0b1111110, // 0
    0b0110000, // 1
    0b1101101, // 2
    0b1111001, // 3
    0b0110011, // 4
    0b1011011, // 5
    0b1011111, // 6
    0b1110000, // 7
    0b1111111, // 8
    0b1111011  // 9
  };

  if (d < 0 || d > 9) return;
  uint8_t mask = seg[d];

  auto fill = [&](int xx, int yy, int w, int h, bool on) {
    tft.fillRect(xx, yy, w, h, on ? col : bg);
  };

  // a
  fill(x + th, y, len, th, mask & 0b1000000);
  // b
  fill(x + th + len, y + th, th, hgt, mask & 0b0100000);
  // c
  fill(x + th + len, y + th + hgt + th, th, hgt, mask & 0b0010000);
  // d
  fill(x + th, y + th + hgt + th + hgt, len, th, mask & 0b0001000);
  // e
  fill(x, y + th + hgt + th, th, hgt, mask & 0b0000100);
  // f
  fill(x, y + th, th, hgt, mask & 0b0000010);
  // g
  fill(x + th, y + th + hgt, len, th, mask & 0b0000001);
}

inline void ui_draw7seg_3digits(int x, int y, int s, int v, uint16_t col, uint16_t bg)
{
  int d0 = v % 10;
  int d1 = (v / 10) % 10;
  int d2 = (v / 100) % 10;
  int adv = (4 * s) + s + 2;  // ancho aprox

  ui_draw7seg_digit(x,           y, s, d2, col, bg);
  ui_draw7seg_digit(x + adv,     y, s, d1, col, bg);
  ui_draw7seg_digit(x + adv * 2, y, s, d0, col, bg);
}


// =====================================================
// SELECTOR DE FUENTES
// =====================================================
enum UIFont {
  UI_FONT_DEFAULT = 0,
  UI_FONT_SMALL,
  UI_FONT_MEDIUM,
  UI_FONT_BIG7SEG
};

inline void ui_useFont(UIFont f)
{
  switch (f) {
    case UI_FONT_SMALL:
      tft.setTextFont(2);
      tft.setTextSize(1);
      break;
    case UI_FONT_MEDIUM:
      tft.setTextFont(4);
      tft.setTextSize(1);
      break;
    case UI_FONT_BIG7SEG:
      tft.setTextFont(7);
      tft.setTextSize(1);
      break;
    default:
      tft.setTextFont(2);
      tft.setTextSize(1);
      break;
  }
}

// =====================================================
// INIT
// =====================================================
inline void ui_initDisplay()
{
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
}


// =====================================================
// BARRA SUPERIOR (ticks)
// =====================================================
inline void ui_drawTopStaticTicks()
{
  const int W = ui_min(DRAW_WIDTH, (int)tft.width());
  const double HzPerPix = (double)g_samplingHz / (double)W;

  const int tickTop = TOPBAR_H - 10;
  const int baseY   = TOPBAR_H - 1;

  tft.drawFastHLine(0, baseY, W, TFT_WHITE);

  double targetHz = HzPerPix * 60.0;
  double pow10 = pow(10.0, floor(log10(targetHz)));
  double cand[3] = {1.0, 2.0, 5.0};
  double stepHz = cand[0] * pow10;
  for (int i = 0; i < 3; i++) {
    double s = cand[i] * pow10;
    if (s >= targetHz) {
      stepHz = s;
      break;
    }
  }

  double startHz    = -((double)g_samplingHz) / 2.0;
  double endHz      =  ((double)g_samplingHz) / 2.0;
  double firstTick  = ceil(startHz / stepHz) * stepHz;

  tft.startWrite();
  for (double h = firstTick; h <= endHz + 1.0; h += stepHz) {
    int x = (int)round((h - startHz) / HzPerPix);
    if (x < 0 || x >= W) continue;
    tft.drawFastVLine(x, tickTop, baseY - tickTop, TFT_WHITE);
  }
  int cx = W / 2;
  tft.drawFastVLine(cx, tickTop, baseY - tickTop, TFT_WHITE);
  tft.endWrite();
}


// =====================================================
// FORMATEADORES
// =====================================================
inline void ui_splitFreq(uint32_t hz, uint32_t &mhz, uint32_t &khz, uint32_t &hz3)
{
  mhz = hz / 1000000UL;
  uint32_t rest = hz % 1000000UL;
  khz = rest / 1000UL;
  hz3 = rest % 1000UL;
}

inline void ui_formatHzShort(uint32_t v, char* out, size_t n)
{
  if (v >= 1000000UL) {
    float f = v / 1000000.0f;
    snprintf(out, n, "%.2fM", f);
  }
  else if (v >= 1000UL) {
    float f = v / 1000.0f;
    snprintf(out, n, "%.1fk", f);
  }
  else {
    snprintf(out, n, "%lu", (unsigned long)v);
  }
}


// =====================================================
// BARRA DE BANDA (ahora sí usando BANDBAR_Y)
// =====================================================
inline void ui_drawBandBar()
{
  tft.fillRect(0, BANDBAR_Y, tft.width(), BANDBAR_H, TFT_BLACK);
  ui_useFont(UI_FONT_SMALL);
  tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
  tft.setCursor(4, BANDBAR_Y + BANDBAR_H - 2);
  tft.print("BANDA: ");
  tft.print(g_bands[g_bandIndex].name);
  ui_useFont(UI_FONT_DEFAULT);
  // reforzar línea de topbar
  tft.drawFastHLine(0, TOPBAR_LINE_Y, tft.width(), TFT_WHITE);
}


// =====================================================
// HUD DINÁMICO
// =====================================================
inline void ui_drawTopDynamic()
{
  static uint32_t prev_lo      = 0;
  static uint32_t prev_ref     = 0;
  static int      prev_stepIdx = -1;
  static uint8_t  prev_encMode = 255;
  static double   prev_iqGain  = 999;
  static double   prev_iqPhase = 999;
  static uint32_t prev_fs      = 0;
  static uint8_t  prev_demod   = 255;

  extern const uint32_t STEP_TABLE[];
  extern int   g_stepIdx;

  bool need = false;
  if (g_loHz       != prev_lo)      need = true;
  if (g_refMag     != prev_ref)     need = true;
  if (g_stepIdx    != prev_stepIdx) need = true;
  if (g_encMode    != prev_encMode) need = true;
  if (fabs(g_iqGain  - prev_iqGain)  > 0.005) need = true;
  if (fabs(g_iqPhase - prev_iqPhase) > 0.005) need = true;
  if (g_samplingHz  != prev_fs)     need = true;
  // 🔴 ESTA FALTABA:
  if (g_demodMode   != prev_demod)  need = true;

  if (!need) {
    return;
  }

  const int freqX = 4,   freqY = 2,  freqW = 240, freqH = 48;
  const int refX  = 250, refY  = 10, refW  = 70,  refH  = 16;
  const int stepX = 330, stepY = 10, stepW = 55,  stepH = 16;
  const int iqX   = 400, iqY   = 10, iqW   = 75,  iqH   = 16;

  // FRECUENCIA
  tft.fillRect(freqX, freqY, freqW, freqH, TFT_BLACK);

  uint32_t mhz, khz, hz3;
  ui_splitFreq(g_loHz, mhz, khz, hz3);

  char bigPart[16];
  snprintf(bigPart, sizeof(bigPart), "%lu.%03lu", (unsigned long)mhz, (unsigned long)khz);

  ui_useFont(UI_FONT_BIG7SEG);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(freqX, freqY + 2);
  tft.print(bigPart);

  int xSmall = freqX + tft.textWidth(bigPart, 7) + 4;
  int miniScale = 2;
  ui_draw7seg_3digits(xSmall, freqY + 10, miniScale, hz3, TFT_GREEN, TFT_BLACK);

  // REF
  tft.fillRect(refX, refY, refW, refH, TFT_BLACK);
  ui_useFont(UI_FONT_SMALL);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(refX, refY + 12);
  tft.print("R=");
  char refStr[16];
  ui_formatHzShort((uint32_t)g_refMag, refStr, sizeof(refStr));
  tft.print(refStr);

  // STEP
  char stepStr[24];
  ui_formatHzShort(STEP_TABLE[g_stepIdx], stepStr, sizeof(stepStr));
  tft.fillRect(stepX, stepY, stepW, stepH, TFT_BLACK);
  ui_useFont(UI_FONT_SMALL);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(stepX, stepY + 12);
  tft.print("S=");
  tft.print(stepStr);

  // MODO (donde antes ponías IQ/FS)
  tft.fillRect(iqX, iqY, iqW, iqH, TFT_BLACK);
  ui_useFont(UI_FONT_SMALL);
  tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
  tft.setCursor(iqX, iqY + 12);
  tft.print(DEMOD_NAMES[g_demodMode]);

  prev_lo      = g_loHz;
  prev_ref     = g_refMag;
  prev_stepIdx = g_stepIdx;
  prev_encMode = g_encMode;
  prev_iqGain  = g_iqGain;
  prev_iqPhase = g_iqPhase;
  prev_fs      = g_samplingHz;
  prev_demod   = g_demodMode;

  tft.drawFastHLine(0, TOPBAR_LINE_Y, tft.width(), TFT_WHITE);
}



// =====================================================
// INIT WATERFALL
// =====================================================
inline void ui_initWaterfall()
{
  wf.setColorDepth(16);
  wf.createSprite(WATERFALL_WIDTH, WATERFALL_HEIGHT);
  tft.setSwapBytes(true);
  wf.fillSprite(TFT_BLACK);

  // reset del suavizado
  memset(g_wfPrevLine, 0, sizeof(g_wfPrevLine));

  if (!g_wfGammaInit) {
    for (int i = 0; i < 256; i++) {
      float nv = i / 255.0f;
      float pg = powf(nv, 0.70f);
      g_wfGammaLUT[i] = (uint8_t)(pg * 255.0f);
    }
    g_wfGammaInit = true;
  }
}


// =====================================================
// ESCALA INTERMEDIA
// =====================================================
inline void ui_drawMidFreqScale(int fft_y_bottom)
{
  int y_top = fft_y_bottom + 1;
  tft.fillRect(0, y_top, tft.width(), SCALEBAR_H, TFT_BLACK);

  tft.drawFastHLine(0, y_top, tft.width(), TFT_WHITE);

  const float fs     = (float)g_samplingHz;
  const float halfFs = fs * 0.5f;
  const int   divisionsPerSide = 10;
  const float stepHz = halfFs / (float)divisionsPerSide;

  const int W = ui_min(DRAW_WIDTH, (int)tft.width());

  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);

  for (int i = -divisionsPerSide; i <= divisionsPerSide; i++) {
    float f = (float)i * stepHz;
    float norm = (f + halfFs) / fs;
    int   x = (int)roundf(norm * (float)(W - 1));

    int tickH = (i == 0) ? 6 : 4;
    tft.drawFastVLine(x, y_top, tickH, TFT_WHITE);

    if ((i % 2) == 0) {
      char buf[16];
      float k = f / 1000.0f;
      if (fabs(k) < 1.0f) {
        snprintf(buf, sizeof(buf), "0");
      } else {
        snprintf(buf, sizeof(buf), "%.0fk", k);
      }
      int tx = x - (strlen(buf) * 6) / 2;
      if (tx < 0) tx = 0;
      if (tx > W - 24) tx = W - 24;
      tft.setCursor(tx, y_top + 6);
      tft.print(buf);
    }
  }

  ui_useFont(UI_FONT_DEFAULT);
  tft.drawFastHLine(0, TOPBAR_LINE_Y, tft.width(), TFT_WHITE);
}


// =====================================================
// MAPA COLOR
// =====================================================
inline uint16_t ui_intensityToColor(uint8_t intensity)
{
  uint8_t r = 0, g = 0, b = 0;
  if (intensity < 64) {
    b = map(intensity, 0, 63, 0, 255);
  } else if (intensity < 128) {
    g = map(intensity, 64, 127, 0, 255); b = 255;
  } else if (intensity < 192) {
    r = map(intensity, 128, 191, 0, 255); g = 255; b = map(intensity, 128, 191, 255, 0);
  } else {
    r = 255; g = map(intensity, 192, 255, 255, 0);
  }
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


// =====================================================
// FFT LINE (fondo se dibuja una sola vez)
// =====================================================
inline void ui_drawFFTLine()
{
  int x = 0;
  int y = g_fft_y_bottom - g_fft_height;
  int w = 480;           // o DRAW_WIDTH
  int h = g_fft_height;

  const int bins_to_draw = ui_min(DRAW_WIDTH, (int)tft.width());

  // buffers estáticos
  static float    fftSmooth[DRAW_WIDTH] = {0};
  static bool     fftSmoothInit = false;
  const float     alpha = 0.1f;

  static int      prevHeights[DRAW_WIDTH] = {0};
  static uint32_t lastRef = 0;

  // si hubo cambio de ref o pidieron reset, hay que repintar fondo una sola vez
  if (g_resetFFTHeights || lastRef != g_refMag) {
    memset(prevHeights, 0, sizeof(prevHeights));
    memset(fftSmooth, 0, sizeof(fftSmooth));
    fftSmoothInit   = false;
    g_resetFFTHeights = false;
    lastRef         = g_refMag;
    g_fftBgDrawn    = false;   // forzamos a que pinte el rectángulo en este frame
  }

  // pinta el fondo SOLO si no está pintado
  if (!g_fftBgDrawn) {
    tft.fillRect(x, y, w, h, FFT_BG_COLOR);
    g_fftBgDrawn = true;
  }
  static int rawHeights[DRAW_WIDTH];
  for (int i = 0; i < bins_to_draw; i++) {
    // ahora usamos escala log relativa a g_refMag
    float nv = ui_mag_to_norm_log(g_magDisp[i]);
    int barHeight = (int)(nv * g_fft_height);
    if (barHeight < 0) barHeight = 0;
    if (barHeight > g_fft_height) barHeight = g_fft_height;
    rawHeights[i] = barHeight;
  }

  static int smoothSpatial[DRAW_WIDTH];
  for (int i = 0; i < bins_to_draw; i++) {
    int left  = (i > 0) ? rawHeights[i - 1] : rawHeights[i];
    int mid   = rawHeights[i];
    int right = (i < bins_to_draw - 1) ? rawHeights[i + 1] : rawHeights[i];
    smoothSpatial[i] = (left + 2 * mid + right) / 4;
  }

  tft.startWrite();
  for (int i = 0; i < bins_to_draw; i++) {
    int targetH;
    if (!fftSmoothInit) {
      fftSmooth[i] = (float)smoothSpatial[i];
      targetH = smoothSpatial[i];
    } else {
      fftSmooth[i] = fftSmooth[i] + alpha * ((float)smoothSpatial[i] - fftSmooth[i]);
      targetH = (int)(fftSmooth[i] + 0.5f);
    }

    if (targetH < 0) targetH = 0;
    if (targetH > g_fft_height) targetH = g_fft_height;

    int prevHeight = prevHeights[i];

    if (targetH > prevHeight) {
      // dibuja solo la parte nueva en verde
      tft.drawFastVLine(i, g_fft_y_bottom - targetH, targetH - prevHeight, FFT_BAR_COLOR);
    } else if (targetH < prevHeight) {
      // “borra” con el color de fondo de la FFT
      tft.drawFastVLine(i, g_fft_y_bottom - prevHeight, prevHeight - targetH, FFT_BG_COLOR);
    }
    prevHeights[i] = targetH;
  }
  fftSmoothInit = true;

  // línea central
  int cx = bins_to_draw / 2;
  tft.drawFastVLine(cx, g_fft_y_bottom - g_fft_height, g_fft_height, TFT_WHITE);
  tft.endWrite();
}


// =====================================================
// WATERFALL
// =====================================================
inline void ui_drawWaterfallLine()
{
  const int bins_to_draw = ui_min(DRAW_WIDTH, WATERFALL_WIDTH);

  int yNew;
  if (g_wfNewestAtTop) {
    wf.scroll(0, +1);
    yNew = 0;
  }
  else {
    wf.scroll(0, -1);
    yNew = WATERFALL_HEIGHT - 1;
  }

  for (int x = 0; x < bins_to_draw; x++) {
    // valor normalizado logarítmico (0..1)
    float nv = ui_mag_to_norm_log(g_magDisp[x]);

    // pasarlo a 0..255 y aplicar gamma como ya lo hacías
    uint8_t idx      = (uint8_t)(nv * 255.0f);
    uint8_t gammaVal = g_wfGammaLUT[idx];


    // === suavizado temporal muy ligero ===
    // 3 partes del valor nuevo + 1 parte del valor anterior
uint8_t prev    = g_wfPrevLine[x];
uint8_t smooth  = (uint8_t)(
    ((uint16_t)gammaVal * WF_SMOOTH_NEW + (uint16_t)prev * WF_SMOOTH_OLD)
    / (WF_SMOOTH_NEW + WF_SMOOTH_OLD)
);
g_wfPrevLine[x] = smooth;


    uint16_t c       = ui_intensityToColor(smooth);
    wf.drawPixel(x, yNew, c);
  }

  wf.pushSprite(0, waterfall_top);
}


// =====================================================
// setters usados por Layout
// =====================================================
inline void ui_setFFTBottom(int y) {
  g_fft_y_bottom = y;
  // si cambió la posición, hay que repintar el fondo
  g_fftBgDrawn = false;
  g_resetFFTHeights = true;
}
inline void ui_setFFTHeight(int h) {
  g_fft_height   = h;
  // si cambió la altura, hay que repintar el fondo
  g_fftBgDrawn = false;
  g_resetFFTHeights = true;
}
inline void ui_setResetFFT(bool v) {
  g_resetFFTHeights = v;
  if (v) {
    g_fftBgDrawn = false;
  }
}

#endif // DISPLAY_UI_H
