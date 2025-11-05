// Globals.h
#ifndef GLOBALS_H
#define GLOBALS_H

#include "Config.h"
extern "C" {
  #include "esp_log.h"
}

#include <TFT_eSPI.h>
#include <si5351.h>

// ================================
// Instancias globales de HW
// ================================
TFT_eSPI   tft  = TFT_eSPI();
TFT_eSprite wf  = TFT_eSprite(&tft);
Si5351     si5351;

// ================================
// SDR / LO / muestreo
// ================================
// LO por defecto
uint32_t g_loHz       = 7050000UL;

// Tabla de frecuencias de muestreo que usa el encoder
// (ajústalas si quieres otras)
const uint32_t g_samplingTable[3] = {
  24000U,   // 0 → 48 kHz (la del DAC fijo)
  48000U,   // 1 → 80 kHz (la que has usado en casi todos los sketches)
  96000U    // 2 → 96 kHz
};

// índice actual dentro de la tabla
int g_samplingIndex = 1;          // ← arrancamos en 80 kHz

// valor efectivo que usan I2S / UI (se sincroniza en setup)
uint32_t g_samplingHz = g_samplingTable[g_samplingIndex];

// ==== Auto-REF para FFT/WF ====
#define FFT_AUTOREF_ENABLED   0     // pon 0 si no quieres que actúe
#define FFT_AUTO_ATTACK_PCT   25    // sube 25% hacia arriba cuando hay pico
#define FFT_AUTO_DECAY_PCT    2     // baja 2% cuando está muy alto

// valor mínimo para no irnos a cero
static double g_refMagMin = 50000;   // ajusta según tu escala


// referencia para escalar la FFT
uint32_t g_refMag     = 200000UL;

// ================================
// Buffers de FFT compartidos
// ================================
double vReal[NUM_SAMPLES];
double vImag[NUM_SAMPLES];

// magnitudes reescaladas a DRAW_WIDTH que consume la UI
double g_magDisp[DRAW_WIDTH];
inline void fft_autoRef()
{
#if FFT_AUTOREF_ENABLED
  // 1. buscamos el pico del frame actual
  double peak = 0.0;
  for (int i = 0; i < DRAW_WIDTH; i++) {
    if (g_magDisp[i] > peak)
      peak = g_magDisp[i];
  }

  if (peak < g_refMagMin)
    peak = g_refMagMin;

  // 2. si el pico es MAYOR que el ref actual → subir rápido (ataque)
  if (peak > g_refMag) {
    // g_refMag = g_refMag + (peak - g_refMag) * ataque
    g_refMag = g_refMag + (peak - g_refMag) * (FFT_AUTO_ATTACK_PCT / 100.0);
  }
  else {
    // 3. si el pico es MENOR → bajar lento (decay)
    g_refMag = g_refMag - g_refMag * (FFT_AUTO_DECAY_PCT / 100.0);
    if (g_refMag < g_refMagMin)
      g_refMag = g_refMagMin;
  }
#endif
}

// ================================
// Corrección IQ (Storage, Encoder, UI)
// ================================
// ================================
// Modo de demodulación (UI)
// ================================
enum DemodMode : uint8_t {
  DEMOD_AM = 0,
  DEMOD_LSB,
  DEMOD_USB,
  DEMOD_DSB,
  DEMOD_CW,
  DEMOD_COUNT
};

uint8_t g_demodMode = DEMOD_AM;

static const char* const DEMOD_NAMES[DEMOD_COUNT] = {
  "AM",
  "LSB",
  "USB",
  "DSB"
};

double g_iqGain  = 1.0;
double g_iqPhase = 0.0;

// ================================
// Encoder / paso
// ================================
int g_encMode = ENC_MODE_NORMAL;     // viene de Config.h
int g_stepIdx = 2;                   // 0=10Hz,1=100Hz,2=1k,3=10k
const uint32_t STEP_TABLE[] = {10, 100, 1000, 10000};

// ================================
// Bandas
// ================================
struct BandDef {
  const char *name;
  uint32_t    loHz;
};

BandDef g_bands[] = {
  // --- Radioaficionado ---

  { "60m HAM",5357000UL },   // canalizada, ponemos centro típico
  { "40m",    7050000UL },   // ya la tenías
  { "30m",   10120000UL },
  { "20m",   14100000UL },
  { "17m",   18130000UL },
  { "15m",   21100000UL },   // ya la tenías
  { "12m",   24940000UL },
  { "10m",   28400000UL },
  { "CB",    27100000UL },   // CB/11m (27.1 MHz centro)

  // --- Broadcast / SW (ondas cortas) ---
  // valores típicos ITU redondeados al centro del segmento

  { "49m SW",  6050000UL },  // 5.9 - 6.2 MHz
  { "41m SW",  7200000UL },  // 7.1 - 7.35 MHz (ojo: se pisa un poco con 40m, normal)
  { "31m SW",  9650000UL },  // 9.4 - 9.9 MHz
  { "25m SW", 11600000UL },  // 11.6 - 12.1 MHz
  { "22m SW", 13600000UL },  // 13.57 - 13.87 MHz
  { "19m SW", 15100000UL },  // 15.1 - 15.8 MHz
  { "16m SW", 17600000UL },  // 17.48 - 17.9 MHz
  { "15m SW", 18900000UL },  // 18.9 - 19.02 MHz (menos usada pero la pongo)
  { "13m SW", 21500000UL },  // 21.45 - 21.85 MHz
  { "11m SW", 25600000UL },  // 25.6 - 26.1 MHz (a veces hay BC aquí)
};

const int g_bandCount = sizeof(g_bands) / sizeof(g_bands[0]);
int       g_bandIndex = 0;

// ================================
// Waterfall
// ================================
int waterfall_top = 0;

// ================================
// Si5351 apply diferido
// ================================
uint32_t     g_lastApplyMs    = 0;
const uint32_t g_applyDelayMs = 35;

// ================================
// Multicore / sync
//  - audio (core 1) escribe IQ
//  - UI    (core 0) procesa FFT y dibuja
// ================================
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

portMUX_TYPE g_sdrMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool g_iq_frame_ready = false;

// ================================
// Util
// ================================
inline void sdr_silence_logs() {
  esp_log_level_set("*", ESP_LOG_NONE);
}

#endif // GLOBALS_H
