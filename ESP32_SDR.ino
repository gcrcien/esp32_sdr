// ====================== ESP32_SDR.ino ======================
#include <Arduino.h>

#include "Config.h"
#include "Globals.h"
#include "Storage.h"
#include "Bands.h"
#include "Si5351_IQ.h"
#include "DisplayUI.h"

// bandera global para avisar que cambió la FS
volatile bool g_samplingChanged = false;

#include "I2S_Capture.h"    // versión con doble buffer y frame counter


// 👇👇👇  WRAPPER para el encoder (lo estaba pidiendo)
// Encoder.h llama a sdr_set_sampling(hz), así que lo definimos aquí.
void sdr_set_sampling(uint32_t hz)
{
  // RX (PCM1808) sigue usando Fs variable (span)
  i2s_set_clk(
      I2S_PORT,
      hz,
      I2S_BITS_PER_SAMPLE_32BIT,
      I2S_CHANNEL_STEREO
  );

#if AUDIO_PASSTHROUGH
  // TX (PT8211) SIEMPRE a Fs fija de audio (24 kHz)
  i2s_set_clk(
      I2S_TX_PORT,
      AUDIO_FS_HZ,
      I2S_BITS_PER_SAMPLE_16BIT,
      I2S_CHANNEL_STEREO
  );
#endif

  // actualizar globales (Fs ADC)
  g_samplingHz      = hz;
  g_samplingChanged = true;

  // filtros SIEMPRE diseñados para AUDIO_FS_HZ (24 kHz)
  audio_filters_init((float)AUDIO_FS_HZ);
}




#include "FFT_IQ.h"
#include "Encoder.h"
#include "Layout.h"

TaskHandle_t g_uiTaskHandle = nullptr;
static uint32_t g_uiNotifyDiv = 0;

// ============================================================
//                TAREA DE UI (CORE 0)
// ============================================================
void uiTask(void *pv)
{
  bool layout_fixed = false;

  static double iqR[NUM_SAMPLES];
  static double iqI[NUM_SAMPLES];
  static uint32_t lastFrameId = 0;

  for (;;) {
    // espera a que el core de audio le diga "ya hay frame"
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // primera vez: arma layout completo
    if (!layout_fixed) {
      layout_setWaterfallTop(180);   // WF abajo
      layout_fixed = true;
    }

    // si cambió la Fs → redibuja todo (ticks, barra, escala, zona FFT, WF)
    if (g_samplingChanged) {
      g_samplingChanged = false;
      layout_setWaterfallTop(180);
    }

    // copiar último frame IQ del core de audio
    uint32_t frameId = 0;
    if (!i2s_copy_latest_iq(iqR, iqI, frameId)) {
      vTaskDelay(1);
      continue;
    }

    // si es el mismo frame que el anterior (pasa en 24k) → no procesar
    if (frameId == lastFrameId) {
      vTaskDelay(1);
      continue;
    }
    lastFrameId = frameId;

    // copiar al buffer real de la FFT (solo la UI toca vReal/vImag)
    for (int i = 0; i < NUM_SAMPLES; i++) {
      vReal[i] = iqR[i];
      vImag[i] = iqI[i];
    }

    // entradas / UI
    enc_pollEncoder();
    enc_pollBandButton();
    bands_select_from_lo(g_loHz);
    ui_drawTopDynamic();

    // ====== FFT COMPLETA ======
    fft_applyWindowIQ();
    fft_runComplex();
    fft_packToDisplay();

fft_autoRef();


    // ====== DIBUJO ======
    ui_drawFFTLine();
    ui_drawWaterfallLine();

    vTaskDelay(1);
  }
}

// ============================================================
//                        SETUP
// ============================================================
void setup()
{
  Serial.begin(115200);
  delay(200);
  //sdr_silence_logs();   // si lo tienes

  // entradas
  pinMode(BAND_SW_PIN, INPUT_PULLUP);
  enc_setup();

  // TFT
  tft.init();
  tft.setRotation(3);
  ui_initDisplay();
  ui_drawTopStaticTicks();
  ui_initWaterfall();
  layout_setWaterfallTop(180);

  // EEPROM / storage
  ee_begin();
  ee_load_or_defaults();

  // bandas
  bands_select_from_lo(g_loHz);

  // Si5351
  si_init();

  g_lastApplyMs = 0;
  si_applyIfDue();

  // I2S con el FS actual (de Globals.h)
  i2s_setup();

  // tarea de UI en core 0
  xTaskCreatePinnedToCore(
      uiTask,
      "uiTask",
      8192,
      nullptr,
      1,
      &g_uiTaskHandle,
      0   // core 0
  );
}

// ============================================================
//                        LOOP (CORE 1)
// ============================================================
void loop()
{
  // captura IQ → llena doble buffer
  i2s_captureIQ();

  // despertar a la UI cada 2 capturas
  if (g_uiTaskHandle) {
    g_uiNotifyDiv++;
    if (g_uiNotifyDiv & 0x1) {
      xTaskNotifyGive(g_uiTaskHandle);
    }
  }

  // tareas de baja frecuencia
  si_applyIfDue();
  ee_maybe_commit();
}
