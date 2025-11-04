// I2S_Capture.h — captura I2S con doble buffer para 2 cores + opción de TX a DAC
#ifndef I2S_CAPTURE_H
#define I2S_CAPTURE_H

#include "Config.h"
#include <driver/i2s.h>
#include <math.h>
#include <string.h>

// mismo que antes
#define I2S_DEFAULT_RL      1
#define SWAP_IQ             1

// 1 = habilita passthrough de canal I hacia el DAC externo (I2S1)
#ifndef AUDIO_PASSTHROUGH
#define AUDIO_PASSTHROUGH 1
#endif

// ==================== buffers compartidos ====================
static double g_iqBufR[2][NUM_SAMPLES];
static double g_iqBufI[2][NUM_SAMPLES];
static volatile int  g_iq_wr_idx      = 0;
static volatile int  g_iq_latest_idx  = -1;
static volatile bool g_iq_has_frame   = false;
// contador de frames capturados (para la UI)
static volatile uint32_t g_iq_frame_counter = 0;

// ==================== setup I2S RX (PCM1808 en I2S0) ====================
// ==================== setup I2S RX (PCM1808 en I2S0) ====================
inline void i2s_setup_rx()
{
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = (uint32_t)g_samplingHz,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = USE_APLL,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  // Orden correcto: mck, bck, ws, data_out, data_in
  i2s_pin_config_t pin_config = {
    .mck_io_num   = I2S_MCLK_PIN,   // <<< AQUÍ VOLVEMOS A USAR GPIO0
    .bck_io_num   = I2S_BCK_PIN,
    .ws_io_num    = I2S_LRCK_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = I2S_DATA_PIN
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_set_clk(
      I2S_PORT,
      g_samplingHz,
      I2S_BITS_PER_SAMPLE_32BIT,
      I2S_CHANNEL_STEREO
  );
  i2s_zero_dma_buffer(I2S_PORT);
  i2s_start(I2S_PORT);
}


// ==================== setup I2S TX (DAC PT8211 en I2S1) ====================
inline void i2s_setup_tx()
{
#if AUDIO_PASSTHROUGH
  i2s_config_t i2s_tx_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = (uint32_t)g_samplingHz,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format =
      (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false,              // para TX no usamos APLL
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_tx = {
    .mck_io_num   = I2S_PIN_NO_CHANGE,   // sin MCLK
    .bck_io_num   = I2S_TX_BCK_PIN,      // 13
    .ws_io_num    = I2S_TX_WS_PIN,       // 12
    .data_out_num = I2S_TX_DATA_PIN,     // 5
    .data_in_num  = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_TX_PORT, &i2s_tx_config, 0, NULL);
  i2s_set_pin(I2S_TX_PORT, &pin_tx);
  i2s_set_clk(
      I2S_TX_PORT,
      g_samplingHz,
      I2S_BITS_PER_SAMPLE_16BIT,
      I2S_CHANNEL_STEREO
  );
  i2s_zero_dma_buffer(I2S_TX_PORT);
  i2s_start(I2S_TX_PORT);

  // forzar que arranquen los relojes con un buffer de silencio
  static int16_t silence[128 * 2] = {0};  // 128 frames estéreo
  size_t bw = 0;
  i2s_write(
      I2S_TX_PORT,
      silence,
      sizeof(silence),
      &bw,
      portMAX_DELAY
  );
#endif
}

// ==================== setup combinado usado por setup() ====================
inline void i2s_setup()
{
  i2s_setup_rx();
  i2s_setup_tx();
}

// ==================== captura IQ (core 1) ====================
inline void i2s_captureIQ()
{
  const int frames          = NUM_SAMPLES;   // 512
  const int bytes_per_frame = 8;             // L+R en 32 bits
  const int total_bytes     = frames * bytes_per_frame;

  static uint8_t i2s_data[NUM_SAMPLES * 8];

  // tiempo aproximado de un frame para la Fs actual
  float frame_time_ms_f = (1000.0f * (float)NUM_SAMPLES) / (float)g_samplingHz;
  uint32_t rd_timeout_ms = (uint32_t)(frame_time_ms_f * 3.0f);
  if (rd_timeout_ms < 5)   rd_timeout_ms = 5;
  if (rd_timeout_ms > 100) rd_timeout_ms = 100;

  size_t remaining = total_bytes;
  uint8_t *p       = i2s_data;

  while (remaining > 0) {
    size_t br = 0;
    esp_err_t err = i2s_read(
        I2S_PORT,
        p,
        remaining,
        &br,
        pdMS_TO_TICKS(rd_timeout_ms)
    );
    if (err != ESP_OK) {
      // si falló la lectura, no publicamos frame nuevo
      return;
    }
    if (br == 0) {
      // timeout sin datos
      return;
    }
    remaining -= br;
    p         += br;
  }

  // llegamos aquí con el frame COMPLETO
  int wr = g_iq_wr_idx;
  double *dstR = g_iqBufR[wr];
  double *dstI = g_iqBufI[wr];

  int32_t *w = (int32_t *)i2s_data;

  static bool autodetect_done = false;
  static bool data_is_RL      = (I2S_DEFAULT_RL != 0);

  // autodetección RL / LR (si L==R asumimos LR estándar)
  {
    int32_t t0 = w[0] >> 8;
    int32_t t1 = w[1] >> 8;
    if (!autodetect_done) {
      if (t0 == t1) {
        data_is_RL = false;
      }
      autodetect_done = true;
    }
  }

  double accI = 0.0, accQ = 0.0;

  for (int n = 0; n < frames; n++) {
    int32_t w0 = w[n*2 + 0] >> 8;  // 24 bits
    int32_t w1 = w[n*2 + 1] >> 8;

    int32_t sL, sR;
    if (data_is_RL) {
      sR = w0;
      sL = w1;
    } else {
      sL = w0;
      sR = w1;
    }

    double sI, sQ;
#if SWAP_IQ
    sI = (double)sR;
    sQ = (double)sL;
#else
    sI = (double)sL;
    sQ = (double)sR;
#endif

    dstR[n] = sI;
    dstI[n] = sQ;

    accI += sI;
    accQ += sQ;
  }

  // quitar DC del frame
  double meanI = accI / (double)frames;
  double meanQ = accQ / (double)frames;
  for (int n = 0; n < frames; n++) {
    dstR[n] -= meanI;
    dstI[n] -= meanQ;
  }

#if AUDIO_PASSTHROUGH
  // *** PASSTHROUGH BÁSICO ***
  // Canal I → mono → DAC (L y R iguales)
  static int16_t dacBuf[NUM_SAMPLES * 2];
  const double SCALE = 1.0 / 256.0;     // 24 bits >> 8 → 16 bits aprox

  for (int n = 0; n < frames; ++n) {
    double v = dstR[n] * SCALE;

    if (v > 32767.0)  v = 32767.0;
    if (v < -32768.0) v = -32768.0;

    int16_t s = (int16_t)v;
    dacBuf[2*n + 0] = s;   // L
    dacBuf[2*n + 1] = s;   // R
  }

  size_t to_write = (size_t)frames * 2 * sizeof(int16_t);
  uint8_t *bp = (uint8_t*)dacBuf;
  while (to_write > 0) {
    size_t bw = 0;
    esp_err_t err = i2s_write(
        I2S_TX_PORT,
        bp,
        to_write,
        &bw,
        pdMS_TO_TICKS(rd_timeout_ms)
    );
    if (err != ESP_OK || bw == 0) {
      break;
    }
    to_write -= bw;
    bp       += bw;
  }
#endif

  // publicar frame
  g_iq_latest_idx  = wr;
  g_iq_has_frame   = true;
  g_iq_frame_counter++;
  g_iq_wr_idx      = wr ^ 1;   // siguiente buffer
}

// ==================== API para la UI (core 0) ====================
inline bool i2s_copy_latest_iq(double *dstR, double *dstI, uint32_t &outFrameId)
{
  if (!g_iq_has_frame) return false;
  int rd = g_iq_latest_idx;
  if (rd < 0 || rd > 1) return false;

  memcpy(dstR, g_iqBufR[rd], sizeof(double) * NUM_SAMPLES);
  memcpy(dstI, g_iqBufI[rd], sizeof(double) * NUM_SAMPLES);
  outFrameId = g_iq_frame_counter;
  return true;
}

#endif // I2S_CAPTURE_H
