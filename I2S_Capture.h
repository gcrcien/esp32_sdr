// I2S_Capture.h — captura I2S con doble buffer + demod AM/SSB/CW
// SSB con método de phasing usando ±45° por rama (I y Q con mismo group delay)

#ifndef I2S_CAPTURE_H
#define I2S_CAPTURE_H

#include "Config.h"
#include <driver/i2s.h>
#include <math.h>
#include <string.h>

// ==================== configuración básica ====================
#define I2S_DEFAULT_RL      1   // 1 = se asume formato RIGHT,LEFT al entrar
#define SWAP_IQ             1   // 1 = intercambia IQ (por si vienen cruzados)

// 1 = habilita salida de audio hacia el DAC externo (I2S1)
#ifndef AUDIO_PASSTHROUGH
#define AUDIO_PASSTHROUGH 1
#endif

// externs del proyecto (definidos en Globals.h)
extern uint32_t g_samplingHz;
extern uint8_t  g_demodMode;

// ==================== mapeo de modos de demod ====================
// AJUSTA ESTO a tu enum Global si difiere:
//   DEMOD_AM=0, DEMOD_LSB=1, DEMOD_USB=2, DEMOD_DSB=3, DEMOD_CW=4
enum : uint8_t {
  DMOD_AM  = 0,
  DMOD_LSB = 1,
  DMOD_USB = 2,
  DMOD_DSB = 3,
  DMOD_CW  = 4
};

// ==================== defines de filtros por modo ====================
// Targets ajustables:

// AM / DSB → Low-pass de audio (voz AM)
#define FILT_AM_FC_HZ      2800.0f   // antes 3000.0f
#define FILT_AM_Q          0.707f    // butterworth aprox

// SSB (USB/LSB) → ventana voz 300–2600 Hz aprox (HP + LP)
// puedes jugar con estos rangos:
//   FILT_SSB_HP_HZ: 200–400 Hz
//   FILT_SSB_LP_HZ: 2200–3200 Hz
#define FILT_SSB_HP_HZ      350.0f
#define FILT_SSB_LP_HZ     2600.0f
#define FILT_SSB_HP_Q       0.707f   // típico 0.5–1.0
#define FILT_SSB_LP_Q       0.707f   // típico 0.5–1.0

// CW → Band-pass estrecho alrededor del tono CW
#define FILT_CW_FC_HZ       700.0f   // tono CW ~700 Hz
#define FILT_CW_Q           7.0f     // bastante estrecho

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define INV_SQRT2 0.70710678f

// ==================== estructura de biquad y helpers ====================

struct Biquad {
  float b0, b1, b2;
  float a1, a2;
  float z1, z2;
};

inline void biquad_reset(Biquad &f)
{
  f.z1 = 0.0f;
  f.z2 = 0.0f;
}

inline float biquad_process(Biquad &f, float x)
{
  float y = f.b0 * x + f.z1;
  f.z1 = f.b1 * x + f.z2 - f.a1 * y;
  f.z2 = f.b2 * x - f.a2 * y;
  return y;
}

inline void biquad_set_lowpass(Biquad &f, float fs, float fc, float Q)
{
  if (fc <= 0.0f) fc = 10.0f;
  if (fc > 0.45f * fs) fc = 0.45f * fs;
  if (Q <= 0.0f) Q = 0.707f;

  float w0    = 2.0f * (float)M_PI * (fc / fs);
  float c     = cosf(w0);
  float s     = sinf(w0);
  float alpha = s / (2.0f * Q);

  float b0 = (1.0f - c) * 0.5f;
  float b1 = 1.0f - c;
  float b2 = (1.0f - c) * 0.5f;
  float a0 = 1.0f + alpha;
  float a1 = -2.0f * c;
  float a2 = 1.0f - alpha;

  f.b0 = b0 / a0;
  f.b1 = b1 / a0;
  f.b2 = b2 / a0;
  f.a1 = a1 / a0;
  f.a2 = a2 / a0;

  biquad_reset(f);
}

inline void biquad_set_bandpass(Biquad &f, float fs, float fc, float Q)
{
  if (fc <= 0.0f) fc = 10.0f;
  if (fc > 0.45f * fs) fc = 0.45f * fs;
  if (Q <= 0.0f) Q = 1.0f;

  float w0    = 2.0f * (float)M_PI * (fc / fs);
  float c     = cosf(w0);
  float s     = sinf(w0);
  float alpha = s / (2.0f * Q);

  float b0 =   alpha;
  float b1 =   0.0f;
  float b2 =  -alpha;
  float a0 =   1.0f + alpha;
  float a1 =  -2.0f * c;
  float a2 =   1.0f - alpha;

  f.b0 = b0 / a0;
  f.b1 = b1 / a0;
  f.b2 = b2 / a0;
  f.a1 = a1 / a0;
  f.a2 = a2 / a0;

  biquad_reset(f);
}

inline void biquad_set_highpass(Biquad &f, float fs, float fc, float Q)
{
  if (fc <= 0.0f) fc = 10.0f;
  if (fc > 0.45f * fs) fc = 0.45f * fs;
  if (Q <= 0.0f) Q = 0.707f;

  float w0    = 2.0f * (float)M_PI * (fc / fs);
  float c     = cosf(w0);
  float s     = sinf(w0);
  float alpha = s / (2.0f * Q);

  float b0 =  (1.0f + c) * 0.5f;
  float b1 = -(1.0f + c);
  float b2 =  (1.0f + c) * 0.5f;
  float a0 =   1.0f + alpha;
  float a1 =  -2.0f * c;
  float a2 =   1.0f - alpha;

  f.b0 = b0 / a0;
  f.b1 = b1 / a0;
  f.b2 = b2 / a0;
  f.a1 = a1 / a0;
  f.a2 = a2 / a0;

  biquad_reset(f);
}

// ==================== Hilbert FIR ±45° por rama ====================

// Usamos un FIR tipo Hilbert común (taps globales) y, por cada rama (I y Q),
// tenemos un buffer de estado. De ahí sacamos x_delay y H{x}, y construimos
// x_-45 y x_+45 con:
//   x_-45 = (x_delay + H{x}) / sqrt(2)
//   x_+45 = (x_delay - H{x}) / sqrt(2)
// Ambos tienen el mismo group delay, y difieren ~90° entre sí.

#define HILB_N 37   // impar; puedes probar 31 / 47

static float s_hilb_taps[HILB_N];

struct Hilbert45 {
  float state[HILB_N];
};

static Hilbert45 s_hilb_I45;
static Hilbert45 s_hilb_Q45;

// Inicializa taps del Hilbert (truncado + ventana Hamming)
inline void hilbert45_init()
{
  int M = (HILB_N - 1) / 2;
  for (int k = 0; k < HILB_N; ++k) {
    int n = k - M;
    float h;
    if (n == 0) {
      h = 0.0f;
    } else if ((n & 1) == 0) {
      h = 0.0f;
    } else {
      h = 2.0f / ((float)M_PI * (float)n);   // 2/(π·n) para n impar
    }

    float w = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * (float)k / (float)(HILB_N - 1));
    s_hilb_taps[k] = h * w;
  }

  memset(s_hilb_I45.state, 0, sizeof(s_hilb_I45.state));
  memset(s_hilb_Q45.state, 0, sizeof(s_hilb_Q45.state));
}

// Procesa UNA muestra por una rama (I o Q) y saca ±45° con mismo delay
inline void hilbert45_process(Hilbert45 &f, float x, float &x_m45, float &x_p45)
{
  // shift del delay line
  for (int i = HILB_N - 1; i > 0; --i) {
    f.state[i] = f.state[i - 1];
  }
  f.state[0] = x;

  // salida Hilbert (aprox +90°)
  float yH = 0.0f;
  for (int i = 0; i < HILB_N; ++i) {
    yH += s_hilb_taps[i] * f.state[i];
  }

  // muestra "centrada" = mismo group delay que Hilbert
  const int M = (HILB_N - 1) / 2;
  float x_d = f.state[M];

  // construir ±45° alrededor de x_d
  x_m45 = INV_SQRT2 * (x_d + yH);  // -45°
  x_p45 = INV_SQRT2 * (x_d - yH);  // +45°
}

// ==================== filtros por modo ====================

static Biquad s_filt_am;

// Para SSB hacemos: High-pass (I/Q) + Low-pass (I/Q)
static Biquad s_filt_ssb_I;      // HP I
static Biquad s_filt_ssb_Q;      // HP Q
static Biquad s_filt_ssb_I_lp;   // LP I
static Biquad s_filt_ssb_Q_lp;   // LP Q

static Biquad s_filt_cw;

// === Low-pass anti-alias para IQ SOLO en el camino de audio ===
#define IQ_AA_FC_HZ   3500.0f   // corte aprox 3.5 kHz en Fs_ADC
#define IQ_AA_Q       0.707f    // butterworth aprox

static Biquad s_iqAA_I;         // LP en rama I (antes de decimar a audio)
static Biquad s_iqAA_Q;         // LP en rama Q (antes de decimar a audio)

// DC-blocker adicional específico para SSB (para matar el tono cercano a 0 Hz)
static float s_ssb_dc = 0.05f;  // Filtro DC-F-bajas
#define SSB_DC_ALPHA 0.07f      // más alto = HP más agresivo en SSB

// Post-filtro global + estimador de ruido
static float s_env_dc      = 0.0f;  // DC del audio (post-filtro)
static float s_env_am_dc   = 0.0f;  // DC de la envolvente AM
static float s_lp_y        = 0.0f;
static float s_noise_env   = 0.0f;
static float s_sig_env     = 0.0f;  // envolvente “rápida” de la señal

// Coeficientes "globales" del post-filtro / denoise
#define AUDIO_GAIN    1.0f     // Ganancia global de audio (lineal)

// Inicializar filtros de audio según Fs actual (Fs AUDIO)
inline void audio_filters_init(float fs)
{
  // AM
  biquad_set_lowpass(s_filt_am, fs, FILT_AM_FC_HZ, FILT_AM_Q);

  // SSB: HP + LP por rama
  biquad_set_highpass(s_filt_ssb_I,    fs, FILT_SSB_HP_HZ, FILT_SSB_HP_Q);
  biquad_set_highpass(s_filt_ssb_Q,    fs, FILT_SSB_HP_HZ, FILT_SSB_HP_Q);
  biquad_set_lowpass (s_filt_ssb_I_lp, fs, FILT_SSB_LP_HZ, FILT_SSB_LP_Q);
  biquad_set_lowpass (s_filt_ssb_Q_lp, fs, FILT_SSB_LP_HZ, FILT_SSB_LP_Q);

  // CW
  biquad_set_bandpass(s_filt_cw, fs, FILT_CW_FC_HZ, FILT_CW_Q);

  biquad_reset(s_filt_am);
  biquad_reset(s_filt_ssb_I);
  biquad_reset(s_filt_ssb_Q);
  biquad_reset(s_filt_ssb_I_lp);
  biquad_reset(s_filt_ssb_Q_lp);
  biquad_reset(s_filt_cw);

  s_env_dc      = 0.0f;
  s_env_am_dc   = 0.0f;
  s_lp_y        = 0.0f;
  s_noise_env   = 0.0f;
  s_sig_env     = 0.0f;
  s_ssb_dc      = 0.0f;     // reset del DC-blocker SSB

  hilbert45_init();
}

// Inicializar LP anti-alias en IQ a Fs del ADC (solo usado en audio)
inline void iq_antialias_init(float fs_adc)
{
  biquad_set_lowpass(s_iqAA_I, fs_adc, IQ_AA_FC_HZ, IQ_AA_Q);
  biquad_set_lowpass(s_iqAA_Q, fs_adc, IQ_AA_FC_HZ, IQ_AA_Q);
}


// ====== Parámetros del post-filtro / noise gate ======

// Filtro de DC muy lento (cuánto tarda en seguir el offset)
#define DENOISE_DC_ALPHA        0.0035f   // rango típico: 0.0005f – 0.005f

// Envolvente rápida de señal (qué tan rápido detecta voz/picos)
#define DENOISE_SIG_ALPHA       0.20f     // rango típico: 0.05f – 0.30f

// Seguimiento del piso de ruido (baja = se adapta rápido cuando el ruido disminuye)
#define DENOISE_NOISE_ALPHA_DOWN 0.040f   // rango típico: 0.002f – 0.050f
// Cuando el ruido sube, que sea más lento
#define DENOISE_NOISE_ALPHA_UP   0.0255f  // rango típico: 0.0005f – 0.010f

// Umbral del gate en múltiplos del piso de ruido
#define DENOISE_GATE_MULT       5.0f      // rango típico: 2.0f – 4.0f
// Atenuación máxima dentro del gate (0.1 = muy agresivo, 0.4 = suave)
#define DENOISE_GATE_GAIN       0.025f    // rango típico: 0.10f – 0.40f

// Suavizado final (low-pass sobre la salida): más alto = más apagado
// A Fs=24k, alpha=0.40 da una fc ~2kHz–3kHz aprox → menos hiss alto.
#define DENOISE_LP_ALPHA        0.25f     // rango típico útil: 0.30f – 0.50f


// Low-pass global + "denoise" ligero (encima de los filtros por modo)
// Post-proceso global del audio (DC, noise gate, suavizado)
float dsp_post_filter(float x)
{
  // 1) Quitar DC muy lento del audio
  s_env_dc += DENOISE_DC_ALPHA * (x - s_env_dc);
  float y = x - s_env_dc;

  // 2) Envolvente
  float env = fabsf(y);

  // Señal (envolvente rápida)
  s_sig_env = (1.0f - DENOISE_SIG_ALPHA) * s_sig_env
              + DENOISE_SIG_ALPHA * env;

  // Piso de ruido (envolvente lenta)
  float noiseAlpha = (env < s_noise_env)
                     ? DENOISE_NOISE_ALPHA_DOWN
                     : DENOISE_NOISE_ALPHA_UP;
  s_noise_env = (1.0f - noiseAlpha) * s_noise_env + noiseAlpha * env;

  // 3) Noise gate
  float gateTh = s_noise_env * DENOISE_GATE_MULT;
  if (gateTh < 1e-7f) gateTh = 1e-7f;

  if (s_sig_env < gateTh) {
    float ratio = s_sig_env / gateTh;    // 0..1
    float gain  = ratio * ratio;         // “soft knee”
    gain *= DENOISE_GATE_GAIN;           // factor global
    y *= gain;
  }

  // 4) LP final (da forma al espectro de audio)
  s_lp_y = (1.0f - DENOISE_LP_ALPHA) * s_lp_y + DENOISE_LP_ALPHA * y;
  return s_lp_y;
}



// ==================== buffers compartidos IQ ====================

static double g_iqBufR[2][NUM_SAMPLES];
static double g_iqBufI[2][NUM_SAMPLES];
static volatile int  g_iq_wr_idx      = 0;
static volatile int  g_iq_latest_idx  = -1;
static volatile bool g_iq_has_frame   = false;
static volatile uint32_t g_iq_frame_counter = 0;

// ==================== demod IQ -> audio (por muestra) ====================

// Procesa UNA muestra IQ normalizada (-1..1) y regresa audio (-1..1)
inline float dsp_process_sample(float i_norm, float q_norm)
{
  float audio = 0.0f;

  switch (g_demodMode) {
    case DMOD_AM: {
        // Demod AM: envolvente de la señal analítica
        float mag = sqrtf(i_norm * i_norm + q_norm * q_norm);

        // Quitar DC de la envolvente (promedio lento)
        const float ENV_ALPHA = 0.001f;
        s_env_am_dc += ENV_ALPHA * (mag - s_env_am_dc);

        float a = mag - s_env_am_dc;         // audio AM sin DC
        a = biquad_process(s_filt_am, a);    // LP AM

        audio = a;
        break;
      }

    case DMOD_USB:
    case DMOD_LSB: {
        // ===== SSB por método de phasing usando ±45° =====
        // 1) ventana de voz por rama I y Q: HP seguido de LP
        float i_bp = biquad_process(s_filt_ssb_I,    i_norm);      // HP
        i_bp       = biquad_process(s_filt_ssb_I_lp, i_bp);        // LP

        float q_bp = biquad_process(s_filt_ssb_Q,    q_norm);      // HP
        q_bp       = biquad_process(s_filt_ssb_Q_lp, q_bp);        // LP

        // 2) redes de fase ±45° por rama (mismo group delay)
        float I_m45, I_p45, Q_m45, Q_p45;
        hilbert45_process(s_hilb_I45, i_bp, I_m45, I_p45);
        hilbert45_process(s_hilb_Q45, q_bp, Q_m45, Q_p45);

        // 3) combinaciones para USB / LSB
        float a;
        if (g_demodMode == DMOD_USB) {
          // USB: I_d + H_Q  → INV_SQRT2 * (I_-45 + I_+45 - Q_-45 + Q_+45)
          a = INV_SQRT2 * (I_m45 + I_p45 - Q_m45 + Q_p45);
        } else {
          // LSB: I_d - H_Q  → INV_SQRT2 * (I_-45 + I_+45 + Q_-45 - Q_+45)
          a = INV_SQRT2 * (I_m45 + I_p45 + Q_m45 - Q_p45);
        }

        // 🔹 DC / HP adicional específico para SSB (mata tono y graves cercanos a 0 Hz)
        s_ssb_dc += SSB_DC_ALPHA * (a - s_ssb_dc);
        a -= s_ssb_dc;

        audio = a;
        break;
      }

    case DMOD_DSB: {
        // DSB simple: tomar I, filtrarlo con LP AM
        float a = biquad_process(s_filt_am, i_norm);
        audio = a;
        break;
      }

    case DMOD_CW: {
        // CW: tratarlo como SSB (ej. tipo USB) + filtro muy estrecho
        float a = i_norm - q_norm;             // combinación sencilla
        a = biquad_process(s_filt_cw, a);      // filtro estrecho centrado en tono CW
        audio = a;
        break;
      }

    default:
      // Fallback: raw I sin filtros
      audio = i_norm;
      break;
  }

  // Post-filtro global + denoise ligero
  float y = dsp_post_filter(audio);

  // Ganancia global
  y *= AUDIO_GAIN;

  // Limitar a -1..1 para evitar clipping duro
  if (y >  1.0f) y =  1.0f;
  if (y < -1.0f) y = -1.0f;

  return y;
}

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

  i2s_pin_config_t pin_config = {
    .mck_io_num   = I2S_MCLK_PIN,   // NO sacar MCLK por ningún pin
    .bck_io_num   = I2S_BCK_PIN,         // 26
    .ws_io_num    = I2S_LRCK_PIN,        // 25
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = I2S_DATA_PIN         // 17
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
    .sample_rate = (uint32_t)AUDIO_FS_HZ,   // 🔹 DAC SIEMPRE a 24 kHz
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_driver_install(I2S_TX_PORT, &i2s_tx_config, 0, nullptr);
  // Aquí también uso tus macros originales PARA TX
  i2s_pin_config_t pin_tx = {
    .mck_io_num   = I2S_PIN_NO_CHANGE,   // PT8211 no usa MCLK
    .bck_io_num   = I2S_TX_BCK_PIN,      // 13
    .ws_io_num    = I2S_TX_WS_PIN,       // 12
    .data_out_num = I2S_TX_DATA_PIN,     // 5
    .data_in_num  = I2S_PIN_NO_CHANGE
  };
  i2s_set_pin(I2S_TX_PORT, &pin_tx);
#endif
}


// ==================== setup combinado usado por setup() ====================

inline void i2s_setup()
{
  i2s_setup_rx();
  i2s_setup_tx();

  // Inicializar filtros de audio para Fs FIJA (24 kHz)
  audio_filters_init((float)AUDIO_FS_HZ);

  // Inicializar low-pass anti-alias de IQ a Fs del ADC (solo para audio)
  iq_antialias_init((float)g_samplingHz);
}

// ==================== captura IQ (core 1) ====================

inline void i2s_captureIQ()
{
  const int frames          = NUM_SAMPLES;
  const int bytes_per_frame = 8;             // L+R en 32 bits
  const int total_bytes     = frames * bytes_per_frame;

  static uint8_t i2s_data[NUM_SAMPLES * 8];

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
      return;
    }
    if (br == 0) {
      return;
    }
    remaining -= br;
    p         += br;
  }

  // Frame COMPLETO
  int wr = g_iq_wr_idx;
  double *dstR = g_iqBufR[wr];
  double *dstI = g_iqBufI[wr];

  int32_t *w = (int32_t *)i2s_data;

  static bool autodetect_done = false;
  static bool data_is_RL      = (I2S_DEFAULT_RL != 0);

  // Autodetección RL / LR
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

#if AUDIO_PASSTHROUGH
  // ====== DSP: IQ -> audio (Fs fija) -> DAC ======
  static int16_t dacBuf[NUM_SAMPLES * 2];

  const float NORM      = 1.0f / 8388608.0f;   // 1 / 2^23
  const float OUT_SCALE = 32767.0f;

  // Factor de decimación: ADC / AUDIO_FS
  // (asumimos que g_samplingHz es múltiplo entero de AUDIO_FS_HZ: 24k/48k/96k)
  int decim = (int)((float)g_samplingHz / (float)AUDIO_FS_HZ);
  if (decim < 1) decim = 1;
  if (decim > frames) decim = frames;

  int outIndex = 0;
#endif

  for (int n = 0; n < frames; n++) {
    int32_t w0 = w[n * 2 + 0] >> 8; // 24 bits
    int32_t w1 = w[n * 2 + 1] >> 8;

    int32_t sL, sR;
    if (data_is_RL) {
      sR = w0;
      sL = w1;
    } else {
      sL = w0;
      sR = w1;
    }

    // === IQ "bruto" para UI / FFT (ancho completo) ===
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

#if AUDIO_PASSTHROUGH
    // === Camino de audio: IQ filtrado + decimación con anti-alias ===
    float fI = (float)sI;
    float fQ = (float)sQ;

    // Low-pass anti-alias en IQ a Fs_ADC
    float fI_filt = biquad_process(s_iqAA_I, fI);
    float fQ_filt = biquad_process(s_iqAA_Q, fQ);

    // Decimación: solo generamos audio cada "decim" muestras
    if ((n % decim) == 0) {
      float i_norm = fI_filt * NORM;
      float q_norm = fQ_filt * NORM;

      // El DSP ahora "ve" una Fs ≈ AUDIO_FS_HZ (24 kHz)
      float a = dsp_process_sample(i_norm, q_norm);  // -1..1

      int16_t s = (int16_t)(a * OUT_SCALE);

      dacBuf[2 * outIndex + 0] = s;   // L
      dacBuf[2 * outIndex + 1] = s;   // R
      outIndex++;
    }
#endif
  }

  // Quitar DC del frame IQ (solo para la UI / FFT)
  double meanI = accI / (double)frames;
  double meanQ = accQ / (double)frames;
  for (int n = 0; n < frames; n++) {
    dstR[n] -= meanI;
    dstI[n] -= meanQ;
  }

#if AUDIO_PASSTHROUGH
  // Escribir al DAC solo lo que realmente generamos
  size_t audioFrames = (size_t)outIndex;
  size_t to_write = audioFrames * 2 * sizeof(int16_t);
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

  // Publicar frame para la UI / FFT
  g_iq_latest_idx  = wr;
  g_iq_has_frame   = true;
  g_iq_frame_counter++;
  g_iq_wr_idx      = wr ^ 1;
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
