// Encoder.h
#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "Config.h"
#include "Globals.h"
#include "Storage.h"
#include "Bands.h"

// ================== externs del proyecto ==================
extern void ee_set_lo(uint32_t hz);
extern void ee_set_ref(uint32_t hz);
extern void ee_set_iq_gain(double g);
extern void ee_set_iq_phase(double ph);
extern void bands_select_from_lo(uint32_t hz);
extern void bands_next_band();
extern void ui_drawBandBar();
extern void ui_drawTopDynamic();

extern uint32_t g_loHz;
extern uint32_t g_refMag;
extern int      g_bandIndex;
extern int      g_encMode;     // ENC_MODE_NORMAL / ENC_MODE_IQ_GAIN / ENC_MODE_IQ_PHASE
extern double   g_iqGain;
extern double   g_iqPhase;

extern const uint32_t STEP_TABLE[];
extern int            g_stepIdx;

// sampleo (S1)
extern const uint32_t g_samplingTable[3];
extern int      g_samplingIndex;
extern uint32_t g_samplingHz;
void sdr_set_sampling(uint32_t hz);

// demod (nuevo)
extern uint8_t g_demodMode;
#ifndef DEMOD_COUNT
#define DEMOD_COUNT 4   // AM, LSB, USB, DSB
#endif

// ================== pines por defecto ==================
#ifndef ENC_A
#define ENC_A 33
#endif
#ifndef ENC_B
#define ENC_B 27
#endif
#ifndef ENC_SW
#define ENC_SW 32
#endif
#ifndef BAND_SW_PIN
#define BAND_SW_PIN 16     // S2
#endif
#ifndef S1_PIN
#define S1_PIN 17
#endif
#ifndef BAND_LONG_MS
#define BAND_LONG_MS 600
#endif

// ================== selección de driver ==================
// 1 = tu driver original con LUT (mejor resolución)
// 0 = driver simple: interrupción en A y se lee B
#ifndef ENC_DRIVER_MODE
#define ENC_DRIVER_MODE 1
#endif

// tiempo mínimo entre IRQ para no reventar la CPU
#ifndef ENC_ISR_MIN_US
#define ENC_ISR_MIN_US 60     // antes estaba en ~120 us
#endif

// =========================================================
// 1) DRIVER 1: Encoder por LUT (tu original) PERO:
//    - debounce configurable
//    - NO se pierden cuartos (guarda el residuo)
// =========================================================
#if ENC_DRIVER_MODE == 1

static volatile int8_t   s_enc_quarters = 0;
static volatile uint8_t  s_enc_prev_ab  = 0;
static volatile uint32_t s_enc_last_us  = 0;

// LUT de 16 estados
static const int8_t ENC_LUT[16] = {
  0, -1, +1, 0,
  +1, 0, 0, -1,
  -1, 0, 0, +1,
  0, +1, -1, 0
};

// 👉 OJO: SIN IRAM_ATTR para evitar el “dangerous relocation”
void enc_isr()
{
  uint32_t now = micros();
  if ((now - s_enc_last_us) < ENC_ISR_MIN_US) return;
  s_enc_last_us = now;

  uint8_t a = (uint8_t)digitalRead(ENC_A);
  uint8_t b = (uint8_t)digitalRead(ENC_B);

  uint8_t curr = (a ? 1 : 0) | (b ? 2 : 0);
  uint8_t idx  = (s_enc_prev_ab << 2) | curr;
  int8_t  d    = ENC_LUT[idx];
  if (d != 0) {
    s_enc_quarters += d;
  }
  s_enc_prev_ab = curr;
}

#endif // ENC_DRIVER_MODE == 1

// =========================================================
// 2) DRIVER 0: Encoder simple (IRQ en A → leo B)
//    - menos interrupciones
//    - no hay LUT
// =========================================================
#if ENC_DRIVER_MODE == 0

static volatile int16_t s_enc_delta = 0;
static volatile uint32_t s_enc_last_us = 0;

void enc_isr_a()
{
  uint32_t now = micros();
  if ((now - s_enc_last_us) < ENC_ISR_MIN_US) return;
  s_enc_last_us = now;

  // lee el otro canal
  bool b = (digitalRead(ENC_B) != 0);
  if (b) s_enc_delta++;   // un sentido
  else   s_enc_delta--;   // el otro
}

#endif // ENC_DRIVER_MODE == 0

// =====================================================
// 3) setup (común)
// =====================================================
inline void enc_setup()
{
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(BAND_SW_PIN, INPUT_PULLUP);
  pinMode(S1_PIN, INPUT_PULLUP);

#if ENC_DRIVER_MODE == 1
  // inicializa estado previo
  uint8_t a = (uint8_t)digitalRead(ENC_A);
  uint8_t b = (uint8_t)digitalRead(ENC_B);
  s_enc_prev_ab = (a ? 1 : 0) | (b ? 2 : 0);

  attachInterrupt(digitalPinToInterrupt(ENC_A), enc_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), enc_isr, CHANGE);

#else
  // modo simple: solo A
  attachInterrupt(digitalPinToInterrupt(ENC_A), enc_isr_a, CHANGE);
#endif
}

// =====================================================
// 4) Botón de banda (S2)
//     - short → siguiente banda
//     - long  → siguiente MODO de demod
// =====================================================
inline void enc_pollBandButton()
{
  static int      prevBtn   = HIGH;
  static uint32_t pressMs   = 0;
  static bool     longFired = false;

  int nowBtn = digitalRead(BAND_SW_PIN);

  // flanco de bajada
  if (prevBtn == HIGH && nowBtn == LOW) {
    pressMs   = millis();
    longFired = false;
  }

  // sigue presionado
  if (prevBtn == LOW && nowBtn == LOW) {
    if (!longFired && (millis() - pressMs) >= BAND_LONG_MS) {
      longFired = true;

      // long → demod
      g_demodMode++;
      if (g_demodMode >= DEMOD_COUNT)
        g_demodMode = 0;

      ui_drawTopDynamic();
      ui_drawBandBar();
    }
  }

  // flanco de subida
  if (prevBtn == LOW && nowBtn == HIGH) {
    uint32_t dur = millis() - pressMs;
    if (dur < BAND_LONG_MS && !longFired) {
      // short → banda
      bands_next_band();
      ui_drawBandBar();
    }
  }

  prevBtn = nowBtn;
}

// =====================================================
// 5) poll principal (lo llama tu uiTask())
//     aquí metemos el "no pierdas cuartos"
// =====================================================
inline void enc_pollEncoder()
{
  // primero S2
  enc_pollBandButton();

  // S1 (cambiar sample rate)
  {
    static bool     s1Prev = false;
    static uint32_t s1Down = 0;
    bool s1Now = (digitalRead(S1_PIN) == LOW);

    if (!s1Prev && s1Now) {
      s1Down = millis();
    }
    if (s1Prev && !s1Now) {
      uint32_t dur = millis() - s1Down;
      if (dur < 250) {
        g_samplingIndex++;
        if (g_samplingIndex >= 3) g_samplingIndex = 0;
        g_samplingHz = g_samplingTable[g_samplingIndex];
        sdr_set_sampling(g_samplingHz);
      }
    }
    s1Prev = s1Now;
  }

  // ==================== LECTURA DEL ENCODER ====================
  int16_t steps = 0;

#if ENC_DRIVER_MODE == 1
  // ---- modo LUT: tenemos cuartos → los acumulamos sin perderlos
  static int8_t s_quarter_rem = 0;
  int8_t quarters;
  noInterrupts();
  quarters = s_enc_quarters;
  s_enc_quarters = 0;
  interrupts();

  quarters += s_quarter_rem;     // agrega lo que sobró antes
  steps = quarters / 4;          // pasos completos
  s_quarter_rem = quarters % 4;  // guarda el residuo (0..3 o 0..-3)

#else
  // ---- modo simple: delta directo
  noInterrupts();
  steps = s_enc_delta;
  s_enc_delta = 0;
  interrupts();
#endif

  // ==================== APLICAR MOVIMIENTO ====================
  if (steps != 0) {
    bool s1Pressed = (digitalRead(S1_PIN) == LOW);

    // ===== modo normal =====
    if (g_encMode == ENC_MODE_NORMAL) {

      if (s1Pressed) {
        // mover REF
        int32_t delta = steps * 1000000L;
        int64_t nr    = (int64_t)g_refMag + delta;
        if (nr < 200000L)    nr = 200000L;
        if (nr > 700000000L)  nr = 700000000L;
        ee_set_ref((uint32_t)nr);
        ui_drawTopDynamic();

      } else {
        // mover LO
        uint32_t st = STEP_TABLE[g_stepIdx];
        int64_t  nf = (int64_t)g_loHz + (int64_t)steps * (int64_t)st;
        if (nf < 5000000L)   nf = 5000000L;
        if (nf > 31000000L)  nf = 31000000L;
        uint32_t finalHz = (uint32_t)nf;
        ee_set_lo(finalHz);

        int oldIdx = g_bandIndex;
        bands_select_from_lo(finalHz);
        if (g_bandIndex != oldIdx) {
          ui_drawBandBar();
        }
        ui_drawTopDynamic();
      }

    // ===== modo IQ GAIN =====
    } else if (g_encMode == ENC_MODE_IQ_GAIN) {
      g_iqGain += (double)steps * 0.01;
      if (g_iqGain < 0.50) g_iqGain = 0.50;
      if (g_iqGain > 1.50) g_iqGain = 1.50;
      ee_set_iq_gain(g_iqGain);
      ui_drawTopDynamic();

    // ===== modo IQ PHASE =====
    } else {
      g_iqPhase += (double)steps * 0.01;
      if (g_iqPhase < -0.50) g_iqPhase = -0.50;
      if (g_iqPhase >  0.50) g_iqPhase =  0.50;
      ee_set_iq_phase(g_iqPhase);
      ui_drawTopDynamic();
    }
  }

  // ==================== BOTÓN DEL ENCODER ====================
  static bool prevEncBtn = false;
  bool encBtn = (digitalRead(ENC_SW) == LOW);
  if (encBtn && !prevEncBtn) {
    if (g_encMode == ENC_MODE_NORMAL) {
      g_stepIdx++;
      if (g_stepIdx >= (int)(sizeof(STEP_TABLE)/sizeof(STEP_TABLE[0]))) {
        g_stepIdx = 0;
      }
      ui_drawTopDynamic();
    }
  }
  prevEncBtn = encBtn;
}

#endif // ENCODER_H
