// Config.h
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <si5351.h>
#include <EEPROM.h>
#include <driver/i2s.h>
#include <math.h>

// ================== LAYOUT GENERAL ==================
// Subimos la barra porque en DisplayUI usamos FONT7 (~48 px)
#define TOPBAR_H               52    // antes 18
// barra de bandas (debajo de la topbar)
#define BANDBAR_H              14
// alto fijo de la FFT (lo usan DisplayUI.h y Layout.h)
#define FFT_FIXED_H            140
// separación entre FFT y waterfall
#define FFT_WF_GAP             0
// barra de escala (la que dibuja marcas de frecuencia)
#define SCALEBAR_H             14

// waterfall
#define DRAW_WIDTH             480
#define WATERFALL_HEIGHT       80
#define WATERFALL_WIDTH        DRAW_WIDTH

// Top de la escala en dB (relativo a REF)
#define UI_DB_TOP   -15.0f       // 0 dB = pico a nivel REF
// Span de la escala (cuántos dB se muestran hacia abajo)
#define UI_DB_SPAN  20.0f      // prueba 40, 50, 60...

// ================== MUESTRAS / FFT ==================
#define NUM_SAMPLES            512

// ================== P I N E S ==================
// encoder
#define ENC_A                  33
#define ENC_B                  27
#define ENC_SW                 32

// botón de bandas
#define BAND_SW_PIN            16
#define BAND_LONG_MS           500

// selector extra
#define S1_PIN                 14

// ================== I2S ==================
#define I2S_PORT               I2S_NUM_0
#define I2S_BCK_PIN            26
#define I2S_LRCK_PIN           25
#define I2S_DATA_PIN           17
#define I2S_MCLK_PIN           0
#define USE_APLL               true

// ==== I2S TX (DAC externo) ====
#define I2S_TX_PORT      I2S_NUM_1
#define I2S_TX_BCK_PIN   13   // gpio13 > bck
#define I2S_TX_WS_PIN    12   // gpio12 > ws
#define I2S_TX_DATA_PIN  5    // gpio5  > din

// ================== EEPROM ==================
#define EEPROM_BYTES           64
#define EE_MAGIC_ADDR          0
#define EE_MAGIC_VAL           0x53445231UL
#define EE_LO_ADDR             4
#define EE_REF_ADDR            8

// nuevos para IQ
#define EE_IQGAIN_ADDR         12     // double → 12..19
#define EE_IQPHASE_ADDR        20     // double → 20..27

// ================== Encoder ==================
#define ENC_COUNTS_PER_DETENT  4
#define DC_MAX_MS              350

// ================== Modo encoder ==================
enum {
  ENC_MODE_NORMAL = 0,
  ENC_MODE_IQ_GAIN,
  ENC_MODE_IQ_PHASE
};

#endif // CONFIG_H
