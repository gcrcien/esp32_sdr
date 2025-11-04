// Si5351_IQ.h
#ifndef SI5351_IQ_H
#define SI5351_IQ_H

#include "Config.h"

// Helpers locales
static inline uint32_t roundDownTo4(uint32_t x) {
  return x - (x % 4);
}
static inline uint32_t roundUpTo4(uint32_t x) {
  return ((x + 3) / 4) * 4;
}

inline void si_init()
{
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
//  si5351.set_correction(11268);  // Este es el valor correcto
  si5351.set_correction(115000, SI5351_PLL_INPUT_XO);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
  si5351.output_enable(SI5351_CLK0, 1);
  si5351.output_enable(SI5351_CLK1, 1);
}

// LO en cuadratura (CLK0=I, CLK1=Q) con fase de 90°
// fout_hz = frecuencia deseada
// comp_hz = corrección en Hz (puedes dejar 0)
inline bool si_setLO_quadrature(uint32_t fout_hz, int32_t comp_hz = 0)
{
  // aplicar compensación
  int64_t target = (int64_t)fout_hz + (int64_t)comp_hz;
  if (target < 0) target = 0;
  uint32_t fout = (uint32_t)target;

  // rango del VCO del Si5351
  const uint32_t PLL_MIN = 600000000UL;
  const uint32_t PLL_MAX = 900000000UL;

  // calcular rango de R posible
  uint32_t Rmin = (PLL_MIN + fout - 1) / fout;  // ceil(PLL_MIN/fout)
  uint32_t Rmax = (PLL_MAX) / fout;             // floor(PLL_MAX/fout)
  if (Rmin < 4) Rmin = 4;
  if (Rmax < 4) Rmax = 4;
  if (Rmax > 900) Rmax = 900;                   // tope de seguridad

  // 1) intentar usar el mayor R que quepa en fase (<=127) y sea múltiplo de 4
  uint32_t Rcap = (Rmax > 127 ? 127 : Rmax);
  uint32_t R = roundDownTo4(Rcap);

  // si ese R quedó por debajo de lo mínimo, usar el mínimo posible múltiplo de 4
  if (R < Rmin) {
    R = roundUpTo4(Rmin);
    if (R > Rmax) {
      // no hay solución válida para esa frecuencia
      return false;
    }
  }

  // frecuencia de la PLL
  uint64_t pll_freq = (uint64_t)fout * (uint64_t)R;
  if (pll_freq < PLL_MIN || pll_freq > PLL_MAX) {
    return false;
  }

  // ambas salidas desde PLLA y en modo entero
  si5351.set_ms_source(SI5351_CLK0, SI5351_PLLA);
  si5351.set_ms_source(SI5351_CLK1, SI5351_PLLA);
  si5351.set_int(SI5351_CLK0, 1);
  si5351.set_int(SI5351_CLK1, 1);

  // programar frecuencia explícitamente
  si5351.set_freq_manual(
    (uint64_t)fout * SI5351_FREQ_MULT,
    pll_freq * SI5351_FREQ_MULT,
    SI5351_CLK0
  );
  si5351.set_freq_manual(
    (uint64_t)fout * SI5351_FREQ_MULT,
    pll_freq * SI5351_FREQ_MULT,
    SI5351_CLK1
  );

  // ===== fase =====
  // paso de fase = 1 / (4*R) del periodo
  // para 90° → hay que escribir "R"
  uint8_t phaseSteps;
  if (R > 127) {
    // si no cabe en 7 bits, lo más que podemos escribir es 127
    // (no hay flag porque lo quitaste)
    phaseSteps = 127;
  } else {
    phaseSteps = (uint8_t)R;   // 90° exactos
  }

  si5351.set_phase(SI5351_CLK0, 0);
  si5351.set_phase(SI5351_CLK1, phaseSteps);

  // aplicar fases
  si5351.pll_reset(SI5351_PLLA);

  // asegurar salida
  si5351.output_enable(SI5351_CLK0, 1);
  si5351.output_enable(SI5351_CLK1, 1);

  return true;
}

// igual que antes: aplica cuando ya toca actualizar g_loHz
inline void si_applyIfDue()
{
  uint32_t now = millis();
  if ((int32_t)(now - g_lastApplyMs) >= 0) {
    static uint32_t last_applied = 0;
    if (g_loHz != last_applied) {
      si_setLO_quadrature(g_loHz, 0);
      last_applied = g_loHz;
    }
    g_lastApplyMs = UINT32_MAX;
  }
}

#endif
