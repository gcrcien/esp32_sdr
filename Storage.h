// Storage.h
#ifndef STORAGE_H
#define STORAGE_H

#include "Config.h"
#include "Globals.h"

static bool     ee_dirty  = false;
static uint32_t ee_lastMs = 0;
static const uint32_t EE_DEBOUNCE_MS = 1000;  // 1s

inline void ee_begin() {
  EEPROM.begin(EEPROM_BYTES);
}

inline void ee_load_or_defaults() {
  uint32_t magic = 0;
  EEPROM.get(EE_MAGIC_ADDR, magic);

  if (magic != EE_MAGIC_VAL) {
    // EEPROM virgen → escribe defaults
    EEPROM.put(EE_MAGIC_ADDR,   EE_MAGIC_VAL);
    EEPROM.put(EE_LO_ADDR,      g_loHz);
    EEPROM.put(EE_REF_ADDR,     g_refMag);
    EEPROM.put(EE_IQGAIN_ADDR,  g_iqGain);
    EEPROM.put(EE_IQPHASE_ADDR, g_iqPhase);
    EEPROM.commit();
    return;
  }

  // EEPROM válida → leer
  EEPROM.get(EE_LO_ADDR,  g_loHz);
  EEPROM.get(EE_REF_ADDR, g_refMag);

  // nuevos campos
  double iqg = 1.0;
  double iqp = 0.0;
  EEPROM.get(EE_IQGAIN_ADDR,  iqg);
  EEPROM.get(EE_IQPHASE_ADDR, iqp);

  if (iqg >= 0.50 && iqg <= 1.50) g_iqGain  = iqg;
  else g_iqGain = 1.0;

  if (iqp >= -0.50 && iqp <= 0.50) g_iqPhase = iqp;
  else g_iqPhase = 0.0;
}

inline void ee_mark_dirty() {
  ee_dirty  = true;
  ee_lastMs = millis();
}

inline void ee_maybe_commit() {
  if (!ee_dirty) return;
  if ((millis() - ee_lastMs) < EE_DEBOUNCE_MS) return;
  EEPROM.commit();
  ee_dirty = false;
}

// ===== setters existentes =====
inline void ee_set_lo(uint32_t lo) {
  if (lo == g_loHz) return;
  g_loHz = lo;
  EEPROM.put(EE_LO_ADDR, g_loHz);
  ee_mark_dirty();
}

inline void ee_set_ref(uint32_t ref) {
  if (ref == g_refMag) return;
  g_refMag = ref;
  EEPROM.put(EE_REF_ADDR, g_refMag);
  ee_mark_dirty();
}

// ===== NUEVOS setters IQ =====
inline void ee_set_iq_gain(double v) {
  if (fabs(v - g_iqGain) < 0.0001) return;
  g_iqGain = v;
  EEPROM.put(EE_IQGAIN_ADDR, g_iqGain);
  ee_mark_dirty();
}

inline void ee_set_iq_phase(double v) {
  if (fabs(v - g_iqPhase) < 0.0001) return;
  g_iqPhase = v;
  EEPROM.put(EE_IQPHASE_ADDR, g_iqPhase);
  ee_mark_dirty();
}

#endif // STORAGE_H
