#pragma once
// ============================================================
// rms.h — ZMPT101B RMS math (pure logic)
// ============================================================
// Host-compilable (no Arduino.h), covered by native tests (T-6).
//
// Revision note P1-7: the old code calibrated a DC offset once over
// ~20 ms (not an integer number of 50 Hz periods) while mains voltage
// was already present, so the mean was biased and the RMS inflated.
// The single-pass mean/variance method below removes the DC
// component mathematically (variance of signal = RMS of its AC
// component), so no offset calibration arrays are needed at all.
// ============================================================

#include <cstdint>
#include <cstddef>

// Computes RMS voltage from raw ADC samples.
//   samples[]        raw ADC codes (12-bit ESP32 ADC: 0..4095)
//   n                number of samples
//   calibration      multiplier from raw voltage RMS to mains volts
//   zero_threshold   readings below this (in volts) are reported as 0
// Returns RMS in volts with the DC component removed.
float zmptRmsFromSamples(const uint16_t* samples, size_t n,
                         float calibration, float zero_threshold);
