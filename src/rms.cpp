#include "rms.h"
#include <cmath>

// ============================================================
// Single-pass RMS with automatic DC removal (P1-7)
// ============================================================
// var(x) = E[x^2] - E[x]^2  ==  mean of (x - mean)^2,
// computed in one pass without storing samples on the device.
// sqrt(var) is the RMS of the AC component only, so a constant
// offset (ADC midpoint bias) contributes nothing.
// ============================================================

float zmptRmsFromSamples(const uint16_t* samples, size_t n,
                         float calibration, float zero_threshold) {
    if (n == 0) {
        return 0.0f;
    }

    uint64_t sum = 0;
    uint64_t sumsq = 0;
    for (size_t i = 0; i < n; i++) {
        uint64_t r = samples[i];
        sum   += r;
        sumsq += r * r;
    }

    double mean = (double)sum / (double)n;
    double var  = (double)sumsq / (double)n - mean * mean;
    if (var < 0.0) {
        var = 0.0;  // guard against tiny negative rounding error
    }

    float v = (float)(sqrt(var) * 3.3 / 4095.0) * calibration;
    return (v < zero_threshold) ? 0.0f : v;
}
