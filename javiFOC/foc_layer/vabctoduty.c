/*
 * voltagetoduty.c
 *
 *  Created on: Oct 20, 2025
 *      Author: JavierMunozSaez
 */

#include <stdint.h>
#include <math.h>
#include <vabctoduty.h>

struct_vabc_pu vabc;//input top this block

// --- Module configuration ---
typedef struct {
    float   vdc_min_pu;            // Below this DC bus (pu), output zeros (fail-safe).
    uint8_t enable_third_harmonic; // 0: SPWM, 1: add zero-sequence (THI).
    float   duty_min;              // Minimum |duty| to ensure ADC triggers (0..1).
    float   duty_max;              // Maximum |duty| (headroom under 1.0).
    uint8_t rescale_all;           // 1: scale all phases uniformly if any exceeds duty_max.
} vabc2duty_cfg_t;

static vabc2duty_cfg_t s_cfg = {
    .vdc_min_pu            = 0.05f,  // 5% bus → disable modulation
    .enable_third_harmonic = 0,
    .duty_min              = 0.01f,  // ensure timer/ADC trigger
    .duty_max              = 0.995f, // keep margin below rails
    .rescale_all           = 1
};

static inline float fabsf_fast(float x) { return x < 0.0f ? -x : x; }

// Push magnitude up to >= duty_min (keeping sign), unless it's exactly zero
static inline float apply_min_magnitude(float x, float duty_min)
{
    float ax = fabsf_fast(x);
    if (ax == 0.0f) return 0.0f;           // true zero stays zero
    if (ax >= duty_min) return x;          // already large enough
    return (x > 0.0f) ? duty_min : -duty_min;
}

// Clamp helper
static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

// Public API — tweak defaults if you like (kept no params to match your header)
void configure_vabc_to_duty_modulator(void)
{
    // Example: change behavior at runtime if desired
    // s_cfg.enable_third_harmonic = 1;
    // s_cfg.duty_min = 0.02f;
    // s_cfg.duty_max = 0.98f;
    // s_cfg.rescale_all = 1;
    // s_cfg.vdc_min_pu = 0.04f;
}

/**
 * Convert phase voltage references (per-unit on VDC_MAX base) to normalized PWM duties in [-1..+1].
 * input_vdc_pu: measured Vdc in per-unit (Vdc_real / VDC_MAX)
 */
void run_vabc_to_duty_modulator(float input_vdc_pu,
                                struct_vabc_pu *input_vabc,
                                struct_duty_cycles_pu *output_duty)
{
    if (!input_vabc || !output_duty) return;

    // Fail-safe: too little bus → no modulation
    float vdc_pu = input_vdc_pu >= 0.0f ? input_vdc_pu : -input_vdc_pu;
    if (vdc_pu < s_cfg.vdc_min_pu) {
        output_duty->dutyA = 0.0f;
        output_duty->dutyB = 0.0f;
        output_duty->dutyC = 0.0f;
        return;
    }

    // Copy (per-unit) references
    float va = input_vabc->va;
    float vb = input_vabc->vb;
    float vc = input_vabc->vc;

    // Optional: third-harmonic (zero-sequence) injection
    if (s_cfg.enable_third_harmonic) {
        float vmax = va; if (vb > vmax) vmax = vb; if (vc > vmax) vmax = vc;
        float vmin = va; if (vb < vmin) vmin = vb; if (vc < vmin) vmin = vc;
        float v0 = -0.5f * (vmax + vmin);
        va += v0; vb += v0; vc += v0;
    }

    //V_phase_pu / Vdc_pu
    float inv_vdc = 1.0f / vdc_pu;
    float mA =  va * inv_vdc;
    float mB =  vb * inv_vdc;
    float mC =  vc * inv_vdc;

    // Uniform rescale if any exceeds duty_max (preserves waveform shape)
    if (s_cfg.rescale_all) {
        float aA = fabsf_fast(mA), aB = fabsf_fast(mB), aC = fabsf_fast(mC);
        float maxAbs = aA; if (aB > maxAbs) maxAbs = aB; if (aC > maxAbs) maxAbs = aC;
        if (maxAbs > s_cfg.duty_max) {
            float k = s_cfg.duty_max / maxAbs;
            mA *= k; mB *= k; mC *= k;
        }
    } else {
        mA = clampf(mA, -s_cfg.duty_max, +s_cfg.duty_max);
        mB = clampf(mB, -s_cfg.duty_max, +s_cfg.duty_max);
        mC = clampf(mC, -s_cfg.duty_max, +s_cfg.duty_max);
    }

    // Enforce minimum magnitude for ADC trigger, but keep exact zero as zero
    mA = apply_min_magnitude(mA, s_cfg.duty_min);
    mB = apply_min_magnitude(mB, s_cfg.duty_min);
    mC = apply_min_magnitude(mC, s_cfg.duty_min);

    // Final clamp to be extra safe
    output_duty->dutyA = clampf(mA, -s_cfg.duty_max, +s_cfg.duty_max);
    output_duty->dutyB = clampf(mB, -s_cfg.duty_max, +s_cfg.duty_max);
    output_duty->dutyC = clampf(mC, -s_cfg.duty_max, +s_cfg.duty_max);
}
