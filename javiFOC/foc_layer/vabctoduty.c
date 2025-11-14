/*
 * voltagetoduty.c
 *
 *  Created on: Oct 20, 2025
 *      Author: JavierMunozSaez
 */


#include "vabctoduty.h"
#include <stdint.h>
#include <math.h>

// --- Module configuration ---
typedef struct {
    float   vdc_min_pu;            // Below this DC bus (pu), output zeros (fail-safe).
    float   duty_min;              // Minimum |duty| to ensure ADC triggers (0..1).
    float   duty_max;              // Maximum |duty| (headroom under 1.0).
} struct_vabc2duty_config;

static struct_vabc2duty_config s_cfg;

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
     s_cfg.duty_min = 0.02f;
     s_cfg.duty_max = 0.98f;
     s_cfg.vdc_min_pu = 0.10f;
}

/**
 * Convert phase voltage references (per-unit on VDC_MAX base) to normalized PWM duties in [-1..+1].
 * input_vdc_pu: measured Vdc in per-unit (Vdc_real / VDC_MAX)
 */
void run_vabc_to_duty_modulator(float input_vdc_pu,
								struct_abc_pu *input_vabc,
                                struct_duty_cycles_pu *output_duty)
{
    if (!input_vabc || !output_duty) return;

    // Fail-safe: too little bus → no modulation
    if (input_vdc_pu < s_cfg.vdc_min_pu) {
        output_duty->dutyA = 0.0f;
        output_duty->dutyB = 0.0f;
        output_duty->dutyC = 0.0f;
        return;
    }

    //V_phase_pu / Vdc_pu
    float inv_vdc = 1.0f / input_vdc_pu;
    float mA =  input_vabc->a * inv_vdc;
    float mB =  input_vabc->b * inv_vdc;
    float mC =  input_vabc->c * inv_vdc;


	mA = clampf(mA, -s_cfg.duty_max, +s_cfg.duty_max);
	mB = clampf(mB, -s_cfg.duty_max, +s_cfg.duty_max);
	mC = clampf(mC, -s_cfg.duty_max, +s_cfg.duty_max);

    // Enforce minimum magnitude for ADC trigger, but keep exact zero as zero
    mA = apply_min_magnitude(mA, s_cfg.duty_min);
    mB = apply_min_magnitude(mB, s_cfg.duty_min);
    mC = apply_min_magnitude(mC, s_cfg.duty_min);

    // Final clamp to be extra safe
    output_duty->dutyA = clampf(mA, -s_cfg.duty_max, +s_cfg.duty_max);
    output_duty->dutyB = clampf(mB, -s_cfg.duty_max, +s_cfg.duty_max);
    output_duty->dutyC = clampf(mC, -s_cfg.duty_max, +s_cfg.duty_max);
}
