/*
 * clarke.c
 *
 *  Created on: Oct 20, 2025
 *      Author: JavierMunozSaez
 */
#include "clarke_and_park.h"


/* ---------- Constants for Clarke ---------- */
static inline float _two_thirds(void) 	{ return 0.6666666666666666f; }//{ return 2.0f / 3.0f; }
static inline float _one_third(void)  	{ return 0.3333333333333333f; }//{ return 1.0f / 3.0f; }
static inline float _sqrt3_over_2(void) { return 0.8660254037844386f; } // √3/2

/* ========== Clarke ========== */
/* Forward Clarke: abc -> αβ0 (power-invariant) */
void clarke_forward(const struct_abc_pu *in_abc,
                                  struct_alphabeta_pu *out_ab0)
{
    const float a = in_abc->a;
    const float b = in_abc->b;
    const float c = in_abc->c;

    const float k23 = _two_thirds();
    const float k  = _sqrt3_over_2();

    out_ab0->alpha = k23 * (a - 0.5f*b - 0.5f*c);
    out_ab0->beta  = k23 * (k * (b - c));
    out_ab0->zero  = _one_third() * (a + b + c);
}

/* Inverse Clarke: αβ0 -> abc (power-invariant) */
void clarke_inverse(const struct_alphabeta_pu *in_ab0,
                                  struct_abc_pu *out_abc)
{
    const float a0 = in_ab0->alpha;
    const float b0 = in_ab0->beta;
    const float z0 = in_ab0->zero;
    const float k  = _sqrt3_over_2();

    out_abc->a = a0 + z0;
    out_abc->b = (-0.5f * a0) + ( k * b0) + z0;
    out_abc->c = (-0.5f * a0) + (-k * b0) + z0;
}

/* ========== Park ========== */
/* Forward Park: αβ0 -> dq0  (θ is electrical angle; pass in cosθ and sinθ) */
void park_forward(const struct_alphabeta_pu *in_ab0,
                                float cos_theta, float sin_theta,
                                struct_dq0_pu *out_dq0)
{
    const float a = in_ab0->alpha;
    const float b = in_ab0->beta;

    out_dq0->d    =  cos_theta * a + sin_theta * b;
    out_dq0->q    = -sin_theta * a + cos_theta * b;
    out_dq0->zero = in_ab0->zero;
}

/* Inverse Park: dq0 -> αβ0 */
void park_inverse(const struct_dq0_pu *in_dq0,
                                float cos_theta, float sin_theta,
                                struct_alphabeta_pu *out_ab0)
{
    const float d = in_dq0->d;
    const float q = in_dq0->q;

    out_ab0->alpha =  cos_theta * d - sin_theta * q;
    out_ab0->beta  =  sin_theta * d + cos_theta * q;
    out_ab0->zero  = in_dq0->zero;
}
