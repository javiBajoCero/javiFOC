/*
 * clarke.h
 *
 *  Created on: Oct 20, 2025
 *      Author: JavierMunozSaez
 */

#ifndef CLARKE_AND_PARK_H_
#define CLARKE_AND_PARK_H_

typedef struct {
	float d;
	float q;
	float zero;
}struct_dq0_pu;

typedef struct {
	float alpha;
	float beta;
	float zero;
}struct_alphabeta_pu;

typedef struct {
	float a;
	float b;
	float c;
}struct_abc_pu;


/* --- Function Prototypes --- */

/**
 * @brief Forward Clarke transform: converts 3-phase (abc) to αβ0.
 * @param[in]  in_abc   Pointer to input abc structure.
 * @param[out] out_ab0  Pointer to output αβ0 structure.
 */
void clarke_forward(const struct_abc_pu *in_abc,
                    struct_alphabeta_pu *out_ab0);

/**
 * @brief Inverse Clarke transform: converts αβ0 to 3-phase (abc).
 * @param[in]  in_ab0   Pointer to input αβ0 structure.
 * @param[out] out_abc  Pointer to output abc structure.
 */
void clarke_inverse(const struct_alphabeta_pu *in_ab0,
                    struct_abc_pu *out_abc);

/**
 * @brief Forward Park transform: converts αβ0 to dq0 reference frame.
 * @param[in]  in_ab0     Pointer to input αβ0 structure.
 * @param[in]  cos_theta  Cosine of electrical angle.
 * @param[in]  sin_theta  Sine of electrical angle.
 * @param[out] out_dq0    Pointer to output dq0 structure.
 */
void park_forward(const struct_alphabeta_pu *in_ab0,
                  float cos_theta, float sin_theta,
                  struct_dq0_pu *out_dq0);

/**
 * @brief Inverse Park transform: converts dq0 back to αβ0.
 * @param[in]  in_dq0     Pointer to input dq0 structure.
 * @param[in]  cos_theta  Cosine of electrical angle.
 * @param[in]  sin_theta  Sine of electrical angle.
 * @param[out] out_ab0    Pointer to output αβ0 structure.
 */
void park_inverse(const struct_dq0_pu *in_dq0,
                  float cos_theta, float sin_theta,
                  struct_alphabeta_pu *out_ab0);

#endif /* CLARKE_AND_PARK_H_ */
