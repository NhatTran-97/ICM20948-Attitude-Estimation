#ifndef INC_QUATERNION_H_
#define INC_QUATERNION_H_

#include "main.h"
#include "imu_data.h"

typedef struct
{
	float32_t s; // real
	float32_t x; // i term
	float32_t y; // j term
	float32_t z; // k term
}quaternion;

void quat_mult(quaternion *qc, quaternion qa, quaternion q_b);
void quat_subtr(quaternion *qc, quaternion qa, quaternion qb);
void quat_add(quaternion *qc, quaternion qa, quaternion qb);
void quat_norm(quaternion* qa);
void quat_inverse(quaternion qa, quaternion *qb);
void vector_rotate(quaternion rot, quaternion in, quaternion *out);
float32_t quat_dot( quaternion qa, quaternion qb);
#endif /* INC_QUATERNION_H_ */
