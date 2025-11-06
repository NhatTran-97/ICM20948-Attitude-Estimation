/**
  ******************************************************************************
  * @file    quaternion.c
  * @brief   Source file of quaternion functions.
  ******************************************************************************
  ******************************************************************************
  */

#include "quaternion.h"
#include "imu_data.h"
#include "arm_math.h"
#include "stdio.h"

/**
 * @brief This function copmutes the multiplication of quaternions
 * @param qc output of multiplication
 * @param qa input quaternion
 * @param qb inut quaterinion
 */
void quat_mult(quaternion *qc, quaternion qa, quaternion qb)
{
	qc ->s = qa.s * qb.s - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z;
	qc ->x = qa.s * qb.x + qa.x * qb.s + qa.y * qb.z - qa.z * qb.y;
	qc ->y = qa.s * qb.y - qa.x * qb.z + qa.y * qb.s + qa.z * qb.x;
	qc ->z = qa.s * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.s;
}


/**
 * @brief Quaternion inversion
 * @param qa input quaternion
 * @param qb inverse of quaternion
 */
void quat_inverse(quaternion qa, quaternion* qb)
{
	float32_t norm_square = qa.s * qa.s + qa.x * qa.x +
			qa.y * qa.y + qa.z * qa.z;
	qb -> s = qa.s / norm_square;
	qb -> x = -qa.x / norm_square;
	qb -> y = -qa.y / norm_square;
	qb -> z = -qa.z / norm_square;
}

/**
 * @brief Quaternion normalizarion
 * @param qa quaternion to be normalized
 */
void quat_norm(quaternion* qa)
{
	float32_t qa_norm;
	arm_status arm_status_temp;

	arm_status_temp = arm_sqrt_f32(qa->s * qa->s + qa->x * qa->x + qa->y * qa->y + qa->z * qa->z, &qa_norm);
	if(arm_status_temp != ARM_MATH_SUCCESS)
	{
	//	printf("error sqrt! %d \n", arm_status_temp);
		while(1);
	}
	qa->s = qa->s / qa_norm;
	qa->x = qa->x / qa_norm;
	qa->y = qa->y / qa_norm;
	qa->z = qa->z / qa_norm;
}

void quat_subtr(quaternion *qc, quaternion qa, quaternion qb)
{
	qc -> s = qa.s - qb.s;
	qc -> x = qa.x - qb.x;
	qc -> y = qa.y - qb.y;
	qc -> z = qa.z - qb.z;
}

float32_t quat_dot( quaternion qa, quaternion qb)
{
	float32_t result;
	result = (qa.s * qb.s) + (qa.x * qb.x) +
			(qa.y * qb.y) + (qa.z * qb.z);
	return result;
}

void quat_add(quaternion *qc, quaternion qa, quaternion qb)
{
	qc -> s = qa.s + qb.s;
	qc -> x = qa.x + qb.x;
	qc -> y = qa.y + qb.y;
	qc -> z = qa.z + qb.z;
}

/**
 * @brief This function rotates vector using a given quaternion.
 * @param rot quaternion defines rotation.
 * @param in input vector
 * @param out vector after rotation
 */
void vector_rotate(quaternion rot, quaternion in, quaternion *out)
{
	out -> s = 0;
	out -> x =  in.x * (rot.s * rot.s + rot.x * rot.x - rot.y * rot.y - rot.z * rot.z) +
			   2 * in.y * (rot.x * rot.y - rot.s * rot.z) +
			   2 * in.z * (rot.x * rot.z + rot.s * rot.y);

	out -> y = 2 * in.x * (rot.x * rot.y + rot.s * rot.z) +
			   in.y * (rot.s * rot.s - rot.x * rot.x + rot.y * rot.y - rot.z * rot.z) +
			   2 * in.z * (rot.y * rot.z - rot.s * rot.x);

	out -> z = 2 * in.x * (rot.x * rot.z - rot.s * rot.y) +
			   2 * in.y * (rot.y * rot.z + rot.s * rot.x) +
			   in.z * (rot.s * rot.s - rot.x * rot.x - rot.y * rot.y + rot.z * rot.z);
}
