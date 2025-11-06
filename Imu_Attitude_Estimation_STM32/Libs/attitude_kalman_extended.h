#ifndef INC_ATTITUDE_KALMAN_EXTENDED_H_
#define INC_ATTITUDE_KALMAN_EXTENDED_H_


#include "imu2quaternion.h"
#include "imu_data.h"
#include "quaternion.h"

void attitude_init_extended(void);
void run_kalman_extended(imu_norm imu, quaternion *qout);
void print_matrix(arm_matrix_instance_f32 matrix);
#endif /* INC_ATTITUDE_KALMAN_EXTENDED_H_ */
