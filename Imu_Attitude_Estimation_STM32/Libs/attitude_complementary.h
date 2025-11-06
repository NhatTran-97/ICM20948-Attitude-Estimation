#ifndef INC_ATTITUDE_COMPLEMENTARY_H_
#define INC_ATTITUDE_COMPLEMENTARY_H_

#include "imu2quaternion.h"
#include "imu_data.h"
#include "quaternion.h"
#include "arm_math.h"
#include "stdio.h"

#define COMPLEMENTARY_GAIN  0.999f
void run_quaternion_complementary(imu_norm imu_norm_data, quaternion *qout);
#endif /* INC_ATTITUDE_COMPLEMENTARY_H_ */
