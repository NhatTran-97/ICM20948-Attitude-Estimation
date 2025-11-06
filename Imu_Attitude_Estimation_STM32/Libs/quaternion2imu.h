
#ifndef INC_QUATERNION2IMU_H_ 
#define INC_QUATERNION2IMU_H_

#include "imu_data.h"
#include "quaternion.h"


void quater2imu(quaternion qa, imu_norm *imu_data, float32_t mag_hor, float32_t mag_ver);


#endif
