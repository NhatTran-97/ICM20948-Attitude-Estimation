#include "attitude_complementary.h"


extern UART_HandleTypeDef huart2;

void run_quaternion_complementary(imu_norm imu_norm_data, quaternion *qout)
{
	static int counter_temp = 0;
	static uint8_t run_first = 0;
	quaternion qaccmag, qacc, qmag;
	static quaternion qgyro;
	float32_t lx, ly;

	// if running first time, we use only acc&mag for quaternion computation
	if(run_first == 0)
	{
		run_first = 1;
		accmag2quat(&qaccmag, &qacc, &qmag, imu_norm_data, &lx, &ly);
		qgyro = qaccmag;
	}
	else
	{
		accmag2quat(&qaccmag, &qacc, &qmag, imu_norm_data, &lx, &ly);
		quat_gyro(&qgyro, imu_norm_data);
		if (quat_dot(qgyro,qaccmag) < 0 )
		{
			qaccmag.s = -qaccmag.s;
			qaccmag.x = -qaccmag.x;
			qaccmag.y = -qaccmag.y;
			qaccmag.z = -qaccmag.z;
		}
		qgyro.s = COMPLEMENTARY_GAIN * qgyro.s + (1.0f - COMPLEMENTARY_GAIN) * qaccmag.s;
		qgyro.x = COMPLEMENTARY_GAIN * qgyro.x + (1.0f - COMPLEMENTARY_GAIN) * qaccmag.x;
		qgyro.y = COMPLEMENTARY_GAIN * qgyro.y + (1.0f - COMPLEMENTARY_GAIN) * qaccmag.y;
		qgyro.z = COMPLEMENTARY_GAIN * qgyro.z + (1.0f - COMPLEMENTARY_GAIN) * qaccmag.z;
		quat_norm(&qgyro);
	}
	*qout = qgyro;
}
