#include "attitude_kalman_extended.h"
#include "quaternion2imu.h"
#include "stdio.h"
#include "stdlib.h"

static float32_t cov_meas[36] =    {0.1, 0, 0, 0, 0, 0,
									0, 0.1, 0, 0, 0, 0,
									0, 0, 0.1, 0, 0, 0,
									0, 0, 0, 0.1, 0, 0,
									0, 0, 0, 0, 0.1, 0,
									0, 0, 0, 0, 0, 0.1};
// variables to compute covariance matrix for gyroscope
static const float32_t gyro_noise = 0.000000001;

static float32_t cov_gyro_quat[16] ={0, 0,     0,     0,
									 0,    0,  0,     0,
									 0,    0,     0,  0,
									 0,    0,     0,     0};
static arm_matrix_instance_f32 cov_gyro_quat_inst;

static float32_t gain_numerator[24];
static float32_t cov_pred[36];

static arm_matrix_instance_f32 gain_numerator_inst;
static arm_matrix_instance_f32 cov_pred_inst;
static arm_matrix_instance_f32 cov_meas_inst;

//variables to compute the gain and covariance update
static float32_t gain_matrix[24];
static float32_t meas_pred_diff[6];
static float32_t quat_correct[4];
static float32_t kh[16];

static arm_matrix_instance_f32 gain_matrix_inst;
static arm_matrix_instance_f32 meas_pred_diff_inst;
static arm_matrix_instance_f32 quat_correct_inst;
static arm_matrix_instance_f32 kh_inst;


// variables to compute the covariance matrix of predictions
static float32_t jh[24];
static float32_t jh_transpose[24];

static arm_matrix_instance_f32 jh_inst;
static arm_matrix_instance_f32 jh_tran_inst;

static void covariance_gyro_extended(quaternion q_old, imu_norm imu_data);
static void covariance_imu_predicted(quaternion qa, float32_t mag_hor, float32_t mag_ver);
static void covariance_update(arm_matrix_instance_f32 H,
		arm_matrix_instance_f32 covB, arm_matrix_instance_f32 covA);

void attitude_init_extended()
{
	arm_mat_init_f32(&cov_gyro_quat_inst,     4, 4, cov_gyro_quat);
	arm_mat_init_f32(&gain_numerator_inst,    4, 6, gain_numerator);
	arm_mat_init_f32(&cov_pred_inst,          6, 6, cov_pred);
	arm_mat_init_f32(&cov_meas_inst,          6, 6, cov_meas);
	arm_mat_init_f32(&gain_matrix_inst,       4, 6, gain_matrix);
	arm_mat_init_f32(&meas_pred_diff_inst,    6, 1, meas_pred_diff);
	arm_mat_init_f32(&quat_correct_inst,      4, 1, quat_correct);
	arm_mat_init_f32(&kh_inst,      		  4, 4, kh);
	arm_mat_init_f32(&jh_inst,          6, 4, jh);
	arm_mat_init_f32(&jh_tran_inst,     4, 6, jh_transpose);
}

void run_kalman_extended(imu_norm imu, quaternion *qout)
{
	arm_status arm_status_temp;
	static uint8_t run_first = 0;
	quaternion qaccmag, qacc, qmag;
	static quaternion qgyro;
	imu_norm imu_pred;
	static float32_t lx, ly, mag_hor, mag_ver;
	// if running first time, we use only acc&mag for quaternion computation
	if(run_first == 0)
	{
		run_first = 1;
		accmag2quat(&qaccmag, &qacc, &qmag, imu, &lx, &ly);
		qgyro = qaccmag;
		arm_status_temp = arm_sqrt_f32(lx *lx + ly * ly, &mag_hor);
		if(arm_status_temp != ARM_MATH_SUCCESS)
		{
		//	printf("error sqrt! %d \n", arm_status_temp);
			while(1);
		}
		arm_status_temp = arm_sqrt_f32(1 - mag_hor * mag_hor, &mag_ver);
		if(arm_status_temp != ARM_MATH_SUCCESS)
		{
		//	printf("error sqrt! %d \n", arm_status_temp);
			while(1);
		}
	//	printf("mag %f, %f, %f \n", imu.mx, imu.my, imu.mz);
		//printf("mag ver and hor %f and %f \n", mag_ver, mag_hor);
	}
	else
	{
		// quaternion prediction using the gyroscope and covariance computation
		covariance_gyro_extended(qgyro, imu);
		quat_gyro(&qgyro, imu);

		// predict the measurements and compute the covariance of the prediction
		quater2imu(qgyro, &imu_pred, mag_hor, mag_ver);
		covariance_imu_predicted(qgyro, mag_hor, mag_ver);

		// Gain computation, denominator:
		arm_status_temp = arm_mat_add_f32(&cov_pred_inst,  &cov_meas_inst,
						&cov_pred_inst);
		if(arm_status_temp != ARM_MATH_SUCCESS)
		{
		//	printf("error matrix addition %d \n", arm_status_temp);
			while(1);
		}
		// inverse of the denominator
		arm_status_temp = arm_mat_inverse_f32(&cov_pred_inst, &cov_pred_inst);
		if(arm_status_temp != ARM_MATH_SUCCESS)
		{
		//	printf("error matrix inverse kalman%d \n", arm_status_temp);
			while(1);
		}

		// numerator of the gain
		arm_status_temp = arm_mat_mult_f32(&cov_gyro_quat_inst,
						&jh_tran_inst, &gain_numerator_inst);
		if(arm_status_temp != ARM_MATH_SUCCESS)
		{
		//	printf("error matrix mult gyro cov! %d \n", arm_status_temp);
			while(1);
		}

		arm_status_temp = arm_mat_mult_f32(&gain_numerator_inst,
				&cov_pred_inst, &gain_matrix_inst);
		if(arm_status_temp != ARM_MATH_SUCCESS)
		{
		//	printf("error matrix mult gyro cov! %d \n", arm_status_temp);
			while(1);
		}

		// Prediction correction
		*(meas_pred_diff_inst.pData) = imu.ax - imu_pred.ax;
		*(meas_pred_diff_inst.pData + 1) = imu.ay - imu_pred.ay;
		*(meas_pred_diff_inst.pData + 2) = imu.az - imu_pred.az;
		*(meas_pred_diff_inst.pData + 3) = imu.mx - imu_pred.mx;
		*(meas_pred_diff_inst.pData + 4) = imu.my - imu_pred.my;
		*(meas_pred_diff_inst.pData + 5) = imu.mz - imu_pred.mz;

		arm_status_temp = arm_mat_mult_f32(&gain_matrix_inst,
						&meas_pred_diff_inst, &quat_correct_inst);
		if(arm_status_temp != ARM_MATH_SUCCESS)
		{
		//	printf("error matrix multiplcation! %d \n", arm_status_temp);
			while(1);
		}
		qgyro.s += *(quat_correct_inst.pData);
		qgyro.x += *(quat_correct_inst.pData + 1);
		qgyro.y += *(quat_correct_inst.pData + 2);
		qgyro.z += *(quat_correct_inst.pData + 3);

		quat_norm(&qgyro);

		// prediction covariance update
		arm_status_temp = arm_mat_mult_f32(&gain_matrix_inst,
						&jh_inst, &kh_inst);
		if(arm_status_temp != ARM_MATH_SUCCESS)
		{
		//	printf("error matrix mult! %d \n", arm_status_temp);
			while(1);
		}
		arm_status_temp = arm_mat_mult_f32(&kh_inst,
						&cov_gyro_quat_inst, &kh_inst);
		if(arm_status_temp != ARM_MATH_SUCCESS)
		{
		//	printf("error matrix mult! %d \n", arm_status_temp);
			while(1);
		}

		arm_status_temp = arm_mat_sub_f32(&cov_gyro_quat_inst,
						&kh_inst, &cov_gyro_quat_inst);
		if(arm_status_temp != ARM_MATH_SUCCESS)
		{
			printf("error matrix mult! %d \n", arm_status_temp);
			while(1);
		}
	}
	*qout = qgyro;
}

static void covariance_gyro_extended(quaternion q_old, imu_norm imu_data)
{
	arm_status arm_status_temp;
	static float32_t cov_gyro[9] = {gyro_noise, 0, 0,
									0, gyro_noise, 0,
									0, 0,  gyro_noise};
	static float32_t gyro2quat_cov[16];
	static float32_t gyro2quat_cov2[12];
	static float32_t cov_gyro_quat_temp[16];

	static arm_matrix_instance_f32 cov_gyro_inst;
	static arm_matrix_instance_f32 gyro2quat_cov_inst;
	static arm_matrix_instance_f32 gyro2quat_cov2_inst;
	static arm_matrix_instance_f32 cov_gyro_quat_temp_inst;

	arm_mat_init_f32(&cov_gyro_inst,  	          3, 3, cov_gyro);
	arm_mat_init_f32(&gyro2quat_cov_inst,  	      4, 4, gyro2quat_cov);
	arm_mat_init_f32(&gyro2quat_cov2_inst,  	  4, 3,  gyro2quat_cov2);
	arm_mat_init_f32(&cov_gyro_quat_temp_inst,    4, 4, cov_gyro_quat_temp);

	gyro2quat_cov[0] = 1;
	gyro2quat_cov[1] = -imu_data.gx / 2;
	gyro2quat_cov[2] = -imu_data.gy / 2;
	gyro2quat_cov[3] = -imu_data.gz / 2;

	gyro2quat_cov[4] = imu_data.gx / 2;
	gyro2quat_cov[5] = 1;
	gyro2quat_cov[6] = imu_data.gz / 2;
	gyro2quat_cov[7] = -imu_data.gy / 2;

	gyro2quat_cov[8] = imu_data.gy / 2;
	gyro2quat_cov[9] = -imu_data.gz / 2;
	gyro2quat_cov[10] = 1;
	gyro2quat_cov[11] = imu_data.gx / 2;

	gyro2quat_cov[12] = imu_data.gz / 2;
	gyro2quat_cov[13] = imu_data.gy / 2;
	gyro2quat_cov[14] = -imu_data.gx / 2;
	gyro2quat_cov[15] = 1;

	covariance_update(gyro2quat_cov_inst, cov_gyro_quat_inst, cov_gyro_quat_inst);
	//printf("nur wos colums %d and %d \n", cov_gyro_quat_inst.numCols, cov_gyro_quat_inst.numRows);
	//print_matrix(cov_gyro_quat_inst);
	gyro2quat_cov2[0] = -q_old.x  ;
	gyro2quat_cov2[1] = -q_old.y   ;
	gyro2quat_cov2[2] = -q_old.z   ;

	gyro2quat_cov2[3] = q_old.s ;
	gyro2quat_cov2[4] = -q_old.z  ;
	gyro2quat_cov2[5] = q_old.y ;

	gyro2quat_cov2[6] =  q_old.z ;
	gyro2quat_cov2[7] =  q_old.s ;
	gyro2quat_cov2[8] =  -q_old.x ;

	gyro2quat_cov2[9]  =  -q_old.y ;
	gyro2quat_cov2[10] =  q_old.x;
	gyro2quat_cov2[11] =  q_old.s ;

	 covariance_update(gyro2quat_cov2_inst, cov_gyro_inst, cov_gyro_quat_temp_inst);

	arm_status_temp = arm_mat_add_f32(&cov_gyro_quat_inst,
			&cov_gyro_quat_temp_inst,  &cov_gyro_quat_inst);
	if(arm_status_temp != ARM_MATH_SUCCESS)
	{
	//	printf("error matrix addition %d \n", arm_status_temp);
		while(1);
	}
}

static void covariance_imu_predicted(quaternion qa, float32_t mag_hor, float32_t mag_ver)
{
	arm_status arm_status_temp;

	*(jh) = -2 * qa.y;
	*(jh + 1) = 2 * qa.z;
	*(jh + 2) = -2 * qa.s;
	*(jh + 3) = 2 * qa.x;

	*(jh + 4) = 2 * qa.x;
	*(jh + 5) = 2 * qa.s;
	*(jh + 6) = 2 * qa.z;
	*(jh + 7) = 2 * qa.y;

	*(jh + 8) = 2 * qa.s;
	*(jh + 9) = -2 * qa.x;
	*(jh + 10) = -2 * qa.y;
	*(jh + 11) = 2 * qa.z;

	*(jh + 12) = -2 * qa.y * mag_ver + 2 * qa.s * mag_hor;
	*(jh + 13) =  2 * qa.z * mag_ver + 2 * qa.x * mag_hor;
	*(jh + 14) = -2 * qa.s * mag_ver - 2 * qa.y * mag_hor;
	*(jh + 15) =  2 * qa.x * mag_ver - 2 * qa.z * mag_hor;

	*(jh + 16) = 2 * qa.x * mag_ver - 2 * qa.z * mag_hor;
	*(jh + 17) = 2 * qa.s * mag_ver + 2 * qa.y * mag_hor;
	*(jh + 18) = 2 * qa.z * mag_ver + 2 * qa.x * mag_hor;
	*(jh + 19) = 2 * qa.y * mag_ver - 2 * qa.s * mag_hor;

	*(jh + 20) =  2 * qa.s * mag_ver + 2 * qa.y * mag_hor;
	*(jh + 21) = -2 * qa.x * mag_ver + 2 * qa.z * mag_hor;
	*(jh + 22) = -2 * qa.y * mag_ver + 2 * qa.s * mag_hor;
	*(jh + 23) =  2 * qa.z * mag_ver + 2 * qa.x * mag_hor;

	arm_status_temp =  arm_mat_trans_f32(&jh_inst, &jh_tran_inst);
	if(arm_status_temp != ARM_MATH_SUCCESS)
	{
	//printf("error transpose! %d \n", arm_status_temp);
		while(1);
	}
	covariance_update(jh_inst, cov_gyro_quat_inst, cov_pred_inst);
}

/* A = HB
 * COV(A) = H COV(B) H^(-1)
 *
 * */
static void covariance_update(arm_matrix_instance_f32 H,
		arm_matrix_instance_f32 covB, arm_matrix_instance_f32 covA)
{
	// defining variables for transpose of the matrix
	float32_t *H_trans = (float32_t* )malloc(4 * H.numCols * H.numRows);
	arm_matrix_instance_f32 H_trans_inst;

	// defining variables to store the first multiplication result
	float32_t *mat_first_mult = (float32_t* )malloc(4 * H.numCols * H.numRows);
	arm_matrix_instance_f32 mat_first_mult_inst;

	// initialization of matrix inst and transpose:
	arm_mat_init_f32(&H_trans_inst, H.numCols,	H.numRows, H_trans);
	arm_mat_trans_f32(&H, &H_trans_inst);
	arm_mat_init_f32(&mat_first_mult_inst, H.numRows, H.numCols, mat_first_mult);

	// multiplication operations
	arm_mat_mult_f32(&H, &covB, &mat_first_mult_inst);
	arm_mat_mult_f32(&mat_first_mult_inst, &H_trans_inst, &covA);
	//print_matrix(covA);
	// Freeing memory
	free(H_trans);
	free(mat_first_mult);
}

void print_matrix(arm_matrix_instance_f32 matrix)
{
	for(int i = 0; i< matrix.numRows;i++)
	{
		for(int j = 0; j < matrix.numCols; j++)
		{
		//	printf("%f ", *(matrix.pData + i * matrix.numRows + j));
		}
		//printf("\n");
	}
}
