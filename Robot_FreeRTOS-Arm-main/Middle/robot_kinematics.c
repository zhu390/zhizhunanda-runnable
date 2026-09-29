/**
	******************************************************************************
	* @file    robot_kinematics.c
	* @brief   机械臂运动学算法实现文件
	*          包含逆运动学求解、角度映射、结果有效性判断与最优解选择等功能
	******************************************************************************
	*/

#include "robot_kinematics.h"
#include "bsp_uart.h"
#include "robot.h"
#include <float.h>
#include <math.h>
#include <string.h>


static struct robot_kinematics g_robot_kinematics = {0};
static float g_current_joint_angle[ROBOT_MAX_JOINT_NUM] = {0};

/* IK 分支锁定（移植自 MechanicalArm Inverse_Kinematics::SelectBestSolution）：
 * 记录上一帧选中的解索引，对非上次解施加惩罚，避免路径经过对称分支边界时关节角突变。 */
static int g_last_best_index = -1;
/* 选解权重：近基座关节权重大（与参考工程 DEFAULT_WEIGHTS 一致） */
static const float IK_SELECT_WEIGHTS[ROBOT_MAX_JOINT_NUM] = {20.0f, 15.0f, 12.0f, 5.0f, 3.0f, 1.0f};
/* 切换分支惩罚系数，>1 表示不鼓励切换 */
#define IK_SWITCH_PENALTY   (10.0f)

static inline float __robot_sqrf(float x)
{
	return x * x;
}

static inline float __robot_pow4f(float x)
{
	float x2 = x * x;
	return x2 * x2;
}

static void robot_kinematics_matrix_multiply(const float a[4][4],
                                             const float b[4][4],
                                             float out[4][4])
{
	float result[4][4] = {0};

	for (uint8_t row = 0u; row < 4u; row++) {
		for (uint8_t col = 0u; col < 4u; col++) {
			for (uint8_t k = 0u; k < 4u; k++) {
				result[row][col] += a[row][k] * b[k][col];
			}
		}
	}

	memcpy(out, result, sizeof(result));
}

static inline uint32_t __robot_all_invalid_mask(void)
{
	return (uint32_t)((1u << ROBOT_KINEMATICS_RESULT_NUM) - 1u);
}

/* 机械臂各关节DH参数 */
#define a2 D_H[2][0]
#define a3 D_H[3][0]
#define d4 D_H[3][2]

/* 各关节运动学逆解公式可参考robot_kinematics_sym_v3_0.m */

static void robot_kinematics_calc_theta3(void)
{
	float px = g_robot_kinematics.T[0][3];
	float py = g_robot_kinematics.T[1][3];
	float pz = g_robot_kinematics.T[2][3];

	float _2_a2_d4 = 2.0f * a2 * d4;
	float _2_pow_a2_2 = 2.0f * __robot_sqrf(a2);
	float _2_pow_a3_2 = 2.0f * __robot_sqrf(a3);
	float _2_pow_d4_2 = 2.0f * __robot_sqrf(d4);
	float const_eq1 = -__robot_pow4f(a2) + _2_pow_a2_2 * (__robot_sqrf(a3) + __robot_sqrf(d4))
				- __robot_pow4f(a3) - 2.0f * __robot_sqrf(a3) * __robot_sqrf(d4) - __robot_pow4f(d4);
	float const_eq2 = -__robot_sqrf(a2) + 2.0f * a2 * a3 - __robot_sqrf(a3) - __robot_sqrf(d4);
	
	float pow_px_2 = __robot_sqrf(px);
	float pow_py_2 = __robot_sqrf(py);
	float pow_pz_2 = __robot_sqrf(pz);
	float pow_distance_2 = pow_px_2 + pow_py_2 + pow_pz_2;

	float eq1 = (const_eq1 + _2_pow_a2_2 * pow_distance_2
		+ _2_pow_a3_2 * pow_distance_2 + _2_pow_d4_2 * pow_distance_2
		- __robot_pow4f(px) - __robot_pow4f(py) - __robot_pow4f(pz)
		- 2.0f * pow_px_2 * (pow_py_2 + pow_pz_2) - 2.0f * pow_py_2 * pow_pz_2);

	/* 数值误差下允许很小的负数，超过阈值则判不可达 */
	if (eq1 < -1e-6f)
	{
		LOG("theta3计算错误\n");
		g_robot_kinematics.result_invalid_mask = __robot_all_invalid_mask();
		return;
	}
	if (eq1 < 0.0f)
	{
		eq1 = 0.0f;
	}

	float denom = (const_eq2 + pow_distance_2);
	if (fabsf(denom) < 1e-6f)
	{
		LOG("theta3计算错误: denom=0\n");
		g_robot_kinematics.result_invalid_mask = __robot_all_invalid_mask();
		return;
	}

	float sqrt_eq1 = sqrtf(eq1);
	float u_theta3_1 = -(_2_a2_d4 + sqrt_eq1) / denom;
	float u_theta3_2 = -(_2_a2_d4 - sqrt_eq1) / denom;

	float theta3_1 = atanf(u_theta3_1) * 2.0f;
	float theta3_2 = atanf(u_theta3_2) * 2.0f;

	// 前4个theta3是同解，后4个theta3是同解
	for (unsigned int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM/2; i++) {
		g_robot_kinematics.result[i][ROBOT_JOINT_3] = theta3_1;
	}

	for (unsigned int i = ROBOT_KINEMATICS_RESULT_NUM/2; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		g_robot_kinematics.result[i][ROBOT_JOINT_3] = theta3_2;
	}
}

static bool __robot_kinematics_calc_theta2(float theta3, float *theta2_1, float *theta2_2)
{
	float pz = g_robot_kinematics.T[2][3];

	float _pow_a2_2 = __robot_sqrf(a2);
	float _pow_a3_2 = __robot_sqrf(a3);
	float _pow_d4_2 = __robot_sqrf(d4);
	float _2_a2_a3 = 2.0f * a2 * a3;
	float _2_a2_d4 = 2.0f * a2 * d4;
	
	float cos_theta3 = cosf(theta3);
	float sin_theta3 = sinf(theta3);

	float const_eq1 = _pow_a2_2 + _pow_a3_2 + _pow_d4_2;
	float sqrt_arg = const_eq1 + _2_a2_a3 * cos_theta3 - _2_a2_d4 * sin_theta3 - __robot_sqrf(pz);
	if (sqrt_arg < -1e-6f)
	{
		return false;
	}
	if (sqrt_arg < 0.0f)
	{
		sqrt_arg = 0.0f;
	}
	float eq1 = sqrtf(sqrt_arg);
	float eq2 = a3 * cos_theta3 - d4 * sin_theta3;
	float eq3 = (d4 * cos_theta3 - pz + a3 * sin_theta3);

	if (fabsf(eq3) < 1e-6f)
	{
		return false;
	}

	float u_theta2_1 = -(a2 + eq1 + eq2) / eq3;
	float u_theta2_2 = -(a2 - eq1 + eq2) / eq3;

	*theta2_1 = atanf(u_theta2_1) * 2.0f;
	*theta2_2 = atanf(u_theta2_2) * 2.0f;
	return true;
}

static void robot_kinematics_calc_theta2(void)
{
	// 前4个theta3是同解，后4个theta3是同解
	float theta3 = 0.0f;
	float theta2_1 = 0;
	float theta2_2 = 0;

	theta3 = g_robot_kinematics.result[0][ROBOT_JOINT_3];
	if (__robot_kinematics_calc_theta2(theta3, &theta2_1, &theta2_2)) {
		g_robot_kinematics.result[0][ROBOT_JOINT_2] = theta2_1;
		g_robot_kinematics.result[1][ROBOT_JOINT_2] = theta2_2;
	} else {
		g_robot_kinematics.result_invalid_mask |= 0x03u;
	}

	theta3 = g_robot_kinematics.result[2][ROBOT_JOINT_3];
	if (__robot_kinematics_calc_theta2(theta3, &theta2_1, &theta2_2)) {
		g_robot_kinematics.result[2][ROBOT_JOINT_2] = theta2_1;
		g_robot_kinematics.result[3][ROBOT_JOINT_2] = theta2_2;
	} else {
		g_robot_kinematics.result_invalid_mask |= 0x0Cu;
	}
}

static bool __robot_kinematics_calc_theta1(float theta2, float theta3, float *theta1)
{	
	float px = g_robot_kinematics.T[0][3];
	float py = g_robot_kinematics.T[1][3];

	float diff_theta2_3 = theta2 - theta3;
	float cos_diff_theta2_3 = cosf(diff_theta2_3);
	float sin_diff_theta2_3 = sinf(diff_theta2_3);
	float cos_theta2 = cosf(theta2);
	float sin_theta2 = sinf(theta2);
	float cos_theta3 = cosf(theta3);
	float sin_theta3 = sinf(theta3);

	float eq1 = a2 * cos_theta2 + a3 * cos_diff_theta2_3 + d4 * sin_diff_theta2_3;
	float denom = (px + eq1);
	if (fabsf(denom) < 1e-6f)
	{
		return false;
	}
	float ratio = (-px + eq1) / denom;
	if (ratio < -1e-6f)
	{
		return false;
	}
	if (ratio < 0.0f)
	{
		ratio = 0.0f;
	}
	float u_theta1 = sqrtf(ratio);

	float eq2 = (2.0f * u_theta1 * (cos_theta2 * (a2 + a3 * cos_theta3 - d4 * sin_theta3)
							+ sin_theta2 * (d4 * cos_theta3 + a3 * sin_theta3)))
			 / (__robot_sqrf(u_theta1) + 1.0f);
	
	// u_theta1需满足(eq2 == py)
	if (fabsf(py - eq2) > ROBOT_ERROR_RANGE) {
		u_theta1 = -u_theta1;
	}

	*theta1 = atanf(u_theta1) * 2.0f;

	return true;
}

static void robot_kinematics_calc_theta1(void)
{
	for (unsigned int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		if ((g_robot_kinematics.result_invalid_mask & (1u << i)) != 0u) {
			continue;
		}
		float theta2 = g_robot_kinematics.result[i][ROBOT_JOINT_2];
		float theta3 = g_robot_kinematics.result[i][ROBOT_JOINT_3];
		float theta1 = 0.0f;
		if (__robot_kinematics_calc_theta1(theta2, theta3, &theta1)) {
			g_robot_kinematics.result[i][ROBOT_JOINT_1] = theta1;
		} else {
			g_robot_kinematics.result_invalid_mask |= (1u << i);
		}
	}
}

static float __robot_kinematics_calc_theta5(float theta1, float theta2, float theta3)
{
	float nx = g_robot_kinematics.T[0][0];
	float ny = g_robot_kinematics.T[1][0];
	float nz = g_robot_kinematics.T[2][0];

	float ox = g_robot_kinematics.T[0][1];
	float oy = g_robot_kinematics.T[1][1];
	float oz = g_robot_kinematics.T[2][1];

	float ax = g_robot_kinematics.T[0][2];
	float ay = g_robot_kinematics.T[1][2];
	float az = g_robot_kinematics.T[2][2];

	float cos_theta1 = cosf(theta1);
	float sin_theta1 = sinf(theta1);
	float cos_theta2 = cosf(theta2);
	float sin_theta2 = sinf(theta2);
	float cos_theta3 = cosf(theta3);
	float sin_theta3 = sinf(theta3);

	float r31 = nx*cos_theta1*cos_theta3*sin_theta2 - nz*sin_theta2*sin_theta3 - nx*cos_theta1*cos_theta2*sin_theta3 - nz*cos_theta2*cos_theta3 - ny*cos_theta2*sin_theta1*sin_theta3 + ny*cos_theta3*sin_theta1*sin_theta2;
	float r32 = ox*cos_theta1*cos_theta3*sin_theta2 - oz*sin_theta2*sin_theta3 - ox*cos_theta1*cos_theta2*sin_theta3 - oz*cos_theta2*cos_theta3 - oy*cos_theta2*sin_theta1*sin_theta3 + oy*cos_theta3*sin_theta1*sin_theta2;
    float r33 = ax*cos_theta1*cos_theta3*sin_theta2 - az*sin_theta2*sin_theta3 - ax*cos_theta1*cos_theta2*sin_theta3 - az*cos_theta2*cos_theta3 - ay*cos_theta2*sin_theta1*sin_theta3 + ay*cos_theta3*sin_theta1*sin_theta2;
	float theta5_zyz = atan2f(sqrtf(__robot_sqrf(r31) + __robot_sqrf(r32)), r33);
	float theta5 = -theta5_zyz + (float)M_PI;
	return theta5;
}

static void robot_kinematics_calc_theta5(void)
{
	for (unsigned int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		if ((g_robot_kinematics.result_invalid_mask & (1u << i)) != 0u) {
			continue;
		}
		float theta2 = g_robot_kinematics.result[i][ROBOT_JOINT_2];
		float theta3 = g_robot_kinematics.result[i][ROBOT_JOINT_3];
		float theta1 = g_robot_kinematics.result[i][ROBOT_JOINT_1];
		g_robot_kinematics.result[i][ROBOT_JOINT_5] = __robot_kinematics_calc_theta5(theta1, theta2, theta3);
	}
}

static float __robot_kinematics_calc_theta4(float theta1, float theta2, float theta3, float theta5)
{
	float theta5_zyz = (float)M_PI - theta5;
	if ((fabsf(theta5_zyz) < ROBOT_ERROR_RANGE) || (fabsf(theta5_zyz - (float)M_PI) < ROBOT_ERROR_RANGE)) {
		return 0;
	}
	
	float ax = g_robot_kinematics.T[0][2];
	float ay = g_robot_kinematics.T[1][2];
	float az = g_robot_kinematics.T[2][2];

	float theta4 = 0; // 用于坐标系对齐
	float cos_theta1 = cosf(theta1);
	float sin_theta1 = sinf(theta1);
	float cos_theta2 = cosf(theta2);
	float sin_theta2 = sinf(theta2);
	float cos_theta3 = cosf(theta3);
	float sin_theta3 = sinf(theta3);
	float cos_theta4 = cosf(theta4);
	float sin_theta4 = sinf(theta4);

	float r23 = ax*cos_theta4*sin_theta1 - ay*cos_theta1*cos_theta4 + az*cos_theta2*sin_theta3*sin_theta4 
	- az*cos_theta3*sin_theta2*sin_theta4 - ax*cos_theta1*cos_theta2*cos_theta3*sin_theta4 - ay*cos_theta2*cos_theta3*sin_theta1*sin_theta4 
	- ax*cos_theta1*sin_theta2*sin_theta3*sin_theta4 - ay*sin_theta1*sin_theta2*sin_theta3*sin_theta4;
	
	float r13 = ax*sin_theta1*sin_theta4 - ay*cos_theta1*sin_theta4 
	- az*cos_theta2*cos_theta4*sin_theta3 + az*cos_theta3*cos_theta4*sin_theta2 
	+ ax*cos_theta1*cos_theta2*cos_theta3*cos_theta4 + ay*cos_theta2*cos_theta3*cos_theta4*sin_theta1 
	+ ax*cos_theta1*cos_theta4*sin_theta2*sin_theta3 + ay*cos_theta4*sin_theta1*sin_theta2*sin_theta3;

	theta4 = atan2f(r23, r13);
	return theta4;
}

static void robot_kinematics_calc_theta4(void)
{
	for (unsigned int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		if ((g_robot_kinematics.result_invalid_mask & (1u << i)) != 0u) {
			continue;
		}
		float theta2 = g_robot_kinematics.result[i][ROBOT_JOINT_2];
		float theta3 = g_robot_kinematics.result[i][ROBOT_JOINT_3];
		float theta1 = g_robot_kinematics.result[i][ROBOT_JOINT_1];
		float theta5 = g_robot_kinematics.result[i][ROBOT_JOINT_5];
		g_robot_kinematics.result[i][ROBOT_JOINT_4] = __robot_kinematics_calc_theta4(theta1, theta2, theta3, theta5);
	}
}

static float __robot_kinematics_calc_theta6(float theta1, float theta2, float theta3, float theta4, float theta5)
{
	(void) theta4;
	float theta5_zyz = (float)M_PI - theta5;
	
	float nx = g_robot_kinematics.T[0][0];
	float ny = g_robot_kinematics.T[1][0];
	float nz = g_robot_kinematics.T[2][0];

	float ox = g_robot_kinematics.T[0][1];
	float oy = g_robot_kinematics.T[1][1];
	float oz = g_robot_kinematics.T[2][1];

	float cos_theta1 = cosf(theta1);
	float sin_theta1 = sinf(theta1);
	float cos_theta2 = cosf(theta2);
	float sin_theta2 = sinf(theta2);
	float cos_theta3 = cosf(theta3);
	float sin_theta3 = sinf(theta3);
	float theta6_zyz = 0;
	float theta6 = 0;
	if ((fabsf(theta5_zyz) < ROBOT_ERROR_RANGE) || (fabsf(theta5_zyz - (float)M_PI) < ROBOT_ERROR_RANGE)) {
		float r12 = -oz*cos_theta2*sin_theta3 + oz*cos_theta3*sin_theta2 + ox*cos_theta1*cos_theta2*cos_theta3+ oy*cos_theta2*cos_theta3*sin_theta1 + ox*cos_theta1*sin_theta2*sin_theta3 + oy*sin_theta1*sin_theta2*sin_theta3;
		float r11 = -nz*cos_theta2*sin_theta3 + nz*cos_theta3*sin_theta2 + nx*cos_theta1*cos_theta2*cos_theta3+ ny*cos_theta2*cos_theta3*sin_theta1 + nx*cos_theta1*sin_theta2*sin_theta3 + ny*sin_theta1*sin_theta2*sin_theta3;
		theta6_zyz = atan2f(-r12, r11);
		theta6 = theta6_zyz - (float)M_PI;
		return theta6;
	}

	float r32 = ox*cos_theta1*cos_theta3*sin_theta2 - oz*sin_theta2*sin_theta3 - ox*cos_theta1*cos_theta2*sin_theta3 - oz*cos_theta2*cos_theta3 - oy*cos_theta2*sin_theta1*sin_theta3 + oy*cos_theta3*sin_theta1*sin_theta2;
	float r31 = nx*cos_theta1*cos_theta3*sin_theta2 - nz*sin_theta2*sin_theta3 - nx*cos_theta1*cos_theta2*sin_theta3 - nz*cos_theta2*cos_theta3 - ny*cos_theta2*sin_theta1*sin_theta3 + ny*cos_theta3*sin_theta1*sin_theta2;
	theta6_zyz = atan2f(r32, -r31);
	theta6 = theta6_zyz - (float)M_PI;
	return theta6;
}

static void robot_kinematics_calc_theta6(void)
{
	for (unsigned int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		if ((g_robot_kinematics.result_invalid_mask & (1u << i)) != 0u) {
			continue;
		}
		float theta1 = g_robot_kinematics.result[i][ROBOT_JOINT_1];
		float theta2 = g_robot_kinematics.result[i][ROBOT_JOINT_2];
		float theta3 = g_robot_kinematics.result[i][ROBOT_JOINT_3];
		float theta4 = g_robot_kinematics.result[i][ROBOT_JOINT_4];
		float theta5 = g_robot_kinematics.result[i][ROBOT_JOINT_5];
		g_robot_kinematics.result[i][ROBOT_JOINT_6] = __robot_kinematics_calc_theta6(theta1, theta2, theta3, theta4, theta5);
	}
}

/**
 * @brief 计算机械臂各关节的角度。
 * 
 * 该函数按照关节求解的依赖顺序依次计算每个关节的角度，
 * 最后将结果无效掩码清零，表示初始状态下所有结果均有效。
 */
static void robot_kinematics_calc(void)
{
	/* 每次解算都清空结果，避免某一步失败时残留旧值导致“天文数字” */
	memset(g_robot_kinematics.result, 0, sizeof(g_robot_kinematics.result));
	g_robot_kinematics.result_invalid_mask = 0;

	robot_kinematics_calc_theta3();
	if (g_robot_kinematics.result_invalid_mask == __robot_all_invalid_mask()) return;

	robot_kinematics_calc_theta2();
	robot_kinematics_calc_theta1();
	robot_kinematics_calc_theta5();
	robot_kinematics_calc_theta4();
	robot_kinematics_calc_theta6();
}

void robot_kinematics_show_result(void)
{
	for (unsigned int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		LOG("result[%d]: ", i);
		for (unsigned int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
			LOG("%.2f ", g_robot_kinematics.result[i][j]);
		}
		int valid = g_robot_kinematics.result_invalid_mask & (1 << i) ? 0 : 1;
		LOG(" valid:%d\n", valid);
	}
}

// 弧度转 0 - 360 度范围内的角度
static float radians_to_degrees_0_360(float radians) {
    // 先将弧度转换为角度
    float degrees = radians * (180.0f / (float)M_PI);
    // 确保角度在 0 - 360 度范围内
    degrees = fmodf(degrees, 360.0f);
    if (degrees < 0) {
        degrees += 360.0f;
    }
	
	// 处理接近 360 度的情况，将 360 度转换为 0 度
    if (fabsf(degrees - 360.0f) < ROBOT_ERROR_RANGE) {
        degrees = 0.0f;
    }
    return degrees;
}

/**
 * @brief 将机械臂运动学计算结果中的关节角度从弧度制转换为角度制，并将角度映射到 0 - 360 度范围。
 * 
 * 该函数遍历机械臂运动学计算得到的所有解，针对每个解中的每个关节角度，
 * 调用 `radians_to_degrees_0_360` 函数将其从弧度制转换为 0 - 360 度范围内的角度。
 */
static void robot_kinematics_radians_to_degrees(void)
{
	for (unsigned int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		for (unsigned int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
			g_robot_kinematics.result[i][j] = radians_to_degrees_0_360(g_robot_kinematics.result[i][j]);	
		}
	}
}

/**
 * @brief 从机械臂运动学计算结果中获取最优解。
 * 
 * 该函数会遍历所有有效的运动学计算结果，计算每个结果与当前关节位置的加权差值，
 * 选择差值最小的结果作为最优解，并将其关节角度存储到传入的结果数组中。
 * 
 * @param result 指向用于存储最优解关节角度的数组的指针。
 * @return int 若成功找到最优解返回 0，若没有有效的解则返回 -1。
 */
static int robot_kinematics_get_optimal_result(volatile float *result)
{
	/* 选择最接近当前关节位置的解：加权平方距离 + 分支锁定惩罚（移植自 MechanicalArm） */
	float min_dist = FLT_MAX;
	int best_index = -1;
	for (unsigned int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		if (g_robot_kinematics.result_invalid_mask & (1u << i)) {
			continue;	// 跳过无效解
		}
		float dist = 0.0f;
		for (unsigned int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
			float diff = g_robot_kinematics.result[i][j] - g_current_joint_angle[j];
			/* 角度差归一化到 [-180, 180]，避免 ±360 等价性干扰距离计算 */
			while (diff > 180.0f)  diff -= 360.0f;
			while (diff < -180.0f) diff += 360.0f;
			dist += IK_SELECT_WEIGHTS[j] * (diff * diff);
		}

		/* 分支锁定：上次选中的解仍有效时，对其它候选施加惩罚 */
		if ((g_last_best_index != -1)
			&& !(g_robot_kinematics.result_invalid_mask & (1u << (unsigned int)g_last_best_index))
			&& ((int)i != g_last_best_index)) {
			dist *= IK_SWITCH_PENALTY;
		}

		if (dist < min_dist) {
			min_dist = dist;
			best_index = (int)i;
		}
	}

	if (best_index == -1) {
		return -1;
	}

	g_last_best_index = best_index;

	for (unsigned int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
		result[j] = g_robot_kinematics.result[best_index][j];
	}

	return 0;
}

/* 复位 IK 分支锁定：在每段新运动开始前调用，使首帧从实际姿态最近解起步，
 * 不被上一段运动锁定的分支带偏。 */
void robot_kinematics_reset_branch_lock(void)
{
	g_last_best_index = -1;
}

/**
 * @brief 将机械臂运动学计算结果中的关节角度映射到关节限制范围内，并标记无效解。
 * 
 * 该函数遍历机械臂运动学计算得到的所有解，针对每个解中的每个关节角度，
 * 尝试将其映射到该关节允许的最小和最大角度范围内。若无法映射到有效范围，
 * 则将该解标记为无效。
 */
static void robot_kinematics_joint_angle_map(void)
{
	for (unsigned int i = 0; i < ROBOT_KINEMATICS_RESULT_NUM; i++) {
		for (unsigned int j = 0; j < ROBOT_MAX_JOINT_NUM; j++) {
			float angle = g_robot_kinematics.result[i][j];
			float min_angle = g_robot.joints[j].min_angle;
			float max_angle = g_robot.joints[j].max_angle;

			/* 逆解计算过程中如果产生 NaN/Inf，直接判该解无效，避免后续映射/最优选择异常 */
			if (!isfinite(angle)) {
				g_robot_kinematics.result_invalid_mask |= (1u << i);
				continue;
			}

			// 临界区做整数处理，防止浮点误差导致角度超出限制
			if (fabsf(angle - min_angle) < ROBOT_ERROR_RANGE) {
				angle = min_angle;		
			}

			if (fabsf(angle - max_angle) < ROBOT_ERROR_RANGE) {
				angle = max_angle;	
			}

			// 尝试映射到关节限制范围内
			if (angle < min_angle) {
				angle += 360.0f;
			} else if (angle > max_angle) {
				angle -= 360.0f;
			}
			
			// 检查映射后的角度是否在限制范围内
			if ((angle < min_angle) || (angle > max_angle)) {
				g_robot_kinematics.result_invalid_mask |= (1 << i);	// 标记解为无效
			}
			g_robot_kinematics.result[i][j] = angle;
		}	
	}	
}

void robot_kinematics_joint_angle_update(volatile float *joint_angle)
{
    for (unsigned int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
        g_current_joint_angle[i] = joint_angle[i];
    }
}

void robot_kinematics_joint_angle_update_by_id(uint32_t joint_id ,float angle)
{
	if (joint_id >= ROBOT_MAX_JOINT_NUM) {
		LOG("robot kinematics joint id is invalid\n");
		return;	
	}
	g_current_joint_angle[joint_id] = angle;
}

int robot_kinematics_forward(const float joint_angle[ROBOT_MAX_JOINT_NUM],
                             float T_out[4][4])
{
	float total[4][4] = {
		{1.0f, 0.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f, 0.0f},
		{0.0f, 0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 0.0f, 1.0f},
	};

	if ((joint_angle == NULL) || (T_out == NULL)) {
		return -1;
	}

	for (uint8_t i = 0u; i < ROBOT_MAX_JOINT_NUM; i++) {
		/* J6 is the gripper. Keep its fixed frame rotation, but exclude its motion and offset. */
		float theta_deg = (i == ROBOT_JOINT_6) ? 0.0f : joint_angle[i];
		float theta;
		float a;
		float alpha;
		float d;
		float ct;
		float st;
		float ca;
		float sa;

		if (!isfinite(theta_deg)) {
			return -1;
		}

		theta = theta_deg * ((float)M_PI / 180.0f);
		a = D_H[i][0];
		alpha = D_H[i][1];
		d = (i == ROBOT_JOINT_6) ? 0.0f : D_H[i][2];
		ct = cosf(theta);
		st = sinf(theta);
		ca = cosf(alpha);
		sa = sinf(alpha);

		float link[4][4] = {
			{ct,      -st,       0.0f, a},
			{st * ca,  ct * ca, -sa,  -d * sa},
			{st * sa,  ct * sa,  ca,   d * ca},
			{0.0f,     0.0f,     0.0f, 1.0f},
		};
		float next[4][4] = {0};

		robot_kinematics_matrix_multiply(total, link, next);
		memcpy(total, next, sizeof(total));
	}

	for (uint8_t row = 0u; row < 4u; row++) {
		for (uint8_t col = 0u; col < 4u; col++) {
			if (!isfinite(total[row][col])) {
				return -1;
			}
		}
	}

	memcpy(T_out, total, sizeof(total));
	return 0;
}

/**
 * @brief 计算机械臂运动学逆解，获取达到目标位姿所需的关节角度。
 * 
 * 该函数接收目标位姿的变换矩阵，计算机械臂各关节的角度，将结果转换为角度制，
 * 映射到关节限制范围内，最后选择最优解返回。同时支持根据参数控制输出调试信息。
 * 
 * @param T_target 指向目标位姿变换矩阵的指针，矩阵按行优先存储。
 * @param result 指向用于存储最优关节角度结果的数组的指针。
 * @param show 控制是否显示调试信息的标志，非零值表示显示，零值表示不显示。
 * @return int 若成功找到最优解返回 0，否则返回 -1。
 */
int robot_kinematics_inverse(float *T_target, volatile float *result, int show)
{
	memcpy(g_robot_kinematics.T, T_target, sizeof(float)*16);

	robot_kinematics_calc();
	if (show) {
		LOG("target:[%f %f %f]\n", T_target[4 * 0 + 3], T_target[4 * 1 + 3], T_target[4 * 2 + 3]);
		LOG("robot kinematics result(rad):\n");
		robot_kinematics_show_result();
	}

	robot_kinematics_radians_to_degrees();

	robot_kinematics_joint_angle_map();
	if (show) {
		LOG("robot kinematics result(deg):\n");
		robot_kinematics_show_result();
	}

	int ret = robot_kinematics_get_optimal_result(result);
	if (show) {
		LOG("robot optimal result(rad):\n");
		for (unsigned int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
			LOG("%.2f ", result[i]);	
		}
		LOG("\n");
	}
	return ret;
}

/**
 * @brief 根据输入的变换矩阵和相对位置，计算新的变换矩阵。
 * 
 * 该函数将输入的变换矩阵复制到输出矩阵，然后根据传入的相对位置信息
 * 对输出矩阵的平移部分（即矩阵的第 4 列的前 3 个元素）进行更新。
 * 
 * @param T_in 输入的 4x4 变换矩阵，按行优先存储。
 * @param T_out 输出的 4x4 变换矩阵，用于存储计算后的结果，按行优先存储。
 * @param pos 指向 `position` 结构体的指针，包含相对位置的 x、y、z 坐标。
 */
void robot_kinematics_cal_T(const float T_in[4][4], float T_out[4][4], struct position *pos)
{
	// 根据末端坐标的相对运动，获得末端的T矩阵(当前仅支持x,y,z的相对运动, 后续支持旋转)
	memcpy(T_out, T_in, sizeof(float)*16);
	T_out[0][3] = pos->x + T_out[0][3];
	T_out[1][3] = pos->y + T_out[1][3];
	T_out[2][3] = pos->z + T_out[2][3];
}

void robot_kinematics_show_T(float T[4][4])
{
	for (unsigned int i = 0; i < 4; i++) {
		for (unsigned int j = 0; j < 4; j++) {
			LOG("%.2f ", T[i][j]);
		}
		LOG("\n");
	}
}
