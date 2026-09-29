#include "robot.h"
#include "bsp_uart.h"
#include "string.h"
#include <stdio.h>
#include "robot_cmd.h"
#include "Emm_V5.h"
#include "gripper.h"
#include "bsp_laser.h"
#include "robot_target.h"

/* --- gripper debug commands (UART1) --- */
static int robot_soft_reset_handle(float *param);
static int robot_abs_rotate_handle(float *param);
static int robot_auto_handle(float *param);
static int robot_rel_rotate_handle(float *param);
static int robot_read_all_handle(float *param);
static int robot_gripper_stop_handle(float *param);
static int robot_gripper_open_handle(float *param);
static int robot_gripper_cur_handle(float *param);
static int robot_gripper_grasp_handle(float *param);
static int robot_laser_on_handle(float *param);
static int robot_laser_off_handle(float *param);
static int robot_target_enable_handle(float *param);
static int robot_target_disable_handle(float *param);
static int robot_time_func_handle(float *param);
static int robot_view_arc_handle(float *param);
static int robot_motion_abort_handle(float *param);


void robot_mqtt_handle(struct robot_cmd *cmd)
{
	float param[6] = {0};
	int strlen = 0;
	int type = 0;
	
	LOG("robot mqtt cmd: %s\n", cmd->cmd);

	// 协议格式示例: [MCU][TYPE][ARG0-5]
	int result = sscanf(cmd->cmd, "+MQTTSUBRECV:0,\"arm/change\",%d,[MCU][%d][%f %f %f %f %f %f]", &strlen, &type,
			&param[0], &param[1], &param[2],
			&param[3], &param[4], &param[5]);
	
	if (result < 8) { // 解析失败
		return;
	}
	
	switch (type)
	{
		case ROBOT_JOINT_ABS_ROTATE:
			robot_abs_rotate_handle(param);
			break;
		
		case ROBOT_AUTO_EVENT:
			robot_auto_handle(param);
			break;
		
		case ROBOT_JOINTS_SYNC_EVENT:
			robot_auto_handle(param);
			break;
		
		default:
			break;
	}
}

static int robot_remote_enable_handle(float *param)
{
	robot_soft_reset_handle(param);	/* 复位 */
	ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_RMODE_ENABLE);
	return robot_send_remote_event();
}

static int robot_remote_disable_handle(float *param)
{
	(void)param;
	ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_RMODE_ENABLE);
	robot_soft_reset_handle(param);	/* 复位 */
	return pdPASS;	
}

static int robot_rel_rotate_handle(float *param)
{
	uint8_t joint_id = (uint8_t)param[0];
	return robot_send_rel_rotate_event(joint_id, param[1]);
}

static int robot_abs_rotate_handle(float *param)
{
	uint8_t joint_id = (uint8_t)param[0];
	return robot_send_abs_rotate_event(joint_id, param[1]);
}

static int robot_auto_handle(float *param)
{
	return robot_send_auto_event((struct position *)param);
}

static int robot_time_func_handle(float *param)
{
	float time_ms = (param[0] > 0.0f) ? param[0] : 6000.0f;
	float radius_mm = (param[1] > 0.0f) ? param[1] : 15.0f;
	LOG("time_func %.0f ms radius %.1f mm\r\n", time_ms, radius_mm);
	return robot_send_time_func_event(time_ms, radius_mm);
}

static int robot_view_arc_handle(float *param)
{
    float duration_ms = (param[0] > 0.0f) ? param[0] : 8000.0f;
    LOG("view_arc %.0f ms\r\n", duration_ms);
    return robot_send_view_arc_event(duration_ms);
}

static int robot_motion_abort_handle(float *param)
{
    (void)param;
    robot_motion_abort();
    LOG("motion_abort latched; soft_reset required\r\n");
    return pdPASS;
}

static int robot_hard_reset_handle(float *param)
{
	(void)param;
	return robot_send_reset_event(true);	
}

static int robot_soft_reset_handle(float *param)
{
	(void)param;
	return robot_send_reset_event(false);	
}

static int robot_read_all_handle(float *param)
{
    (void)param;
    return robot_send_read_all_event();
}

static int robot_remote_event_handle(float *param)
{
	if (!ROBOT_STATUS_IS(g_robot.status, ROBOT_STATUS_RMODE_ENABLE)) {
		return pdPASS;
	}

	float vx = -param[0] * ROBOT_REMOTE_MAX_VELOCITY;
	float vy = param[1] * ROBOT_REMOTE_MAX_VELOCITY;
	float vz = (param[4] - param[5]) / 2 * ROBOT_REMOTE_MAX_VELOCITY;
	float rx = -param[3] * ROBOT_REMOTE_MAX_RPM;
	float ry = param[2] * ROBOT_REMOTE_MAX_RPM;
	
	taskENTER_CRITICAL();
	g_remote_control.vx = vx;
	g_remote_control.vy = vy;
	g_remote_control.vz = vz;
	g_remote_control.rx = rx;
	g_remote_control.ry = ry;
	taskEXIT_CRITICAL();

	return pdPASS;
}

static int robot_zero_handle(float *param)
{
	(void)param;
	LOG("robot reset zero.\r\n");
	for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
		Emm_V5_Reset_CurPos_To_Zero((uint8_t)(i + 1));
		vTaskDelay(10);
	}
	LOG("robot reset zero Finish!!!.\r\n");
	return pdPASS;
}

/* ===================== 夹爪联调命令 =====================
 * 串口助手输入示例：
 * - gripper_stop
 * - gripper_open
 * - gripper_cur
 * - gripper_grasp <fruit_type>
 *   例：gripper_grasp 0   (0=草莓,1=番茄,2=葡萄)
 */

static int robot_gripper_stop_handle(float *param)
{
	(void)param;
	gripper_result_t r = gripper_stop(6);
	LOG("gripper_stop r=%d\r\n", (int)r);
	return (r == GRIPPER_OK) ? pdPASS : pdFAIL;
}

static int robot_gripper_open_handle(float *param)
{
	(void)param;
	gripper_result_t r = gripper_open();
	LOG("gripper_open r=%d\r\n", (int)r);
	return (r == GRIPPER_OK) ? pdPASS : pdFAIL;
}

static int robot_gripper_cur_handle(float *param)
{
	(void)param;
	uint16_t ma = 0u;
	gripper_result_t r = gripper_read_phase_current_ma(0, &ma);
	LOG("gripper_current=%u mA r=%d\r\n", (unsigned)ma, (int)r);
	return (r == GRIPPER_OK) ? pdPASS : pdFAIL;
}

static int robot_gripper_grasp_handle(float *param)
{
	gripper_fruit_t type = (gripper_fruit_t)((uint8_t)param[0]);
	gripper_grasp_cfg_t cfg = {0};

	if (!gripper_get_preset(type, &cfg)) {
		LOG("gripper_grasp invalid type=%u\r\n", (unsigned)type);
		return pdFAIL;
	}

	uint16_t touch_ma = 0u;
	gripper_result_t r = gripper_grasp_force(&cfg, &touch_ma);
	LOG("gripper_grasp type=%u limit=%u touch=%u r=%d\r\n",
		(unsigned)type, (unsigned)cfg.current_limit_ma, (unsigned)touch_ma, (int)r);
	return (r == GRIPPER_OK) ? pdPASS : pdFAIL;
}

static int robot_laser_on_handle(float *param)
{
	(void)param;
	BSP_Laser_Off();
	LOG("laser_on rejected: formal workflow safety gate required.\r\n");
	return pdFAIL;
}

static int robot_laser_off_handle(float *param)
{
	(void)param;
	BSP_Laser_Off();
	LOG("laser_off\r\n");
	return pdPASS;
}

static int robot_target_enable_handle(float *param)
{
	(void)param;
	BSP_Laser_Off();
	LOG("target_enable rejected: use formal WORKFLOW_CTRL and TARGET_CTRL.\r\n");
	return pdFAIL;
}

static int robot_target_disable_handle(float *param)
{
	(void)param;
	BSP_Laser_Off();
	robot_target_disable_request();
	LOG("target_disable: target substate held, laser forced off.\r\n");
	return pdPASS;
}

static struct robot_cmd_info robot_uart1_cmd_table[] = {
	/* 远程控制指令 */
	{"remote_event",   robot_remote_event_handle},
	{"remote_enable",  robot_remote_enable_handle},
	{"remote_disable", robot_remote_disable_handle},

	/* 机械臂控制指令 */
	{"abs_rotate",  robot_abs_rotate_handle},
	{"rel_rotate",  robot_rel_rotate_handle},
	{"auto",        robot_auto_handle},
    {"view_arc",   robot_view_arc_handle},
	{"circle",      robot_time_func_handle},
	{"time_func",   robot_time_func_handle},
	{"hard_reset",  robot_hard_reset_handle},
	{"soft_reset",  robot_soft_reset_handle},
    {"motion_abort", robot_motion_abort_handle},
    {"read_all",    robot_read_all_handle},
	{"zero",        robot_zero_handle},

	/* 夹爪控制指令 */
	{"gripper_stop",  robot_gripper_stop_handle},
	{"gripper_open",  robot_gripper_open_handle},
	{"gripper_cur",   robot_gripper_cur_handle},
	{"gripper_grasp", robot_gripper_grasp_handle},

	/* 激光控制指令 */
	{"target_enable",  robot_target_enable_handle},
	{"target_disable", robot_target_disable_handle},
	{"laser_on",       robot_laser_on_handle},
	{"laser_off",      robot_laser_off_handle},
	{NULL, NULL},
};

void robot_uart1_handle(struct robot_cmd *rb_cmd)
{
	float param[6] = {0};
	char *cmd = rb_cmd->cmd;
	int ret;

	// Trim leading whitespace
	while (*cmd == ' ' || *cmd == '\t') {
		cmd++;
	}

	for (int i = 0; robot_uart1_cmd_table[i].event_type != NULL; i++) {
		const char *event = robot_uart1_cmd_table[i].event_type;
		size_t len = strlen(event);
		// Check if the command starts with the event string
		if (strncmp(cmd, event, len) == 0) {
			// Check if it's a full match (followed by space, null terminator, or newline)
			char next_char = cmd[len];
			if (next_char == ' ' || next_char == '\0' || next_char == '\r' || next_char == '\n') {
				
				// Try to parse parameters after the command
				char *param_str = cmd + len;
				sscanf(param_str, "%f %f %f %f %f %f", &param[0], &param[1], &param[2], 
					&param[3], &param[4], &param[5]);

				// Execute command
				LOG("Found handler for '%s', executing...\n", event);
				ret = robot_uart1_cmd_table[i].cmd_func(param);
				if (ret != pdPASS) {
					LOG("[ERROR] event_type:%s failed\n", event);
				}
				return; // Command found and executed, exit function
			}
		}
	}

	LOG("uart cmd parse error: '%s'\n", rb_cmd->cmd);
}

