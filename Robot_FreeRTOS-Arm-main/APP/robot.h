#ifndef __ROBOT_H__
#define __ROBOT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 强制重新定义为 float 类型以避免编译器双精度警告 */
#undef M_PI
#define M_PI 3.14159265358979323846f 

#define _USE_MATH_DEFINES
#include <math.h>
#include "FreeRTOS.h"
#include "stdint.h"
#include "queue.h"
#include "string.h"
#include "stdbool.h"
#include "hal_data.h"
#include "task.h"

///* 数学库常量宏定义（兼容性处理） */
//#if !defined(__STRICT_ANSI__) || defined(_POSIX_C_SOURCE) || defined(_POSIX_SOURCE) || defined(_XOPEN_SOURCE) || defined(_GNU_SOURCE) || defined(_BSD_SOURCE) || defined(_USE_MATH_DEFINES)
//#define M_E         2.7182818284590452354f
//#define M_LOG2E     1.4426950408889634074f
//#define M_LOG10E    0.43429448190325182765f
//#define M_LN2       0.69314718055994530942f
//#define M_LN10      2.30258509299404568402f
//#define M_PI        3.14159265358979323846f
//#define M_PI_2      1.57079632679489661923f
//#define M_PI_4      0.78539816339744830962f
//#define M_1_PI      0.31830988618379067154f
//#define M_2_PI      0.63661977236758134308f
//#define M_2_SQRTPI  1.12837916709551257390f
//#define M_SQRT2     1.41421356237309504880f
//#define M_SQRT1_2   0.70710678118654752440f
//#endif

#define ROBOT_MAX_JOINT_NUM                 6       /* 机械臂最大关节数 */
#define ROBOT_MAX_EVENT_NUM                 20      /* 消息队列最大事件数 */

#define ROBOT_MQTT_ENABLE                   0U      /* 使能 MQTT 服务开关 */
#define ESP8266_MQTT_ENABLE                 0U      /* ESP8266 MQTT 使能开关 */



#define ROBOT_CMD_MAX_NUM                   50      /* 命令行缓存最大数量 */
#define ROBOT_CMD_LENGTH                    128     /* 单条命令最大长度 */
#define ROBOT_CMD_QUEUE_TIMEOUT             100     /* 命令队列发送超时时间，单位 ms */

/* 任务栈与优先级配置 */
#define ROBOT_CONTROL_TASK_STACK_SIZE       16384    /* 运动控制任务栈大小 */
/* 使用 FreeRTOS 数字优先级 (数值越大优先级越高) */
#define ROBOT_CONTROL_TASK_PRIORITY         (3)     /* 控制任务优先级（最高） */

#define ROBOT_CMD_SERVICE_STACK_SIZE        2048    /* 指令解析任务栈大小 */
#define ROBOT_CMD_SERVICE_PRIORITY          (2)     /* 指令解析优先级 */

#define ROBOT_MQTT_SYNC_TASK_STACK_SIZE     2048    /* MQTT 同步任务栈大小 */
#define ROBOT_MQTT_SYNC_TASK_PRIORITY       (1)     /* MQTT 同步优先级 */

#define ROBOT_REMOTE_MAX_VELOCITY           (20.0f) /* 远程控制末端最大线速度，单位 mm/s */
#define ROBOT_REMOTE_MAX_RPM                (5.0f)  /* 远程控制末端最大角速度，单位 rpm */
#define ROBOT_REMOTE_TIME_RESOLUTION        (50)    /* 远程控制时间插值分辨率，单位 ms */

#define ROBOT_JOINT_DEFAULT_VELOCITY        (10.0f) /* 关节默认速度 */
#define ROBOT_JOINT_DEFAULT_ACCELERATION    200     /* 关节默认加速度 */

#define ROBOT_INTERPOLATION_TIME_RESOLUTION (10)    /* 插补时间分辨率，单位 ms（与控制周期对齐，S曲线每点=1个控制周期） */
/* S 曲线速度规划参数（参考 MechanicalArm_Code_V4） */
#define SCURVE_VMAX     (180.0f)                              /* 最大速度 mm/s */
#define SCURVE_AMAX     (250.0f)                              /* 最大加速度 mm/s² */
#define SCURVE_OMEGA    (2.0f * SCURVE_AMAX / SCURVE_VMAX)   /* 角频率 rad/s ≈ 3.333 */
#define SCURVE_T_ACCEL  (3.14159265f / SCURVE_OMEGA)          /* 加速段时长 s ≈ 0.9425 */
#define SCURVE_S_ACCEL  (SCURVE_VMAX * SCURVE_T_ACCEL / 2.0f)/* 加速段位移 mm ≈ 141.76 */
#define ROBOT_AUTO_SLOW_Y_THRESHOLD      (-90.0f) /* 大伸展区阈值，进入后降低 AUTO 轨迹速度 */
#define ROBOT_AUTO_SLOW_PROFILE_SCALE    (0.40f)   /* 大伸展 AUTO 速度/加速度缩放系数 */

#define ROBOT_RESET_DEFAULT_ANGLE           360     /* 复位默认搜索角度范围 */
#define ROBOT_RESET_DEFAULT_VELOCITY        (15.0f) /* 复位默认速度，单位 rpm */
#define ROBOT_RESET_DEFAULT_ACCELERATION    100     /* 复位默认加速度 */

/* ROBOT 状态标志位定义 */
#define ROBOT_STATUS_LIMIT_ENABLE           0U      /* 限位开关使能标志 */
#define ROBOT_STATUS_LIMIT_HAPPENED         1U      /* 限位开关触发标志 */
#define ROBOT_STATUS_READY                  2U      /* 机械臂就绪标志 */
#define ROBOT_STATUS_RMODE_ENABLE           3U      /* 远程控制模式使能标志 */
#define ROBOT_STATUS_MQTT_CONNECTED         4U      /* MQTT 连接状态标志 */
#define ROBOT_STATUS_POSE_VALID             5U      /* 当前末端位姿是否可信 */
#define ROBOT_STATUS_POSE_DEGRADED          6U      /* 位姿进入退化模式(反馈异常但可继续作业) */
#define ROBOT_STATUS_AUTO_BUSY              7U      /* ROBOT_AUTO_EVENT 已入队或正在执行 */
#define ROBOT_STATUS_VISUAL_SERVO_ACTIVE    8U      /* visual servo is running */

#define ROBOT_STATUS_MASK(status)           (1u << (uint32_t)(status))
#define ROBOT_STATUS_IS(x, status)          (((x) & ROBOT_STATUS_MASK(status)) != 0u)     /* 判断状态位是否置位 */
#define ROBOT_STATUS_SET(x, status)         ((x) = (uint32_t)((x) | ROBOT_STATUS_MASK(status)))      /* 设置状态位 */
#define ROBOT_STATUS_CLEAR(x, status)       ((x) = (uint32_t)((x) & ~ROBOT_STATUS_MASK(status)))     /* 清除状态位 */

#define ROBOT_CAN_DELAY                     5       /* CAN 发送/接收等待延时，单位 ms */

#define ROBOT_PID_PERIOD                    (10)    /* 控制周期，单位 ms (100Hz) */
#define ROBOT_PID_SETTLE_PERIODS            (10)     /* 末端稳定段：前馈清零后额外执行的周期数(100ms) */
#define ROBOT_FF_OUTPUT_LIMIT               (200.0f)/* 前馈+P 输出限幅，单位 °/s */

#define ROBOT_CAN_TIMEOUT                   (15)    /* CAN 通信超时时间，单位 ms */

#define ROBOT_ERROR_RANGE                   (1e-4f) /* 浮点数计算误差允许范围 */
#define ROBOT_JOINT_ANGLE_ERROR_RANGE       (1e-1f) /* 关节角度控制误差允许范围 */

#define ROBOT_MQTT_SYNC_TIME                (100)   /* MQTT 同步心跳周期，单位 ms */

#define HAL_Delay(ms)    vTaskDelay(pdMS_TO_TICKS(ms))

/* 兼容性辅助函数：适配原 STM32 代码的 HAL 延时函数 */
static inline uint32_t HAL_GetTick(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}


/* 电机转动方向枚举 */
enum motor_dir
{
    MOTOR_DIR_CW = 0,     /* 顺时针 */
    MOTOR_DIR_CCW = 1,    /* 逆时针 */
};

/* 逻辑方向枚举 */
enum dir
{
    DIR_POSITIVE = 1,     /* 正方向 */
    DIR_NEGATIVE = -1,    /* 负方向 */
};

/* 关节结构体 */
struct joint
{
    float current_angle;                  /* 当前角度 (0-360度) */
    enum motor_dir postive_direction;     /* 关节正方向对应的电机物理旋转方向 */
    float reduction_ratio;                /* 减速比 */
    const external_irq_instance_t * p_limit_irq; /* 指向瑞萨 FSP 外部中断实例的指针 */
    float min_angle;                      /* 关节最小软限位角度 */
    float max_angle;                      /* 关节最大软限位角度 */
    enum dir reset_dir;                   /* 复位搜索方向 */
    
    /* 运行时状态变量 (初始化时无需修改) */
    volatile uint32_t status;            /* 关节运行状态标志位 */
    float velocity;                       /* 当前运行速度 */
    float acceleration;                   /* 当前运行加速度 */
};

/* 笛卡尔空间位置坐标 (mm) */
struct position
{
    float x;
    float y;
    float z;
};

/* 笛卡尔空间姿态坐标 (弧度或角度) */
struct rotate
{
    float x;
    float y;
    float z;
};

/* 机器人总控结构体 */
struct robot	
{	
    TaskHandle_t control_handle;          /* 控制任务句柄 */
    TaskHandle_t cmd_service_handle;      /* 指令任务句柄 */
    TaskHandle_t mqtt_sync_task_handle;   /* MQTT 同步任务句柄 */
    TaskHandle_t remote_service_handle;   /* 远程服务任务句柄 */
    float T[4][4];                        /* 正运动学齐次变换矩阵 */
    struct joint joints[ROBOT_MAX_JOINT_NUM]; /* 6 个关节实例 */
    uint32_t status;                      /* 全局状态标志 */
    QueueHandle_t event_queue;            /* 事件消息队列 */
    QueueHandle_t cmd_queue;              /* 命令消息队列 */
    struct position cur_pos;              /* 当前末端位置 */
    struct rotate cur_rot;                /* 当前末端姿态 */
};

/* 远程控制数据结构 */
struct robot_remote_control
{
    float vx;   // 末端 x 轴线速度 (mm/s)
    float vy;   // 末端 y 轴线速度 (mm/s)
    float vz;   // 末端 z 轴线速度 (mm/s)
    float rx;   // 末端绕 x 轴角速度 (rpm)
    float ry;   // 末端绕 y 轴角速度 (rpm)
    float result[ROBOT_MAX_JOINT_NUM];    /* 逆解算出的目标关节角度 */
    int lock;                             /* 互斥锁 */
};

/* 机器人事件类型枚举 */
enum robot_event_type
{
    ROBOT_JOINT_REL_ROTATE = 0,     /* 相对旋转事件：指定关节相对当前位置旋转一定角度 */
    ROBOT_JOINT_ABS_ROTATE,         /* 绝对旋转事件：指定关节旋转到指定绝对角度 */
    ROBOT_LIMIT_SWITCH_EVENT,       /* 限位开关事件：关节触发限位开关时产生 */
    ROBOT_AUTO_EVENT,               /* 自动路径事件：给定目标位姿，自动规划并控制各关节运动 */
    ROBOT_TIME_FUNC_EVENT,          /* 时间函数事件：末端根据时间函数 P(t) 轨迹运动 (如圆/直线) */
    ROBOT_HARD_RESET_EVENT,         /* 硬件复位事件：依靠限位开关将机械臂复位到初始零点 */
    ROBOT_SOFT_RESET_EVENT,         /* 软件复位事件：直接控制各关节回到设定零位 */
    ROBOT_TEST_EVENT,               /* 测试事件：用于验证系统功能的专用事件 */
    ROBOT_REMOTE_CONTROL_EVENT,     /* 远程控制事件：根据上位机/遥控器的速度指令实时控制 */
    ROBOT_VISUAL_SERVO_EVENT,       /* visual servo event for target alignment */
    ROBOT_JOINTS_SYNC_EVENT,        /* 关节同步事件：强制同步所有关节状态 */
    ROBOT_READ_ALL_EVENT,           /* 批量读取 J1~J5 当前角度 */
    ROBOT_VIEW_ARC_EVENT,           /* 独立演示：平滑经过左、正、右三视图 */
};

typedef enum
{
    ROBOT_AUTO_RESULT_NONE = 0,
    ROBOT_AUTO_RESULT_RUNNING,
    ROBOT_AUTO_RESULT_OK,
    ROBOT_AUTO_RESULT_FAILED,
    ROBOT_AUTO_RESULT_ABORTED,
} robot_auto_result_t;

typedef enum
{
    ROBOT_JOINT_RESULT_NONE = 0,
    ROBOT_JOINT_RESULT_RUNNING,
    ROBOT_JOINT_RESULT_OK,
    ROBOT_JOINT_RESULT_FAILED,
    ROBOT_JOINT_RESULT_ABORTED,
} robot_joint_result_t;

typedef enum
{
    ROBOT_RESET_RESULT_NONE = 0,
    ROBOT_RESET_RESULT_RUNNING,
    ROBOT_RESET_RESULT_OK,
    ROBOT_RESET_RESULT_FAILED,
    ROBOT_RESET_RESULT_ABORTED,
} robot_reset_result_t;

typedef enum
{
    ROBOT_VISUAL_SERVO_FAULT_NONE = 0,
    ROBOT_VISUAL_SERVO_FAULT_FEEDBACK,
    ROBOT_VISUAL_SERVO_FAULT_FK,
    ROBOT_VISUAL_SERVO_FAULT_IK,
} robot_visual_servo_fault_t;

/* 关节索引枚举 */
enum
{
    ROBOT_JOINT_1 = 0,
    ROBOT_JOINT_2,
    ROBOT_JOINT_3,
    ROBOT_JOINT_4,
    ROBOT_JOINT_5,
    ROBOT_JOINT_6,
};

/* 命令行来源枚举 */
enum cmd_type
{
    CMD_TYPE_UART1 = 0,   /* 来自串口 */
    CMD_TYPE_USB,         /* USB CDC command port */
    CMD_TYPE_MQTT,        /* 来自 MQTT 网络 */
};

/* 机器人控制事件结构体 */
struct robot_event
{
    enum robot_event_type type; /* 事件类型 */
    uint8_t joint_id;           /* 关节 ID (如适用) */
    uint32_t generation;        /* 安全中止代际：旧代际运动事件不得继续执行 */
    float param[6];             /* 事件参数 (角度、坐标等) */
};

/* 机器人指令结构体 */
struct robot_cmd
{
    enum cmd_type type;         /* 指令来源 */
    char cmd[ROBOT_CMD_LENGTH]; /* 指令字符串内容 */
};

/* 时间函数指针定义：根据时间 t 返回目标位置 pos */
typedef int(*robot_time_func)(uint32_t time_ms, struct position *pos);

/* 外部全局变量声明 */
extern struct robot g_robot;              /* 机器人全局单例 */
extern const float D_H[6][4];             /* DH 参数表 */
extern const float T_0_6_reset[4][4];     /* 复位姿态矩阵 */
extern volatile struct robot_remote_control g_remote_control; /* 远程控制数据 */

/* 函数声明 */
void robot_init(void);
void robot_cmd_service(void *pvParameters);
void robot_cmd_send_from_isr(char *cmd, enum cmd_type type);
int robot_cmd_send(char *cmd, enum cmd_type type);
int robot_send_rel_rotate_event(uint8_t joint_id, float angle);
int robot_send_auto_event(struct position *pos);
int robot_send_auto_event_scaled(struct position *pos, float profile_scale);
int robot_send_reset_event(bool hard_reset);
bool robot_is_soft_reset_done(void);
bool robot_is_hard_reset_done(void);
bool robot_is_auto_busy(void);
robot_auto_result_t robot_auto_result_consume(void);
robot_joint_result_t robot_joint_result_consume(void);
robot_reset_result_t robot_reset_result_consume(void);
int robot_send_abs_rotate_event(uint8_t joint_id, float angle);
int robot_send_remote_event(void);
int robot_visual_servo_start(void);
void robot_visual_servo_stop(void);
void robot_visual_servo_set_velocity(float vx, float vy, float vz);
bool robot_is_visual_servo_active(void);
robot_visual_servo_fault_t robot_visual_servo_fault_get(void);
void robot_visual_servo_fault_clear(void);
void robot_motion_abort(void);
bool robot_motion_abort_latched(void);
int robot_send_time_func_event(float time_limit_ms, float radius_mm);
int robot_send_view_arc_event(float duration_ms);
int robot_send_read_all_event(void);
uint32_t robot_joint_veloccity_to(uint32_t joint_id, float velocity, uint8_t acceleration);

#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_H__ */
