#include "robot_capture.h"
#include "robot.h"
#include "jetson_vision.h"
#include "bsp_uart.h"
#include "FreeRTOS.h"
#include <string.h>

#define CAPTURE_STEP_TIMEOUT_MS  15000u
#define CAPTURE_JOINT_TIMEOUT_MS 5000u
#define CAPTURE_RESET_TIMEOUT_MS 45000u

typedef enum
{
    CAPTURE_IDLE = 0,
    CAPTURE_START_STEP,
    CAPTURE_WAIT_AUTO,
    CAPTURE_WAIT_JOINT,
    CAPTURE_WAIT_RESET,
} capture_state_t;

typedef enum
{
    CAPTURE_STEP_AUTO = 0,
    CAPTURE_STEP_ABS,
    CAPTURE_STEP_RESET,
    CAPTURE_STEP_DONE,
} capture_step_type_t;

typedef struct
{
    capture_step_type_t type;
    uint8_t joint_id;
    float value;
} capture_step_t;

typedef struct
{
    capture_state_t state;
    uint8_t action;
    uint8_t point_id;
    uint8_t step_index;
    uint32_t step_start_ms;
    bool base_pose_ready;
} capture_ctx_t;

static const capture_step_t s_left_steps[] =
{
    { CAPTURE_STEP_AUTO, 0u, 0.0f },
    { CAPTURE_STEP_ABS,  0u, 70.0f },
    { CAPTURE_STEP_ABS,  3u, 330.0f },
    { CAPTURE_STEP_ABS,  4u, 86.0f },
    { CAPTURE_STEP_DONE, 0u, 0.0f },
};

static const capture_step_t s_front_steps[] =
{
    { CAPTURE_STEP_AUTO, 0u, 0.0f },
    { CAPTURE_STEP_ABS,  0u, 90.0f },
    { CAPTURE_STEP_ABS,  3u, 0.0f },
    { CAPTURE_STEP_ABS,  4u, 83.0f },
    { CAPTURE_STEP_DONE, 0u, 0.0f },
};

static const capture_step_t s_right_steps[] =
{
    { CAPTURE_STEP_AUTO, 0u, 0.0f },
    { CAPTURE_STEP_ABS,  0u, 115.0f },
    { CAPTURE_STEP_ABS,  3u, 35.0f },
    { CAPTURE_STEP_ABS,  4u, 85.0f },
    { CAPTURE_STEP_DONE, 0u, 0.0f },
};

static const capture_step_t s_home_steps[] =
{
    { CAPTURE_STEP_RESET, 0u, 0.0f },
    { CAPTURE_STEP_DONE,  0u, 0.0f },
};

static capture_ctx_t s_capture;
static robot_capture_result_t s_result = ROBOT_CAPTURE_RESULT_NONE;
static uint8_t s_result_action = 0u;
static uint8_t s_result_point_id = 0u;

static bool capture_action_valid(uint8_t action, uint8_t point_id)
{
    if ((action == JETSON_CAPTURE_ACTION_GOTO) ||
        (action == JETSON_CAPTURE_ACTION_SELECT))
    {
        return (point_id >= 1u) && (point_id <= 3u);
    }
    return (action == JETSON_CAPTURE_ACTION_HOME) && (point_id == 0u);
}

static const capture_step_t *capture_steps_for(uint8_t action, uint8_t point_id)
{
    if (action == JETSON_CAPTURE_ACTION_HOME)
    {
        return s_home_steps;
    }

    switch (point_id)
    {
        case 1u: return s_left_steps;
        case 2u: return s_front_steps;
        case 3u: return s_right_steps;
        default: return NULL;
    }
}

static void capture_set_terminal_result(robot_capture_result_t result)
{
    s_result_action = s_capture.action;
    s_result_point_id = s_capture.point_id;
    s_result = result;
    s_capture.state = CAPTURE_IDLE;
    s_capture.action = 0u;
    s_capture.point_id = 0u;
    s_capture.step_index = 0u;
}

static void capture_fail(robot_capture_result_t result)
{
    robot_motion_abort();
    robot_visual_servo_stop();
    s_capture.base_pose_ready = false;
    capture_set_terminal_result(result);
}

static void capture_finish(void)
{
    if (s_capture.action == JETSON_CAPTURE_ACTION_HOME)
    {
        s_capture.base_pose_ready = false;
    }
    capture_set_terminal_result(ROBOT_CAPTURE_RESULT_OK);
}

static void capture_start_current_step(uint32_t now_ms)
{
    const capture_step_t *steps = capture_steps_for(s_capture.action, s_capture.point_id);
    if (steps == NULL)
    {
        capture_fail(ROBOT_CAPTURE_RESULT_FAILED);
        return;
    }

    const capture_step_t *step = &steps[s_capture.step_index];
    s_capture.step_start_ms = now_ms;

    switch (step->type)
    {
        case CAPTURE_STEP_AUTO:
        {
            if (s_capture.base_pose_ready)
            {
                s_capture.step_index++;
                break;
            }

            struct position pos = { .x = 0.0f, .y = -130.0f, .z = -15.0f };
            if (robot_send_auto_event(&pos) == pdPASS)
            {
                s_capture.state = CAPTURE_WAIT_AUTO;
            }
            else
            {
                capture_fail(ROBOT_CAPTURE_RESULT_FAILED);
            }
            break;
        }

        case CAPTURE_STEP_ABS:
            if (robot_send_abs_rotate_event(step->joint_id, step->value) == pdPASS)
            {
                s_capture.state = CAPTURE_WAIT_JOINT;
            }
            else
            {
                capture_fail(ROBOT_CAPTURE_RESULT_FAILED);
            }
            break;

        case CAPTURE_STEP_RESET:
            if (robot_send_reset_event(false) == pdPASS)
            {
                s_capture.state = CAPTURE_WAIT_RESET;
            }
            else
            {
                capture_fail(ROBOT_CAPTURE_RESULT_FAILED);
            }
            break;

        case CAPTURE_STEP_DONE:
            capture_finish();
            break;

        default:
            capture_fail(ROBOT_CAPTURE_RESULT_FAILED);
            break;
    }
}

void robot_capture_init(void)
{
    memset(&s_capture, 0, sizeof(s_capture));
    s_capture.state = CAPTURE_IDLE;
    s_result = ROBOT_CAPTURE_RESULT_NONE;
}

bool robot_capture_request(uint8_t action, uint8_t point_id)
{
    if (!capture_action_valid(action, point_id) ||
        (s_capture.state != CAPTURE_IDLE) ||
        robot_is_auto_busy() || robot_is_visual_servo_active() ||
        robot_motion_abort_latched())
    {
        return false;
    }

    s_capture.action = action;
    s_capture.point_id = point_id;
    s_capture.step_index = 0u;
    s_capture.step_start_ms = 0u;
    s_capture.state = CAPTURE_START_STEP;
    s_result = ROBOT_CAPTURE_RESULT_RUNNING;
    LOG("[CAPTURE] request action=%u point=%u\r\n",
        (unsigned)action, (unsigned)point_id);
    return true;
}

void robot_capture_step(uint32_t now_ms)
{
    if (s_capture.state == CAPTURE_IDLE)
    {
        return;
    }

    switch (s_capture.state)
    {
        case CAPTURE_START_STEP:
            capture_start_current_step(now_ms);
            break;

        case CAPTURE_WAIT_AUTO:
        {
            robot_auto_result_t result = robot_auto_result_consume();
            if (result == ROBOT_AUTO_RESULT_OK)
            {
                s_capture.base_pose_ready = true;
                s_capture.step_index++;
                s_capture.state = CAPTURE_START_STEP;
            }
            else if ((result == ROBOT_AUTO_RESULT_FAILED) ||
                     (result == ROBOT_AUTO_RESULT_ABORTED) ||
                     ((now_ms - s_capture.step_start_ms) > CAPTURE_STEP_TIMEOUT_MS))
            {
                capture_fail((result == ROBOT_AUTO_RESULT_ABORTED) ?
                             ROBOT_CAPTURE_RESULT_ABORTED : ROBOT_CAPTURE_RESULT_FAILED);
            }
            break;
        }

        case CAPTURE_WAIT_JOINT:
        {
            robot_joint_result_t result = robot_joint_result_consume();
            if (result == ROBOT_JOINT_RESULT_OK)
            {
                s_capture.step_index++;
                s_capture.state = CAPTURE_START_STEP;
            }
            else if ((result == ROBOT_JOINT_RESULT_FAILED) ||
                     (result == ROBOT_JOINT_RESULT_ABORTED) ||
                     ((now_ms - s_capture.step_start_ms) > CAPTURE_JOINT_TIMEOUT_MS))
            {
                capture_fail((result == ROBOT_JOINT_RESULT_ABORTED) ?
                             ROBOT_CAPTURE_RESULT_ABORTED : ROBOT_CAPTURE_RESULT_FAILED);
            }
            break;
        }

        case CAPTURE_WAIT_RESET:
        {
            robot_reset_result_t result = robot_reset_result_consume();
            if (result == ROBOT_RESET_RESULT_OK)
            {
                s_capture.step_index++;
                s_capture.state = CAPTURE_START_STEP;
            }
            else if ((result == ROBOT_RESET_RESULT_FAILED) ||
                     (result == ROBOT_RESET_RESULT_ABORTED) ||
                     ((now_ms - s_capture.step_start_ms) > CAPTURE_RESET_TIMEOUT_MS))
            {
                capture_fail((result == ROBOT_RESET_RESULT_ABORTED) ?
                             ROBOT_CAPTURE_RESULT_ABORTED : ROBOT_CAPTURE_RESULT_FAILED);
            }
            break;
        }

        default:
            capture_fail(ROBOT_CAPTURE_RESULT_FAILED);
            break;
    }
}

void robot_capture_cancel(void)
{
    if (s_capture.state != CAPTURE_IDLE)
    {
        capture_fail(ROBOT_CAPTURE_RESULT_ABORTED);
    }
}

bool robot_capture_is_active(void)
{
    return s_capture.state != CAPTURE_IDLE;
}

robot_capture_result_t robot_capture_result_consume(uint8_t *action, uint8_t *point_id)
{
    robot_capture_result_t result = s_result;
    if ((result == ROBOT_CAPTURE_RESULT_NONE) || (result == ROBOT_CAPTURE_RESULT_RUNNING))
    {
        return result;
    }

    if (action != NULL)
    {
        *action = s_result_action;
    }
    if (point_id != NULL)
    {
        *point_id = s_result_point_id;
    }
    s_result = ROBOT_CAPTURE_RESULT_NONE;
    return result;
}
