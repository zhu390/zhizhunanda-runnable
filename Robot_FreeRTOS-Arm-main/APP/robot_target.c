#include "robot_target.h"
#include "bsp_uart.h"
#include <math.h>
#include <string.h>

typedef struct
{
    robot_target_state_t state;
    robot_target_event_t event;
    uint32_t enter_ms;
    uint32_t last_step_ms;
    uint32_t last_vision_ms;
    uint32_t fire_raw_change_ms;
    int16_t dcx;
    int16_t dcy;
    uint8_t stable_count;
    uint8_t confirm_stable_count;
    bool has_vision;
    bool alignment_confirmed;
    bool ready_sent;
    bool fire_raw;
    bool fire_debounced;
    bool fire_release_seen;
    struct position target;
} target_ctx_t;

volatile bool ROBOT_TARGET_ENABLED = false;
volatile bool ROBOT_TARGET_FIRE_ENABLE = false;

static target_ctx_t s_target;

static float clampf_local(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static const char *target_state_name(robot_target_state_t state)
{
    switch (state)
    {
        case ROBOT_TARGET_STATE_INIT:        return "INIT";
        case ROBOT_TARGET_STATE_WAIT_DETECT: return "WAIT_DETECT";
        case ROBOT_TARGET_STATE_ALIGN:       return "ALIGN";
        case ROBOT_TARGET_STATE_CONFIRM:     return "CONFIRM";
        case ROBOT_TARGET_STATE_OUTPUT:      return "OUTPUT";
        case ROBOT_TARGET_STATE_HOLD:        return "HOLD";
        case ROBOT_TARGET_STATE_FAULT:       return "FAULT";
        default:                             return "UNKNOWN";
    }
}

static void target_stop_visual_servo(void)
{
#if TARGET_USE_VISUAL_SERVO
    robot_visual_servo_stop();
#endif
}

static void target_hold_visual_servo(void)
{
#if TARGET_USE_VISUAL_SERVO
    robot_visual_servo_set_velocity(0.0f, 0.0f, 0.0f);
#endif
}

static void target_set_event(robot_target_event_t event)
{
    if (s_target.event == ROBOT_TARGET_EVENT_NONE)
    {
        s_target.event = event;
    }
}

static void reset_alignment_counts(void)
{
    s_target.stable_count = 0u;
    s_target.confirm_stable_count = 0u;
    s_target.alignment_confirmed = false;
}

static void enter_state(robot_target_state_t state, uint32_t now_ms)
{
    if (s_target.state != state)
    {
        LOG("[TARGET] %s -> %s\r\n",
            target_state_name(s_target.state), target_state_name(state));
        reset_alignment_counts();
    }
    s_target.state = state;
    s_target.enter_ms = now_ms;
}

static bool vision_fresh(uint32_t now_ms)
{
    return s_target.has_vision &&
           ((now_ms - s_target.last_vision_ms) <= TARGET_VISION_VALID_MS);
}

static bool align_in_tolerance(void)
{
    return (fabsf((float)s_target.dcx) <= TARGET_ALIGN_TOL_PX) &&
           (fabsf((float)s_target.dcy) <= TARGET_ALIGN_TOL_PX);
}

static void target_update_fire_button(bool pressed, uint32_t now_ms)
{
    if (pressed != s_target.fire_raw)
    {
        s_target.fire_raw = pressed;
        s_target.fire_raw_change_ms = now_ms;
    }

    if ((now_ms - s_target.fire_raw_change_ms) >= TARGET_FIRE_KEY_DEBOUNCE_MS)
    {
        s_target.fire_debounced = s_target.fire_raw;
        if (!s_target.fire_debounced)
        {
            s_target.fire_release_seen = true;
        }
    }
}

static bool target_fire_allowed(void)
{
    return (s_target.fire_release_seen && s_target.fire_debounced) ||
           ROBOT_TARGET_FIRE_ENABLE;
}

#if TARGET_USE_VISUAL_SERVO
static bool target_start_visual_servo(void)
{
    if (robot_is_visual_servo_active())
    {
        return true;
    }
    return robot_visual_servo_start() == pdPASS;
}

static void target_update_visual_servo_velocity(uint32_t now_ms)
{
    float err_px = fmaxf(fabsf((float)s_target.dcx), fabsf((float)s_target.dcy));
    float speed_limit = (err_px <= TARGET_ALIGN_TOL_PX_COARSE) ?
                        TARGET_VS_FINE_MAX_SPEED_MM_S : TARGET_VS_MAX_SPEED_MM_S;
    float vx = clampf_local((float)s_target.dcx * TARGET_VS_KX_MM_S_PER_PX,
                            -speed_limit, speed_limit);
    float vz = clampf_local((float)s_target.dcy * TARGET_VS_KZ_MM_S_PER_PX,
                            -speed_limit, speed_limit);

    robot_visual_servo_set_velocity(vx, 0.0f, vz);
    s_target.last_step_ms = now_ms;
    LOG("[TARGET] visual servo vx=%.2f vz=%.2f dcx=%d dcy=%d\r\n",
        vx, vz, (int)s_target.dcx, (int)s_target.dcy);
}
#else
static bool target_send_alignment_step(uint32_t now_ms)
{
    if (((now_ms - s_target.last_step_ms) < TARGET_ALIGN_PERIOD_MS) || robot_is_auto_busy())
    {
        return true;
    }

    float err_px = fmaxf(fabsf((float)s_target.dcx), fabsf((float)s_target.dcy));
    float step_limit = (err_px <= TARGET_ALIGN_TOL_PX_COARSE) ?
                       TARGET_MAX_STEP_MM_FINE : TARGET_MAX_STEP_MM;
    float dx = clampf_local((float)s_target.dcx * TARGET_KX_MM_PER_PX,
                            -step_limit, step_limit);
    float dz = clampf_local((float)s_target.dcy * TARGET_KY_MM_PER_PX,
                            -step_limit, step_limit);

    s_target.target = g_robot.cur_pos;
    s_target.target.x += dx;
    s_target.target.z += dz;
    if (robot_send_auto_event(&s_target.target) != pdPASS)
    {
        return false;
    }
    s_target.last_step_ms = now_ms;
    return true;
}
#endif

void robot_target_init(void)
{
    memset(&s_target, 0, sizeof(s_target));
    s_target.state = ROBOT_TARGET_STATE_INIT;
    target_stop_visual_servo();
    ROBOT_TARGET_ENABLED = false;
    ROBOT_TARGET_FIRE_ENABLE = false;
}

bool robot_target_start_at_current_pose(void)
{
    uint32_t now_ms = HAL_GetTick();

    target_stop_visual_servo();
    if (!ROBOT_STATUS_IS(g_robot.status, ROBOT_STATUS_POSE_VALID) ||
        robot_is_auto_busy() || robot_motion_abort_latched())
    {
        ROBOT_TARGET_ENABLED = false;
        return false;
    }

    robot_visual_servo_fault_clear();
    s_target.event = ROBOT_TARGET_EVENT_NONE;
    s_target.last_step_ms = now_ms;
    s_target.last_vision_ms = 0u;
    s_target.dcx = 0;
    s_target.dcy = 0;
    s_target.has_vision = false;
    s_target.ready_sent = false;
    s_target.fire_raw = true;
    s_target.fire_debounced = true;
    s_target.fire_raw_change_ms = now_ms;
    s_target.fire_release_seen = false;
    reset_alignment_counts();
    ROBOT_TARGET_ENABLED = true;
    enter_state(ROBOT_TARGET_STATE_WAIT_DETECT, now_ms);
    return true;
}

void robot_target_stop_hold(void)
{
    uint32_t now_ms = HAL_GetTick();
    ROBOT_TARGET_ENABLED = false;
    ROBOT_TARGET_FIRE_ENABLE = false;
    target_stop_visual_servo();
    enter_state(ROBOT_TARGET_STATE_HOLD, now_ms);
}

bool robot_target_enable_request(void)
{
    return robot_target_start_at_current_pose();
}

void robot_target_disable_request(void)
{
    robot_target_stop_hold();
}

void robot_target_mark_preposition_ready_once(void)
{
}

void robot_target_step(const target_obs_t *obs)
{
    uint32_t now_ms = (obs != NULL) ? obs->now_ms : HAL_GetTick();
    bool new_vision = (obs != NULL) && obs->has_vision;

    if (obs != NULL)
    {
        target_update_fire_button(obs->fire_button, now_ms);
    }

    if (!ROBOT_TARGET_ENABLED)
    {
        target_stop_visual_servo();
        return;
    }

    if (robot_visual_servo_fault_get() != ROBOT_VISUAL_SERVO_FAULT_NONE)
    {
        target_stop_visual_servo();
        if (s_target.state != ROBOT_TARGET_STATE_FAULT)
        {
            target_set_event(ROBOT_TARGET_EVENT_FAULT);
            enter_state(ROBOT_TARGET_STATE_FAULT, now_ms);
        }
        return;
    }

    if (new_vision)
    {
        if (obs->vision_valid)
        {
            s_target.dcx = obs->dcx;
            s_target.dcy = obs->dcy;
            s_target.last_vision_ms = now_ms;
            s_target.has_vision = true;
            LOG("[TARGET] vision state=%s dcx=%d dcy=%d stable=%u\r\n",
                target_state_name(s_target.state), (int)s_target.dcx,
                (int)s_target.dcy, (unsigned)s_target.stable_count);
        }
        else
        {
            s_target.has_vision = false;
            target_stop_visual_servo();
            if ((s_target.state == ROBOT_TARGET_STATE_ALIGN) ||
                (s_target.state == ROBOT_TARGET_STATE_CONFIRM) ||
                (s_target.state == ROBOT_TARGET_STATE_OUTPUT))
            {
                target_set_event(ROBOT_TARGET_EVENT_VISION_LOST);
                enter_state(ROBOT_TARGET_STATE_WAIT_DETECT, now_ms);
            }
        }
    }

    switch (s_target.state)
    {
        case ROBOT_TARGET_STATE_WAIT_DETECT:
            target_stop_visual_servo();
            if (!s_target.ready_sent)
            {
                target_set_event(ROBOT_TARGET_EVENT_READY);
                s_target.ready_sent = true;
            }
            if (vision_fresh(now_ms))
            {
                target_set_event(ROBOT_TARGET_EVENT_VISION_RECOVERED);
#if TARGET_USE_VISUAL_SERVO
                if (!target_start_visual_servo())
                {
                    target_set_event(ROBOT_TARGET_EVENT_FAULT);
                    enter_state(ROBOT_TARGET_STATE_FAULT, now_ms);
                    break;
                }
#endif
                enter_state(ROBOT_TARGET_STATE_ALIGN, now_ms);
            }
            break;

        case ROBOT_TARGET_STATE_ALIGN:
            if (!vision_fresh(now_ms))
            {
                target_stop_visual_servo();
                target_set_event(ROBOT_TARGET_EVENT_VISION_LOST);
                enter_state(ROBOT_TARGET_STATE_WAIT_DETECT, now_ms);
                break;
            }

            if (align_in_tolerance())
            {
                target_hold_visual_servo();
                if (new_vision && (s_target.stable_count < TARGET_ALIGN_STABLE_COUNT))
                {
                    s_target.stable_count++;
                }
                if (s_target.stable_count >= TARGET_ALIGN_STABLE_COUNT)
                {
                    enter_state(ROBOT_TARGET_STATE_CONFIRM, now_ms);
                }
                break;
            }

            reset_alignment_counts();
#if TARGET_USE_VISUAL_SERVO
            if (!target_start_visual_servo())
            {
                target_set_event(ROBOT_TARGET_EVENT_FAULT);
                enter_state(ROBOT_TARGET_STATE_FAULT, now_ms);
                break;
            }
            if (new_vision)
            {
                target_update_visual_servo_velocity(now_ms);
            }
#else
            if (new_vision && !target_send_alignment_step(now_ms))
            {
                target_set_event(ROBOT_TARGET_EVENT_FAULT);
                enter_state(ROBOT_TARGET_STATE_FAULT, now_ms);
            }
#endif
            break;

        case ROBOT_TARGET_STATE_CONFIRM:
            target_hold_visual_servo();
            if (!vision_fresh(now_ms))
            {
                target_stop_visual_servo();
                target_set_event(ROBOT_TARGET_EVENT_VISION_LOST);
                enter_state(ROBOT_TARGET_STATE_WAIT_DETECT, now_ms);
                break;
            }
            if (!align_in_tolerance())
            {
                target_set_event(ROBOT_TARGET_EVENT_ALIGNMENT_LOST);
                enter_state(ROBOT_TARGET_STATE_ALIGN, now_ms);
                break;
            }
            if (new_vision && !s_target.alignment_confirmed &&
                (s_target.confirm_stable_count < TARGET_CONFIRM_STABLE_COUNT))
            {
                s_target.confirm_stable_count++;
                if (s_target.confirm_stable_count >= TARGET_CONFIRM_STABLE_COUNT)
                {
                    s_target.alignment_confirmed = true;
                    target_set_event(ROBOT_TARGET_EVENT_ALIGN_DONE);
                }
            }
            if (s_target.alignment_confirmed && target_fire_allowed())
            {
                target_stop_visual_servo();
                enter_state(ROBOT_TARGET_STATE_OUTPUT, now_ms);
            }
            break;

        case ROBOT_TARGET_STATE_OUTPUT:
            target_stop_visual_servo();
            if (!vision_fresh(now_ms))
            {
                target_set_event(ROBOT_TARGET_EVENT_VISION_LOST);
                enter_state(ROBOT_TARGET_STATE_WAIT_DETECT, now_ms);
            }
            else if (!align_in_tolerance())
            {
                target_set_event(ROBOT_TARGET_EVENT_ALIGNMENT_LOST);
                enter_state(ROBOT_TARGET_STATE_ALIGN, now_ms);
            }
            else if (!target_fire_allowed() ||
                     ((now_ms - s_target.enter_ms) >= TARGET_OUTPUT_MAX_MS))
            {
                target_set_event(ROBOT_TARGET_EVENT_HOLD);
                enter_state(ROBOT_TARGET_STATE_HOLD, now_ms);
            }
            break;

        case ROBOT_TARGET_STATE_HOLD:
            target_stop_visual_servo();
            break;

        case ROBOT_TARGET_STATE_FAULT:
            target_stop_visual_servo();
            break;

        case ROBOT_TARGET_STATE_INIT:
        default:
            target_stop_visual_servo();
            break;
    }
}

robot_target_event_t robot_target_event_consume(void)
{
    robot_target_event_t event = s_target.event;
    s_target.event = ROBOT_TARGET_EVENT_NONE;
    return event;
}

bool robot_target_output_requested(void)
{
    return ROBOT_TARGET_ENABLED && (s_target.state == ROBOT_TARGET_STATE_OUTPUT);
}

robot_target_state_t robot_target_state_get(void)
{
    return s_target.state;
}
