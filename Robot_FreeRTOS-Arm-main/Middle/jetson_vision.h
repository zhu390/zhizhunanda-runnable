#ifndef JETSON_VISION_H_
#define JETSON_VISION_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef JETSON_LEGACY_PROTOCOL_ENABLE
#define JETSON_LEGACY_PROTOCOL_ENABLE 0
#endif

#define JETSON_SOF                 0xFFu
#define JETSON_EOF                 0xFEu
#define JETSON_FRAME_LEN           0x05u
#define JETSON_FUNC_VISION_ERROR   0x03u
#define JETSON_VISION_PAYLOAD_LEN  4u

#define JETSON_CTRL_SOF            0xAAu
#define JETSON_CTRL_EOF            0xBBu
#define JETSON_FUNC_TARGET_CTRL    0x01u

#define JETSON_UNIFIED_SOF0        0xA5u
#define JETSON_UNIFIED_SOF1        0x5Au
#define JETSON_UNIFIED_VERSION     0x01u
#define JETSON_UNIFIED_MAX_PAYLOAD 32u

#define JETSON_MSG_HEARTBEAT       0x01u
#define JETSON_MSG_TARGET_CTRL     0x02u
#define JETSON_MSG_VISION_ERROR    0x03u
#define JETSON_MSG_CAPTURE_CTRL    0x04u
#define JETSON_MSG_SAFE_DISTANCE   0x05u
#define JETSON_MSG_WORKFLOW_CTRL   0x06u
#define JETSON_MSG_STATUS          0x81u
#define JETSON_MSG_ERROR           0xFEu

#define JETSON_HEARTBEAT_TIMEOUT_MS 600u

#define RA6_TO_JETSON_SOF          0xCCu
#define RA6_TO_JETSON_EOF          0xDDu
#define RA6_TO_JETSON_READY        0x01u
#define RA6_TO_JETSON_ALIGN_DONE   0x02u
#define RA6_TO_JETSON_OUTPUT       0x03u
#define RA6_TO_JETSON_TARGET_CTRL  0x04u
#define RA6_TO_JETSON_HEARTBEAT    0x05u
#define RA6_TO_JETSON_SAFE_DISTANCE 0x06u
#define RA6_TO_JETSON_VISION_STATE 0x07u
#define RA6_TO_JETSON_CAPTURE_POINT 0x10u
#define RA6_TO_JETSON_CAPTURE_HOME  0x11u
#define RA6_TO_JETSON_SELECTED_VIEW 0x12u
#define RA6_TO_JETSON_WORKFLOW      0x20u
#define RA6_TO_JETSON_COMMAND_ACK   0x21u
#define RA6_TO_JETSON_ERROR        0xFEu

#define RA6_TO_JETSON_CAPTURE_DONE RA6_TO_JETSON_CAPTURE_HOME
#define RA6_TO_JETSON_TARGET_PRESTART RA6_TO_JETSON_SELECTED_VIEW

#define JETSON_CAPTURE_ACTION_GOTO   0x01u
#define JETSON_CAPTURE_ACTION_HOME   0x02u
#define JETSON_CAPTURE_ACTION_SELECT 0x03u
#define JETSON_CAPTURE_ACTION_CURRENT 0x04u

#define JETSON_CAPTURE_ACTION_FINISH JETSON_CAPTURE_ACTION_HOME

#define JETSON_WORKFLOW_ACTION_START_MEASURE      0x01u
#define JETSON_WORKFLOW_ACTION_FINISH_RETURN_HOME 0x02u
#define JETSON_WORKFLOW_ACTION_ABORT_HOLD         0x03u

#define JETSON_WORKFLOW_IDLE                       0x00u
#define JETSON_WORKFLOW_START_ACCEPTED             0x01u
#define JETSON_WORKFLOW_MEASURE_POSITION_READY     0x02u
#define JETSON_WORKFLOW_DISTANCE_SAFE_LATCHED      0x03u
#define JETSON_WORKFLOW_RETREAT_DONE_WAIT_RESTART  0x04u
#define JETSON_WORKFLOW_RETURN_HOME_DONE           0x05u
#define JETSON_WORKFLOW_ABORTED_HOLD               0x06u
#define JETSON_WORKFLOW_FAULT_HOLD                 0x07u
#define JETSON_WORKFLOW_RETREAT_STEP_READY         0x08u

#define JETSON_ERROR_NONE          0x00u
#define JETSON_ERROR_VERSION_MISMATCH 0x01u
#define JETSON_ERROR_UNKNOWN_TYPE  0x02u
#define JETSON_ERROR_INVALID_PARAM 0x03u
#define JETSON_ERROR_POSE_INVALID  0x04u
#define JETSON_ERROR_BUSY          0x05u
#define JETSON_ERROR_MOTION_FAILED 0x06u
#define JETSON_ERROR_HEARTBEAT_TIMEOUT 0x07u
#define JETSON_ERROR_SAFETY        0x08u
#define JETSON_ERROR_SAFE_DISTANCE_TOO_CLOSE 0x09u
#define JETSON_ERROR_VISION_LOST   0x0Au
#define JETSON_ERROR_SOFT_RESET_FAILED 0x0Bu
#define JETSON_ERROR_INVALID_STATE 0x0Cu
#define JETSON_ERROR_SEQ_CONFLICT  0x0Du
#define JETSON_ERROR_TARGET_GATE_DENIED 0x0Eu
#define JETSON_ERROR_MOTION_ABORTED 0x0Fu

void jetson_vision_init(void);
void jetson_vision_process(void);
bool jetson_get_vision_error_ex(int16_t *dcx, int16_t *dcy, bool *valid, uint8_t *seq);
bool jetson_get_target_control_value(uint8_t *value, uint8_t *seq);
bool jetson_get_target_control_ex(bool *enable, uint8_t *seq);
bool jetson_get_capture_control_ex(uint8_t *action, uint8_t *point_id, uint8_t *seq);
bool jetson_get_safe_distance_ex(uint16_t *distance_mm, bool *valid, uint8_t *seq);
bool jetson_get_workflow_control(uint8_t *action, uint8_t *seq);
bool jetson_get_vision_error(int16_t *dcx, int16_t *dcy);
bool jetson_get_target_control(bool *enable);
bool jetson_get_capture_control(uint8_t *action, uint8_t *point_id);
bool jetson_get_safe_distance(uint16_t *distance_mm, bool *valid);
bool jetson_is_unified_protocol_active(void);
bool jetson_is_link_alive(uint32_t now_ms);
bool jetson_send_status(uint8_t seq, uint8_t event, uint8_t value, uint8_t error_code);
bool jetson_send_error(uint8_t seq, uint8_t error_code);
bool jetson_send_status_u8(uint8_t func, uint8_t value);
void jetson_notify_rx_char_from_isr(uint8_t byte);
void jetson_notify_tx_complete_from_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* JETSON_VISION_H_ */
