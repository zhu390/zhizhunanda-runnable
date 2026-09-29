#ifndef ROBOT_CAPTURE_H_
#define ROBOT_CAPTURE_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ROBOT_CAPTURE_RESULT_NONE = 0,
    ROBOT_CAPTURE_RESULT_RUNNING,
    ROBOT_CAPTURE_RESULT_OK,
    ROBOT_CAPTURE_RESULT_FAILED,
    ROBOT_CAPTURE_RESULT_ABORTED,
} robot_capture_result_t;

void robot_capture_init(void);
bool robot_capture_request(uint8_t action, uint8_t point_id);
void robot_capture_step(uint32_t now_ms);
void robot_capture_cancel(void);
bool robot_capture_is_active(void);
robot_capture_result_t robot_capture_result_consume(uint8_t *action, uint8_t *point_id);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_CAPTURE_H_ */
