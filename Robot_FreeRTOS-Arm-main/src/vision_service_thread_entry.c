#include "vision_service_thread.h"
#include "jetson_vision.h"
#include "robot_workflow.h"
#include "bsp_laser.h"

void vision_service_thread_entry(void * pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    jetson_vision_init();
    robot_workflow_init();

    while (1)
    {
        robot_workflow_obs_t obs = {0};

        jetson_vision_process();
        obs.now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        obs.has_workflow_control = jetson_get_workflow_control(&obs.workflow_action,
                                                               &obs.workflow_seq);
        obs.has_capture_control = jetson_get_capture_control_ex(&obs.capture_action,
                                                                &obs.capture_point_id,
                                                                &obs.capture_seq);
        obs.has_target_control = jetson_get_target_control_value(&obs.target_value,
                                                                 &obs.target_seq);
        obs.has_vision = jetson_get_vision_error_ex(&obs.dcx, &obs.dcy,
                                                    &obs.vision_valid,
                                                    &obs.vision_seq);
        obs.has_distance = jetson_get_safe_distance_ex(&obs.distance_mm,
                                                       &obs.distance_valid,
                                                       &obs.distance_seq);
        obs.heartbeat_alive = jetson_is_link_alive(obs.now_ms);
        obs.fire_button = BSP_Laser_FireKey_IsPressed();
        robot_workflow_step(&obs);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
