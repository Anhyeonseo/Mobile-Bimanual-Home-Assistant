#ifndef ACTUATOR_CORE_LIFT_ENDPOINT_H
#define ACTUATOR_CORE_LIFT_ENDPOINT_H
#include "actuator_core/lift_controller.h"
#include <stddef.h>
/* AL v1, 36 bytes; LS v1, 64 bytes. HOME/MOVE/CANCEL/STATUS/KEEPALIVE/RESET.
 * Height is signed micrometres. Separate lease excludes free mobile velocity. */
typedef struct {uint32_t observed_ms;bool base_stopped,arms_safe,mode_ok,hardware_ok;} actuator_lift_interlock_t;
typedef struct {
    actuator_lift_t *lift;
    actuator_lift_interlock_t interlock;
    uint32_t boot_id,session,sequence,valid_until_ms,lease_max_ms,rejected;
    bool active,have_interlock,transport_ready;
} actuator_lift_endpoint_t;
bool actuator_lift_endpoint_init(actuator_lift_endpoint_t *e,actuator_lift_t *lift,
    uint32_t boot_id,uint32_t lease_max_ms);
void actuator_lift_endpoint_poll(actuator_lift_endpoint_t *e,uint32_t now_ms);
/* Always called on global stop; cancels output ownership without re-homing. */
void actuator_lift_endpoint_stop(actuator_lift_endpoint_t *e);
bool actuator_lift_endpoint_exchange(actuator_lift_endpoint_t *e,const uint8_t *command,
    size_t length,uint32_t now_ms,uint8_t response[64]);
#endif
