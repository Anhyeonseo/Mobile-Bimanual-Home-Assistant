#ifndef ACTUATOR_CORE_DEVICE_STARTUP_H
#define ACTUATOR_CORE_DEVICE_STARTUP_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* Explicit maintenance transaction plan for the four mobile motors. No normal
 * boot writes, no ID/baud changes, no automatic torque disable or homing. */
typedef struct { uint16_t model[4]; uint8_t baud_code; uint32_t timeout_ms; } actuator_device_profile_t;
typedef struct { uint8_t id,address,length,data[2]; bool write; uint32_t token; } actuator_device_operation_t;
typedef enum { DEVICE_STARTUP_IDLE, DEVICE_STARTUP_CHECKING, DEVICE_STARTUP_READY,
    DEVICE_STARTUP_FAULT } actuator_device_startup_state_t;
typedef struct {
    actuator_device_profile_t profile;
    actuator_device_startup_state_t state;
    actuator_device_operation_t pending;
    uint32_t started_ms,token;
    uint8_t axis,phase;
    bool allow_mode_change,mode_changed,active,eprom_unlocked;
} actuator_device_startup_t;
/* Zero-initialize before first begin. An active/unlocked transaction cannot restart.
 * Models and baud are read back, never guessed. Setting mode is a separately
 * selected maintenance operation requiring independent secured/torque-off proof.
 * Even then, register 40 is read and must say torque OFF before unlocking. */
bool actuator_device_startup_begin(actuator_device_startup_t *s,
    const actuator_device_profile_t *profile,uint32_t now_ms,
    bool change_modes,bool mechanism_secured);
bool actuator_device_startup_next(actuator_device_startup_t *s,uint32_t now_ms,
    actuator_device_operation_t *operation);
/* Caller validates STS reply ID/checksum, TX/RX completion and quiet, then passes
 * READ data or an empty WRITE acknowledgement. Timeout never retries a write. */
bool actuator_device_startup_reply(actuator_device_startup_t *s,uint32_t token,
    const uint8_t *data,size_t length,bool transport_ok,uint32_t now_ms);
void actuator_device_startup_poll(actuator_device_startup_t *s,uint32_t now_ms);
/* Two-slot persistent boot counter. Storage callback must make each slot write
 * durable and read it back before exposing a boot ID. Torn writes fail closed.
 * Initial all-FF storage requires explicit commissioning, never silent reset. */
typedef bool (*actuator_boot_read_fn)(void *context,unsigned slot,uint8_t bytes[16]);
typedef bool (*actuator_boot_write_fn)(void *context,unsigned slot,const uint8_t bytes[16]);
bool actuator_boot_advance(actuator_boot_read_fn read_slot,actuator_boot_write_fn write_slot,
    void *context,bool initialize_erased,uint32_t *boot_id);
#endif
