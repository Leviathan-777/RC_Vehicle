/*
 * Public API for motor control component
 */

#ifndef RC_VEHICLE_MOTOR_CONTROL_H
#define RC_VEHICLE_MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    MOTOR_A = 0U,
    MOTOR_B = 1U
} motor_id_t;

/* Initialize PWM timer and motor GPIO channels. Call once at startup. */
void motor_init(void);

/* Control functions for motors */
void motor_forward(motor_id_t motor, int speed);
void motor_reverse(motor_id_t motor, int speed);
void motor_brake(motor_id_t motor);
void motor_stop(motor_id_t motor);

#ifdef __cplusplus
}
#endif

#endif /* RC_VEHICLE_MOTOR_CONTROL_H */