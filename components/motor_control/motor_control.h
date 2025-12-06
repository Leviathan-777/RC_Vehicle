/**
 * @file motor_control.h
 * @brief Public API for the RC vehicle motor control component.
 *
 * This header exposes simple, blocking control functions for two motors
 * (A and B). Functions operate on percentage speed values (0-100).
 */

#ifndef RC_VEHICLE_MOTOR_CONTROL_H
#define RC_VEHICLE_MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Motor identifier */
typedef enum
{
    MOTOR_A = 0U, /**< Motor A */
    MOTOR_B = 1U  /**< Motor B */
} motor_id_t;

/**
 * @brief Initialize PWM timer and motor GPIO channels.
 *
 * Call once at application startup before invoking other motor APIs.
 */
void motor_init(void);

/**
 * @brief Drive a motor forward at the requested speed.
 *
 * @param motor Motor identifier (MOTOR_A or MOTOR_B).
 * @param speed Speed percentage in range 0..100.
 */
void motor_forward(motor_id_t motor, int speed);

/**
 * @brief Drive a motor in reverse at the requested speed.
 *
 * @param motor Motor identifier (MOTOR_A or MOTOR_B).
 * @param speed Speed percentage in range 0..100.
 */
void motor_reverse(motor_id_t motor, int speed);

/**
 * @brief Apply braking to the selected motor (both inputs driven HIGH).
 *
 * @param motor Motor identifier (MOTOR_A or MOTOR_B).
 */
void motor_brake(motor_id_t motor);

/**
 * @brief Stop the selected motor (both inputs driven LOW).
 *
 * @param motor Motor identifier (MOTOR_A or MOTOR_B).
 */
void motor_stop(motor_id_t motor);

#ifdef __cplusplus
}
#endif

#endif /* RC_VEHICLE_MOTOR_CONTROL_H */