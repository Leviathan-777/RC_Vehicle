/**
 * @file vehicle_control.h
 * @brief Public API for the RC vehicle control component.
 *
 * This header exposes simple, blocking control functions for two motors
 * (A and B). Functions operate on percentage speed values (0-100).
 */

#ifndef RC_VEHICLE_VEHICLE_CONTROL_H
#define RC_VEHICLE_VEHICLE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

void vehicle_control_init(void);
void toggleLights();
void moveForward(int speed_percent);
void moveReverse(int speed_percent);
void turnLeft(int speed_percent);
void turnRight(int speed_percent);
void brakeVehicle();
void stopVehicle();


#ifdef __cplusplus
}
#endif

#endif /* RC_VEHICLE_VEHICLE_CONTROL_H */