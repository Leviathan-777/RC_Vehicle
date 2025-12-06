/**
 * @file ble_comm.h
 * @brief Public API for the RC vehicle BLE communication component.
 *
 * This header exposes initialization and BLE data transmission functions.
 */

#ifndef RC_VEHICLE_BLE_COMM_H
#define RC_VEHICLE_BLE_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

/**
 * @brief Initialize BLE subsystem and start advertising.
 *
 * This will initialize the NimBLE host, register GATT services and start
 * the BLE host task. Call once at startup.
 */
void ble_init(void);

/**
 * @brief Send a BLE notification on the TX characteristic.
 *
 * @param data Pointer to the data buffer to send.
 * @param len Length of the data buffer in bytes.
 * @return 0 on success, negative or NimBLE error code on failure.
 */
int ble_send_notification(const void *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* RC_VEHICLE_BLE_COMM_H */