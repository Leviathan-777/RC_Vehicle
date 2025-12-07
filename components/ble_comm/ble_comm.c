/**
 * @file ble_comm.c
 * @brief BLE communication for the RC vehicle.
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "ble_comm.h"
#include "vehicle_control.h"

#define BLE_DEVICE_NAME "ESP32 Vehicle"

#define FORWARD_CMD 'F'
#define REVERSE_CMD 'R'
#define BRAKE_CMD   'B'
#define STOP_CMD    'S'
#define LEFT_CMD    'L'
#define RIGHT_CMD   'G'
#define LIGHTS_CMD  'H'

static const char *TAG = "BLE_COMMUNICATION";

static uint16_t ble_attr_read_handle;
static uint16_t ble_attr_write_handle;

/* UART Service UUIDs */
static const ble_uuid128_t nus_svc_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);
static const ble_uuid128_t nus_rx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);
static const ble_uuid128_t nus_tx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

static uint8_t ble_conn_handle;

static int ble_uart_rx_callback(uint16_t conn_handle,
                                uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt,
                                void *arg);
static void ble_app_advertise(void);
static void ble_on_sync(void);

/**
 * @brief GATT access callback for the RX characteristic.
 *
 * This callback receives incoming GATT writes and parses simple single-letter
 * commands followed by an optional numeric speed value (e.g. "F50" = forward
 * at 50%). Commands supported: F, R, S, L, G, B.
 */
static int ble_uart_rx_callback(uint16_t conn_handle,
                                uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt,
                                void *arg)
{
    char incoming[64] = {0};
    int len = ctxt->om->om_len;

    if (len >= (int)sizeof(incoming)) 
    {
        len = (int)sizeof(incoming) - 1;
    }

    memcpy(incoming, ctxt->om->om_data, len);
    ESP_LOGI(TAG, "RX: %s", incoming);

    /* Parse simple commands */
    switch (incoming[0]) 
    {
        case FORWARD_CMD:
        {
            int speed = atoi(&incoming[1]);
            moveForward(speed);
            ESP_LOGI(TAG, "Vehicle forward %d%%", speed);
            break;
        }
        case REVERSE_CMD:
        {
            int speed = atoi(&incoming[1]);
            moveReverse(speed);
            ESP_LOGI(TAG, "Vehicle reverse %d%%", speed);
            break;
        }
        case LEFT_CMD:
        {
            int speed = atoi(&incoming[1]);
            turnLeft(speed);
            ESP_LOGI(TAG, "Vehicle left %d%%", speed);
            break;
        }
        case RIGHT_CMD:
        {
            int speed = atoi(&incoming[1]);
            turnRight(speed);
            ESP_LOGI(TAG, "Vehicle right %d%%", speed);
            break;
        }        
        case BRAKE_CMD:
        {
            brakeVehicle();
            ESP_LOGI(TAG, "Vehicle brake");
            break;
        }
        case STOP_CMD:
        {
            stopVehicle();
            ESP_LOGI(TAG, "Vehicle stop");
            break;
        }
        case LIGHTS_CMD:
        {
            toggleLights();
            ESP_LOGI(TAG, "Toggle lights");
            break;
        }
        default:
        {
            ESP_LOGW(TAG, "Unknown command");
        }
    }

    return 0;
}

/**
 * @brief Access callback for the TX characteristic.
 *
 * No read/write functionality is required for the TX characteristic in this
 * implementation; the callback exists to satisfy the GATT table.
 */
static int tx_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)ctxt;
    (void)arg;
    return 0;
}

/* UART GATT Table */
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        /*** UART Service ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (ble_uuid_t*)&nus_svc_uuid,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                /* RX characteristic (Write from client) */
                .uuid = (ble_uuid_t*)&nus_rx_uuid,
                .flags = BLE_GATT_CHR_F_WRITE,
                .access_cb = ble_uart_rx_callback,
                .arg = NULL,
                .descriptors = NULL,
                .min_key_size = 0,
                .val_handle = &ble_attr_write_handle,
            },
            {
                /* TX characteristic (Notify to client) */
                .uuid = (ble_uuid_t*)&nus_tx_uuid,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .access_cb = tx_access_cb,
                .arg = NULL,
                .descriptors = NULL,
                .min_key_size = 0,
                .val_handle = &ble_attr_read_handle,
            },
            {0}
        }
    },
    {0}
};

/**
 * @brief GAP event handler.
 *
 * Handles connect/disconnect/advertising-complete events and ensures advertising
 * is restarted when appropriate.
 */
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
        case BLE_GAP_EVENT_CONNECT:
        {
            if (event->connect.status == 0)
            {
                ESP_LOGI(TAG, "Connected");
                ble_conn_handle = event->connect.conn_handle;
                printf("Connection handle: %d\n", ble_conn_handle);
            } 
            else 
            {
                ESP_LOGI(TAG, "Connection failed, restarting advertising");
                ble_app_advertise();
            }
            break;
        }
        case BLE_GAP_EVENT_DISCONNECT:
        {
            ESP_LOGI(TAG, "Disconnected");
            ble_app_advertise();
            break;
        }
        case BLE_GAP_EVENT_ADV_COMPLETE:
        {
            ESP_LOGI(TAG, "Advertising complete");
            ble_app_advertise();
            break;
        }
        default:
        {
            break;
        }
    }
    return 0;
}

/**
 * @brief Configure advertisement fields and start advertising.
 */
static void ble_app_advertise(void)
{
    struct ble_gap_adv_params adv_params = 
    {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };

    struct ble_hs_adv_fields fields = {0};
    fields.name = (uint8_t *)BLE_DEVICE_NAME;
    fields.name_len = strlen(BLE_DEVICE_NAME);
    fields.name_is_complete = 1;

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids16 = NULL;
    fields.uuids128 = (ble_uuid128_t[]){ nus_svc_uuid };
    fields.uuids128_is_complete = 1;

    ble_gap_adv_set_fields(&fields);
    int rc = ble_gap_adv_start(0, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc != 0) 
    {
        ESP_LOGE(TAG, "Error starting advertisement: %d", rc);
    }
}

/**
 * @brief NimBLE sync callback; called when the host and controller are ready.
 */
static void ble_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_conn_handle);
    ble_app_advertise();
}

/**
 * @brief BLE host task used by NimBLE when running under FreeRTOS.
 *
 * This function runs the NimBLE event loop and returns only when the stack
 * is stopped.
 */
void bleprph_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

/**
 * @brief Send a notification on the TX characteristic to the connected peer.
 *
 * @param data Pointer to the buffer to send.
 * @param len Number of bytes to send.
 * @return 0 on success, negative or NimBLE error code on failure.
 */
int ble_send_notification(const void *data, size_t len)
{
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) 
    {
        ESP_LOGE(TAG, "Failed to allocate mbuf for notify");
        return -1;
    }

    int rc = ble_gattc_notify_custom(ble_conn_handle, ble_attr_read_handle, om);
    if (rc != 0) 
    {
        ESP_LOGE(TAG, "ble_gatts_notify failed: %d", rc);
    }

    return rc;
}

/**
 * @brief Initialize the BLE peripheral (NimBLE) and start the host task.
 */
void ble_init(void)
{
    /* Initialize NimBLE */
    ESP_ERROR_CHECK(nimble_port_init());
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT; /* No PIN */
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0; /* No Secure Connections */
    ble_hs_cfg.sync_cb = ble_on_sync;

    nimble_port_freertos_init(bleprph_host_task);

    ESP_LOGI(TAG, "BLE UART Motor Control Ready!");
}