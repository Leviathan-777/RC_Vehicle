/**
 * @file vehicle_control.c
 * @brief Vehicle control implementation for the RC vehicle.
 */

#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "vehicle_control.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "motor_control.h"


static const char *TAG = "VEHICLE_CONTROL";

void toggleLights()
{
    gpio_set_level(GPIO_NUM_2, !gpio_get_level(GPIO_NUM_2));
    gpio_set_level(GPIO_NUM_4, !gpio_get_level(GPIO_NUM_4));
    ESP_LOGI(TAG, "Toggled lights");
}

void moveForward(int speed_percent)
{
    motor_forward(MOTOR_A, speed_percent);
    motor_forward(MOTOR_B, speed_percent);
    ESP_LOGI(TAG, "Vehicle moving forward at %d%% speed", speed_percent);
}

void moveReverse(int speed_percent)
{
    motor_reverse(MOTOR_A, speed_percent);
    motor_reverse(MOTOR_B, speed_percent);
    ESP_LOGI(TAG, "Vehicle moving reverse at %d%% speed", speed_percent);
}

void turnLeft(int speed_percent)
{
    motor_reverse(MOTOR_A, speed_percent);
    motor_forward(MOTOR_B, speed_percent);
    ESP_LOGI(TAG, "Vehicle turning left at %d%% speed", speed_percent);
}

void turnRight(int speed_percent)
{
    motor_forward(MOTOR_A, speed_percent);
    motor_reverse(MOTOR_B, speed_percent);
    ESP_LOGI(TAG, "Vehicle turning right at %d%% speed", speed_percent);
}

void brakeVehicle()
{
    motor_brake(MOTOR_A);
    motor_brake(MOTOR_B);
    ESP_LOGI(TAG, "Vehicle braking");
}

void stopVehicle()
{
    motor_stop(MOTOR_A);
    motor_stop(MOTOR_B);
    ESP_LOGI(TAG, "Vehicle stopped");
}

void vehicle_control_init()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_2) | (1ULL << GPIO_NUM_4),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_2, 0);
    gpio_set_level(GPIO_NUM_4, 0);
    motor_init();
    ESP_LOGI(TAG, "Vehicle control initialized");
}