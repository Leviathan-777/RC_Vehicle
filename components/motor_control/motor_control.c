#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "motor_control.h"
#include "esp_log.h"

/* Pin definitions */
#define MOTOR_A_IN1_GPIO 18U
#define MOTOR_A_IN2_GPIO 19U
#define MOTOR_B_IN1_GPIO 21U
#define MOTOR_B_IN2_GPIO 22U

#define MOTOR_A_IN1_PWM LEDC_CHANNEL_0
#define MOTOR_A_IN2_PWM LEDC_CHANNEL_1
#define MOTOR_B_IN1_PWM LEDC_CHANNEL_2
#define MOTOR_B_IN2_PWM LEDC_CHANNEL_3

/* PWM settings */
#define PWM_FREQ        50000U    // 50 kHz
#define PWM_RES         LEDC_TIMER_10_BIT
#define PWM_MAX_DUTY    1023U  // 1023 for 10-bit
#define PWM_MIN_DUTY    0U

/* Motor driver polarity */
#define REVERSE_POLARITY 0U  // Set to 1 to swap motor directions

static const char *TAG = "MOTOR_CONTROL";

/* Static function prototypes */
static void pwm_init_timer(void);
static void pwm_init_channel(int gpio, ledc_channel_t channel);
static inline void set_duty(ledc_channel_t channel, uint16_t duty);


/* Timer config */
static void pwm_init_timer(void)
{
    ledc_timer_config_t timer = 
    {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .duty_resolution = PWM_RES,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);
}

/* Channel config for each IN pin */
static void pwm_init_channel(int gpio, ledc_channel_t channel)
{
    ledc_channel_config_t ch = 
    {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = channel,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = gpio,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ch);
}

/* Public init function */
void motor_init(void)
{
    pwm_init_timer();

    pwm_init_channel(REVERSE_POLARITY ? MOTOR_A_IN1_GPIO : MOTOR_A_IN2_GPIO, MOTOR_A_IN1_PWM);
    pwm_init_channel(REVERSE_POLARITY ? MOTOR_A_IN2_GPIO : MOTOR_A_IN1_GPIO, MOTOR_A_IN2_PWM);
    pwm_init_channel(REVERSE_POLARITY ? MOTOR_B_IN1_GPIO : MOTOR_B_IN2_GPIO, MOTOR_B_IN1_PWM);
    pwm_init_channel(REVERSE_POLARITY ? MOTOR_B_IN2_GPIO : MOTOR_B_IN1_GPIO, MOTOR_B_IN2_PWM);
}

/* Helper to set PWM duty */
static inline void set_duty(ledc_channel_t channel, uint16_t duty)
{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
}

// =========================
// MOTOR CONTROL FUNCTIONS
// =========================
void motor_forward(motor_id_t motor, int speed)
{
    uint16_t duty = (PWM_MAX_DUTY * speed) / 100U;
    switch (motor)
    {
        case MOTOR_A:
            set_duty(MOTOR_A_IN1_PWM, duty);   // IN1 → PWM
            set_duty(MOTOR_A_IN2_PWM, 0);      // IN2 → LOW
            break;
        case MOTOR_B:
            set_duty(MOTOR_B_IN1_PWM, duty);   // IN1 → PWM
            set_duty(MOTOR_B_IN2_PWM, 0);      // IN2 → LOW
            break;
        default:
            ESP_LOGE(TAG, "Invalid motor ID in motor_forward");
            break;
    }
}



void motor_reverse(motor_id_t motor,int speed)
{
    uint16_t duty = (PWM_MAX_DUTY * speed) / 100U;
    switch (motor)
    {
        case MOTOR_A:
            set_duty(MOTOR_A_IN1_PWM, 0);    /* IN1 → LOW */
            set_duty(MOTOR_A_IN2_PWM, duty); /* IN2 → PWM */
            break;
        case MOTOR_B:
            set_duty(MOTOR_B_IN1_PWM, 0);    /* IN1 → LOW */
            set_duty(MOTOR_B_IN2_PWM, duty); /* IN2 → PWM */
            break;
        default:
            ESP_LOGE(TAG, "Invalid motor ID in motor_reverse");
            break;
    }
}

void motor_brake(motor_id_t motor)
{
    switch (motor)
    {
        case MOTOR_A:
            set_duty(MOTOR_A_IN1_PWM, PWM_MAX_DUTY);
            set_duty(MOTOR_A_IN2_PWM, PWM_MAX_DUTY);
            break;
        case MOTOR_B:
            set_duty(MOTOR_B_IN1_PWM, PWM_MAX_DUTY);
            set_duty(MOTOR_B_IN2_PWM, PWM_MAX_DUTY);
            break;
        default:
            ESP_LOGE(TAG, "Invalid motor ID in motor_brake");
            break;
    }
}

void motor_stop(motor_id_t motor)
{
    switch (motor)
    {
        case MOTOR_A:
            set_duty(MOTOR_A_IN1_PWM, PWM_MIN_DUTY);
            set_duty(MOTOR_A_IN2_PWM, PWM_MIN_DUTY);
            break;
        case MOTOR_B:
            set_duty(MOTOR_B_IN1_PWM, PWM_MIN_DUTY);
            set_duty(MOTOR_B_IN2_PWM, PWM_MIN_DUTY);
            break;
        default:
            ESP_LOGE(TAG, "Invalid motor ID in motor_stop");
            break;
    }
}
