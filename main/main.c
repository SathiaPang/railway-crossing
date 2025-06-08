/**
 * Project: Automated Railway Crossing Gate (Two-Sensor System - Final Logic)
 * Author: Alex Satya
 * Created on: 08-June-2025
 * Hardware: ESP32-38Pins-ZY-ESP32
 * Software framework: ESP-IDF v5.4.1 (VS Code Extension 1.9.1)
 * Description: Final version with a safety-clearing sequence. After a train passes,
 * the system waits 5 seconds with a flashing light before opening the gate.
 */

/*----------------------------------*/
// HEADERS
/*----------------------------------*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

/*----------------------------------*/
// PROGRAM CONSTANTS
/*----------------------------------*/
// GPIO Pin Definitions
#define RED_PIN 26
#define GREEN_PIN 27
#define BUZZER_PIN 25
#define SERVO_PIN 2

// Sensor Configuration
#define APPROACH_SENSOR_ADC_CHANNEL ADC_CHANNEL_6 // GPIO34
#define CROSSING_SENSOR_ADC_CHANNEL ADC_CHANNEL_5 // GPIO33
#define SENSOR_THRESHOLD 2000                     // If sensor value < threshold, train detected

// Servo Configuration
#define SERVO_LEDC_TIMER LEDC_TIMER_0
#define SERVO_LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_TIMER_BIT LEDC_TIMER_13_BIT
#define LEDC_BASE_FREQ 50
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2500
#define SERVO_MAX_DEGREE 180
#define GATE_OPEN_ANGLE 90
#define GATE_CLOSED_ANGLE 0

// State Machine -- UPDATED WITH NEW STATE
enum State
{
    IDLE,
    TRAIN_APPROACHING,
    TRAIN_PASSING,
    CLEARING_CROSSING
};
enum State currentState = IDLE;

/*----------------------------------*/
// GLOBAL VARIABLES
/*----------------------------------*/
static const char *TAG = "RAILWAY_SYSTEM_V3";
adc_oneshot_unit_handle_t adc1_handle;

/*----------------------------------*/
// FUNCTIONS DECLARATION
/*----------------------------------*/
static void configure_peripherals(void);
static void init_servo(void);
void move_servo_to(int angle);
static uint32_t servo_angle_to_duty(int angle);

/*----------------------------------*/
// MAIN FUNCTION -- FULLY REVISED LOGIC
/*----------------------------------*/
void app_main(void)
{
    configure_peripherals();
    init_servo();

    // Set initial state
    move_servo_to(GATE_OPEN_ANGLE);
    gpio_set_level(GREEN_PIN, 1);
    gpio_set_level(RED_PIN, 0);
    gpio_set_level(BUZZER_PIN, 0);

    while (1)
    {
        int approach_sensor_val, crossing_sensor_val;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, APPROACH_SENSOR_ADC_CHANNEL, &approach_sensor_val));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, CROSSING_SENSOR_ADC_CHANNEL, &crossing_sensor_val));

        bool approach_detected = (approach_sensor_val < SENSOR_THRESHOLD);
        bool crossing_detected = (crossing_sensor_val < SENSOR_THRESHOLD);

        switch (currentState)
        {
        case IDLE:
            if (approach_detected)
            {
                ESP_LOGI(TAG, "STATE: IDLE -> TRAIN_APPROACHING");
                currentState = TRAIN_APPROACHING;

                gpio_set_level(GREEN_PIN, 0);
                // Warning sequence
                for (int i = 0; i < 3; i++)
                {
                    gpio_set_level(RED_PIN, 1);
                    gpio_set_level(BUZZER_PIN, 1);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    gpio_set_level(RED_PIN, 0);
                    gpio_set_level(BUZZER_PIN, 0);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                }
                move_servo_to(GATE_CLOSED_ANGLE);
                gpio_set_level(RED_PIN, 1); // Solid red after closing
            }
            break;

        case TRAIN_APPROACHING:
            if (crossing_detected)
            {
                ESP_LOGI(TAG, "STATE: TRAIN_APPROACHING -> TRAIN_PASSING");
                currentState = TRAIN_PASSING;
            }
            break;

        case TRAIN_PASSING:
            // MODIFIED: When train leaves crossing sensor, move to CLEARING state
            if (!crossing_detected)
            {
                ESP_LOGI(TAG, "STATE: TRAIN_PASSING -> CLEARING_CROSSING");
                currentState = CLEARING_CROSSING;
            }
            break;

        // NEW STATE: Implement the 5-second safety delay
        case CLEARING_CROSSING:
            ESP_LOGI(TAG, "Track clear. Starting 5-second safety delay.");
            bool safe_to_open = true;
            // Flash red light for 5 seconds
            for (int i = 0; i < 5; i++)
            {
                gpio_set_level(RED_PIN, 1);
                vTaskDelay(500 / portTICK_PERIOD_MS);
                gpio_set_level(RED_PIN, 0);
                vTaskDelay(500 / portTICK_PERIOD_MS);

                // SAFETY CHECK: Re-read sensor during delay. If a train appears, abort opening!
                adc_oneshot_read(adc1_handle, APPROACH_SENSOR_ADC_CHANNEL, &approach_sensor_val);
                if (approach_sensor_val < SENSOR_THRESHOLD)
                {
                    ESP_LOGE(TAG, "New train detected during clearing! Aborting gate open.");
                    currentState = TRAIN_APPROACHING; // Go back to approaching state
                    safe_to_open = false;
                    break; // Exit the for loop immediately
                }
            }

            // Only open the gate if the delay completed without detecting a new train
            if (safe_to_open)
            {
                ESP_LOGI(TAG, "STATE: CLEARING_CROSSING -> IDLE");
                gpio_set_level(GREEN_PIN, 1);
                move_servo_to(GATE_OPEN_ANGLE);
                currentState = IDLE;
            }
            break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/*----------------------------------*/
// FUNCTION DEFINITIONS (These are unchanged)
/*----------------------------------*/
static void configure_peripherals(void)
{
    gpio_set_direction(RED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREEN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
    adc_oneshot_unit_init_cfg_t init_config1 = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, APPROACH_SENSOR_ADC_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, CROSSING_SENSOR_ADC_CHANNEL, &config));
}

static void init_servo(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = SERVO_LEDC_TIMER, .duty_resolution = LEDC_TIMER_BIT, .freq_hz = LEDC_BASE_FREQ, .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&timer_conf);
    ledc_channel_config_t channel_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .channel = SERVO_LEDC_CHANNEL, .timer_sel = SERVO_LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .gpio_num = SERVO_PIN, .duty = 0, .hpoint = 0};
    ledc_channel_config(&channel_conf);
}

static uint32_t servo_angle_to_duty(int angle)
{
    uint32_t pulse_us = SERVO_MIN_PULSEWIDTH_US + ((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle / SERVO_MAX_DEGREE);
    uint32_t duty = ((1 << LEDC_TIMER_BIT) - 1) * pulse_us / 20000;
    return duty;
}

void move_servo_to(int angle)
{
    uint32_t duty_cycle = servo_angle_to_duty(angle);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL, duty_cycle);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL);
}