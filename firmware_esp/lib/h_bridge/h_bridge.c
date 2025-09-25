#include "h_bridge.h"

void init_gpio_bridge(){
    gpio_set_direction(INPUT_LEFT_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(INPUT_LEFT_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(STBY_LEFT, GPIO_MODE_OUTPUT);
    gpio_set_level(STBY_LEFT, HIGH);

    gpio_set_direction(INPUT_RIGHT_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(INPUT_RIGHT_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(STBY_RIGHT, GPIO_MODE_OUTPUT);
    gpio_set_level(STBY_RIGHT, HIGH);
    
    gpio_set_direction(LEDC_OUTPUT_RIGHT, GPIO_MODE_OUTPUT);
    gpio_set_direction(LEDC_OUTPUT_LEFT, GPIO_MODE_OUTPUT);
}

void init_pwm_bridge(){
    ledc_timer_config_t ledc_timer_left = {
        .speed_mode         = LEDC_MODE,
        .timer_num          = LEDC_TIMER_LEFT,
        .duty_resolution    = LEDC_DUTY_RES,
        .freq_hz            = LEDC_FREQUENCY,
        .clk_cfg            = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_left));

    ledc_timer_config_t ledc_timer_right = {
        .speed_mode         = LEDC_MODE,
        .timer_num          = LEDC_TIMER_RIGHT,
        .duty_resolution    = LEDC_DUTY_RES,
        .freq_hz            = LEDC_FREQUENCY,
        .clk_cfg            = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_right));

    ledc_channel_config_t ledc_left_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = MOTOR_CHANNEL(MOTOR_LEFT),
        .timer_sel      = LEDC_TIMER_LEFT,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_OUTPUT(MOTOR_LEFT),
        .duty           = 0,
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_left_channel));

    ledc_channel_config_t ledc_right_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = MOTOR_CHANNEL(MOTOR_RIGHT),
        .timer_sel      = LEDC_TIMER_RIGHT,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_OUTPUT(MOTOR_RIGHT),
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_right_channel));
}

// Define a velocidade e a direção do motor (assumindo a direção pelo sinal da velocidade)
esp_err_t update_motor(motor_side_t motor, int u){
    u > 0 ? _set_forward(motor) : _set_backward(motor);
    
    u = u > 0 ? u : -u;  // Abs of action control u

    ledc_set_duty(LEDC_MODE, MOTOR_CHANNEL(motor), u);
    ledc_update_duty(LEDC_MODE, MOTOR_CHANNEL(motor));

    return ESP_OK;
}

// Controla motor para frente, considerando in1 = 1 e in2 = 0
esp_err_t _set_forward(motor_side_t motor){
    gpio_set_level(MOTOR_INPUT_1(motor), HIGH);
    gpio_set_level(MOTOR_INPUT_2(motor), LOW);

    return ESP_OK;
}

// Controla motor para trás, considerando in1 = 0 e in2 = 1
esp_err_t _set_backward(motor_side_t motor){
    gpio_set_level(MOTOR_INPUT_1(motor), LOW);
    gpio_set_level(MOTOR_INPUT_2(motor), HIGH);

    return ESP_OK;
}