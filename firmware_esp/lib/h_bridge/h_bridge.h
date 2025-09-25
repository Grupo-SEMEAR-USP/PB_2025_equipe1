#ifndef H_BRIDGE_H
#define H_BRIDGE_H

// Inclusão do mínimo de bibliotecas necessárias
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

typedef enum{
    MOTOR_RIGHT = 0,
    MOTOR_LEFT = 1,
} motor_side_t;

// Definem constantes para representar os estados digitais dos pinos
#define HIGH 1
#define LOW  0

// GPIOs da ponte H para o motor esquerdo
#define INPUT_LEFT_1      GPIO_NUM_25 
#define INPUT_LEFT_2      GPIO_NUM_33
#define STBY_LEFT        GPIO_NUM_32 

// GPIOs da ponte H para o motor direito
#define INPUT_RIGHT_1     GPIO_NUM_19
#define INPUT_RIGHT_2     GPIO_NUM_21
#define STBY_RIGHT        GPIO_NUM_22

// Pinos de saída para o sinal PWM
#define LEDC_OUTPUT_LEFT  GPIO_NUM_26 
#define LEDC_CHANNEL_LEFT  LEDC_CHANNEL_0

#define LEDC_OUTPUT_RIGHT GPIO_NUM_23
#define LEDC_CHANNEL_RIGHT LEDC_CHANNEL_1

// Configurações do LEDC (PWM)
#define LEDC_MODE          LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER_LEFT    LEDC_TIMER_0
#define LEDC_TIMER_RIGHT   LEDC_TIMER_0
#define LEDC_DUTY_RES      LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY     (5000) // 5 kHz

// LEDC Fade
#define LEDC_MAX_DUTY      (8191) // 2^res - 1
#define LEDC_MIN_DUTY      (0)
#define LEDC_FADE_DURATION (3000) // 3s 

// Funções macro
#define MOTOR_INPUT_1(MOTOR_SIDE) ((MOTOR_SIDE) == (MOTOR_LEFT) ? INPUT_LEFT_1 : INPUT_RIGHT_1)
#define MOTOR_INPUT_2(MOTOR_SIDE) ((MOTOR_SIDE) == (MOTOR_LEFT) ? INPUT_LEFT_2 : INPUT_RIGHT_2)
#define MOTOR_STBY(MOTOR_SIDE) ((MOTOR_SIDE) == (MOTOR_LEFT) ? STBY_LEFT : STBY_RIGHT)
#define MOTOR_CHANNEL(MOTOR_SIDE) ((MOTOR_SIDE) == (MOTOR_LEFT) ? LEDC_CHANNEL_LEFT : LEDC_CHANNEL_RIGHT)
#define MOTOR_OUTPUT(MOTOR_SIDE) ((MOTOR_SIDE) == (MOTOR_LEFT) ? LEDC_OUTPUT_LEFT : LEDC_OUTPUT_RIGHT)

// Protótipos das funções
void init_gpio_bridge();
void init_pwm_bridge();
esp_err_t update_motor(motor_side_t motor, int u);
esp_err_t _set_forward(motor_side_t motor);
esp_err_t _set_backward(motor_side_t motor);

#endif