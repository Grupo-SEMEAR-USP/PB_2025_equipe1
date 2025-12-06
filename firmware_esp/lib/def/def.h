#ifndef DEF_H
#define DEF_H

#include <stdbool.h>
#include "driver/gpio.h"

/*--- Definições ---*/

#define CNTRL_PERIOD_MS 50  
#define COMM_PERIOD_MS  20

#define HIGH 1
#define LOW 0

#define PI 3.14159265359f

// /*--- GPIO (Rizzolis) ---*/

// /* Encoder */
// // Motor direito (B)
// #define ENCODER_GPIO_RA     GPIO_NUM_33
// #define ENCODER_GPIO_RB     GPIO_NUM_32

// // Motor esquerdo (A)
// #define ENCODER_GPIO_LA     GPIO_NUM_14
// #define ENCODER_GPIO_LB     GPIO_NUM_15

// /* Ponte H */
// #define STBY                GPIO_NUM_18

// // Motor direito (B)
// #define INPUT_RIGHT_1       GPIO_NUM_21
// #define INPUT_RIGHT_2       GPIO_NUM_19
// #define PWM_RIGHT           GPIO_NUM_22
// #define LEDC_CHANNEL_RIGHT  LEDC_CHANNEL_1

// // Motor esquerdo (A)
// #define INPUT_LEFT_1        GPIO_NUM_16
// #define INPUT_LEFT_2        GPIO_NUM_17
// #define PWM_LEFT            GPIO_NUM_4
// #define LEDC_CHANNEL_LEFT   LEDC_CHANNEL_2

/*--- GPIO (Eleonora) ---*/

/* Encoder */
// Motor direito (B)
#define ENCODER_GPIO_RA     GPIO_NUM_5
#define ENCODER_GPIO_RB     GPIO_NUM_19

// Motor esquerdo (A)
#define ENCODER_GPIO_LA     GPIO_NUM_22
#define ENCODER_GPIO_LB     GPIO_NUM_23

/* Ponte H */
#define STBY                GPIO_NUM_25

// Motor direito (B)
#define INPUT_RIGHT_1       GPIO_NUM_12
#define INPUT_RIGHT_2       GPIO_NUM_14
#define PWM_RIGHT           GPIO_NUM_13
#define LEDC_CHANNEL_RIGHT  LEDC_CHANNEL_1

// Motor esquerdo (A)
#define INPUT_LEFT_1        GPIO_NUM_32
#define INPUT_LEFT_2        GPIO_NUM_33
#define PWM_LEFT            GPIO_NUM_27
#define LEDC_CHANNEL_LEFT   LEDC_CHANNEL_2

/*--- GPIO (Marco) ---*/

// /* Encoder */
// // Motor direito
// #define ENCODER_GPIO_RA     GPIO_NUM_26
// #define ENCODER_GPIO_RB     GPIO_NUM_14

// // Motor esquerdo
// #define ENCODER_GPIO_LA     GPIO_NUM_32
// #define ENCODER_GPIO_LB     GPIO_NUM_33

// /* Ponte H */
// #define STBY                GPIO_NUM_19

// // Motor direito
// #define INPUT_RIGHT_1       GPIO_NUM_22
// #define INPUT_RIGHT_2       GPIO_NUM_21
// #define PWM_RIGHT           GPIO_NUM_23
// #define LEDC_CHANNEL_RIGHT  LEDC_CHANNEL_1

// // Motor esquerdo
// #define INPUT_LEFT_1        GPIO_NUM_4
// #define INPUT_LEFT_2        GPIO_NUM_18
// #define PWM_LEFT            GPIO_NUM_25
// #define LEDC_CHANNEL_LEFT   LEDC_CHANNEL_0

/*--- Estrutura da Comunicação ---*/
typedef struct {
    float left_data;
    float right_data;
} data_t;

/*--- Variáveis Globais ---*/

extern float G_TARGET_L;
extern float G_TARGET_R;

extern int G_ENC_L;
extern int G_ENC_R;

extern float G_PWM_L;
extern float G_PWM_R;

extern float G_RADS_L;
extern float G_RADS_R;

// PID stop on zero
extern bool BREAK_FLAG;

// Comando Comunicação
extern data_t G_CMD;

#endif // DEF_H