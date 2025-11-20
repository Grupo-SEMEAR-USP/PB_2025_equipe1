#ifndef DEF_H
#define DEF_H

#include <stdbool.h>
#include "driver/gpio.h"

/*--- Definições ---*/

#define CNTRL_PERIOD_MS 50  
#define COMM_PERIOD_MS 50

#define HIGH 1
#define LOW 0

#define PI 3.14159265359f

/*--- GPIO ---*/

/* Encoder */
// Motor direito
#define ENCODER_GPIO_RA     GPIO_NUM_18
#define ENCODER_GPIO_RB     GPIO_NUM_19

// Motor esquerdo
#define ENCODER_GPIO_LA     GPIO_NUM_14
#define ENCODER_GPIO_LB     GPIO_NUM_15

/* Ponte H */
#define STBY                GPIO_NUM_33

// Motor direito
#define INPUT_RIGHT_1       GPIO_NUM_4
#define INPUT_RIGHT_2       GPIO_NUM_2
#define PWM_RIGHT           GPIO_NUM_26

// Motor esquerdo
#define INPUT_LEFT_1        GPIO_NUM_27
#define INPUT_LEFT_2        GPIO_NUM_32 
#define PWM_LEFT            GPIO_NUM_25

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