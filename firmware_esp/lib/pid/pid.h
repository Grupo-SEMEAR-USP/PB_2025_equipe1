#ifndef PID_H
#define PID_H

#include "pid_ctrl.h"
#include "encoder.h"
#include "h_bridge.h"

// Target values that will be send to PID 
extern float TARGET_VALUE_L;
extern float TARGET_VALUE_R;

// Values read from encoder
extern int ENCODER_READ_L;
extern int ENCODER_READ_R;

// PWM values that will be send to motors
extern float LEFT_PWM_VALUE;
extern float RIGHT_PWM_VALUE;

extern float RADS_L;
extern float RADS_R;

// PID stop on zero
extern bool BREAK_FLAG;

/* Parâmetros do controlador*/

/* Motor Esquerdo */
#define KP_L 1.5 
#define KI_L 0.05   
#define KD_L 0.1   
#define TICKS_TO_RADS_LEFT (3.141592*2*(1000/PERIOD))/1321 // Fator de conversão entre ticks do encoder para RPM

/* Motor Direito */
#define KP_R 1.5    
#define KI_R 0.05     
#define KD_R 0.1  
#define TICKS_TO_RADS_RIGHT (3.141592*2*(1000/PERIOD))/1321 // Fator de conversão entre ticks do encoder para RPM

/* Limites para ação de controle conforme resolução do PWM */
#define Max_Output LEDC_MAX_DUTY
#define Min_Output -LEDC_MAX_DUTY
/* Limites para evitar integral windup */
#define Max_integral 200
#define Min_integral -200

#define PERIOD 50  // Período do loop de controle (ms)

typedef enum {
    PID_RIGHT = 0,
    PID_LEFT = 1
} pid_side_t;

/* Funções Macro */
#define PID_SIDE_KP(SIDE) ((SIDE) == PID_LEFT ? KP_L : KP_R)
#define PID_SIDE_KI(SIDE) ((SIDE) == PID_LEFT ? KI_L : KI_R)
#define PID_SIDE_KD(SIDE) ((SIDE) == PID_LEFT ? KD_L : KD_R)

#define PID_TICKS_TO_RADS(SIDE) ((SIDE) == PID_LEFT ? TICKS_TO_RADS_LEFT : TICKS_TO_RADS_RIGHT)

/* Protótipos de Funções */
pid_ctrl_block_handle_t init_pid(pid_side_t side);
esp_err_t pid_calculate(pid_ctrl_block_handle_t pid_block_L, pid_ctrl_block_handle_t pid_block_R);

#endif