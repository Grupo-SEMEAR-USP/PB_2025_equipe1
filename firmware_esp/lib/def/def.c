#include "def.h"

/* Definições Iniciais */
float G_TARGET_L = 0;
float G_TARGET_R = 0;

int G_ENC_L = 0;
int G_ENC_R = 0;

float G_PWM_L = 0;
float G_PWM_R = 0;

float G_RADS_L = 0;
float G_RADS_R = 0;

// PID stop on zero
bool BREAK_FLAG = false;

data_t G_CMD = { .left_data = 0.0, .right_data = 0.0};