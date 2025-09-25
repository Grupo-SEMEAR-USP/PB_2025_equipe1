#ifndef ENCODER_H
#define ENCODER_H

/* Importe de bibliotecas */
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_log.h"

/* Definição do enum para identificar o lado */
typedef enum {
    ENC_RIGHT = 0,
    ENC_LEFT = 1
} encoder_side_t;

/* Limites do PCNT */
#define PCNT_HIGH_LIMIT 10000
#define PCNT_LOW_LIMIT  -10000

/* Definições do encoder direito */
#define ENCODER_GPIO_RA GPIO_NUM_4
#define ENCODER_GPIO_RB GPIO_NUM_18

/* Definições do encoder esquerdo */
#define ENCODER_GPIO_LA GPIO_NUM_13
#define ENCODER_GPIO_LB GPIO_NUM_14

#define ENCODER_A(SIDE) ((SIDE) == (ENC_LEFT) ? ENCODER_GPIO_LA : ENCODER_GPIO_RA)
#define ENCODER_B(SIDE) ((SIDE) == (ENC_LEFT) ? ENCODER_GPIO_LB : ENCODER_GPIO_RB)


// Protótipos das funções
esp_err_t init_encoder(encoder_side_t side);
int get_encoder_count(encoder_side_t side);
void reset_encoder_count(encoder_side_t side);

#endif