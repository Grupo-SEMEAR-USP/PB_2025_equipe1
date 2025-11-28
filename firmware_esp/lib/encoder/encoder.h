#ifndef ENCODER_H
#define ENCODER_H

/* Importe de bibliotecas */
#include "driver/pulse_cnt.h"

/* Identificador de lado */
typedef enum {
    ENC_RIGHT = 0,
    ENC_LEFT = 1
} encoder_side_t;

/* Protótipos das Funções */
pcnt_unit_handle_t init_encoder(encoder_side_t side);
int get_encoder_vel(pcnt_unit_handle_t handler);
int get_encoder_position(pcnt_unit_handle_t handler);

#endif // ENCODER_H