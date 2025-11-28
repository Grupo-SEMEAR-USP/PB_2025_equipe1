#ifndef UART_COMMUNICATION_H
#define UART_COMMUNICATION_H

/* Importe de bibliotecas */
#include "esp_err.h"
#include "def.h"

/* Protótipos das Funções */
esp_err_t uart_init();
esp_err_t uart_send_frame(data_t *cmd);
void uart_send(float left_data, float right_data);
void uart_read();

#endif // UART_COMMUNICATION_H