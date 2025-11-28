//include guard
#ifndef COMUNICACAO_H
#define COMUNICACAO_H

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"

#define BUFFER_SIZE 1024
#define RX_PIN 16
#define TX_PIN 17
#define PORT_NUM UART_NUM_0
#define BOUNDRATE 11520

/**
 *@brief Inicializa a comunicação serial
*/
void init_com();
void read_data(char* values);
void send_data(char* string);





#endif