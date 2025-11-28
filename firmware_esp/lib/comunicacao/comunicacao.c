#include "cominicacao.h"


void init_com(){
    //Instalando O Driver UART
    QueueHandle_t queue_handle;
    ESP_ERROR_CHECK(uart_driver_install(PORT_NUM,BUFFER_SIZE,BUFFER_SIZE,10,&queue_handle,0));
    
    //configurando Parametros da Comunicação UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(PORT_NUM, &uart_config));
}

/**
 * 
 *@brief Recebe uma string e envia ela para porta serial. 
 *
 *@param string String que sera enviada para porta serial
 *@return Não retorna nada para o usuario
 *
 *@note Prefereivel usar ESP_LOGI
*/
void send_data(char* string){
uart_write_bytes(PORT_NUM, string, strlen(string));
uart_write_bytes(PORT_NUM,"\n", 1);
}

/**
 * 
 *@brief Recebe um Espaço pra escrever uma string de ate 1024 caracteres e Prenche esse espaço 
 * 
 *A função read_data lê dados da serial e os armazena em um buffer interno(placeholder), garantindo que os dados sejam terminados em nulo, e os copia para o buffer externo fornecido, que deve ter espaço para até 1.024 caracteres. 
 *Ela usa uart_read_bytes para ler os dados da serial e assume que o chamador fornece um endereço de memória 
 *válido para o parâmetro data.
 *@param data Buffer externo cujo será armazernado a leitura final da serial
*/
void read_data(char *data) {
    uint8_t placeholder[BUFFER_SIZE];
    int len = uart_read_bytes(PORT_NUM, placeholder, BUFFER_SIZE - 1, 100 / portTICK_PERIOD_MS);

    if (len > 0) {
        placeholder[len] = '\0';               // garante string válida
        strcpy(data, (char*)placeholder);    // copia para o buffer externo
    }
}
