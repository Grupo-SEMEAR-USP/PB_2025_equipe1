#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Includes dos seus módulos de hardware, controle e comunicação
#include "h_bridge.h"
#include "encoder.h"
#include "pid.h"
#include "cominicacao.h"

// Estrutura com parâmetros para task de controle
typedef struct {
    pid_ctrl_block_handle_t pid_l_handle;
    pid_ctrl_block_handle_t pid_r_handle;
} control_task_params_t;

static control_task_params_t task_params;

void pid_task(void *pvParameters) {
    // Recebe o ponteiro genérico e o converte para o tipo da struct
    control_task_params_t *params = (control_task_params_t *)pvParameters;

    // Chama a função do pid_calculate
    pid_calculate(params->pid_l_handle, params->pid_r_handle);
}

void com_task(void *pvParameters){
while (1)
{
    char data[1024];
    read_data(data);
    TARGET_VALUE_L = atof(data);
    vTaskDelay(10/portTICK_PERIOD_MS);
    ESP_LOGI("TARGET","%f",TARGET_VALUE_L);
}
}

void app_main(void)
{

    // Inicialização dos Módulos de Hardware
    init_gpio_bridge();
    init_pwm_bridge();
    ESP_LOGI("APP_MAIN", "Ponte H inicializada.");
    
    
    ESP_ERROR_CHECK(init_encoder(ENC_LEFT));
    ESP_ERROR_CHECK(init_encoder(ENC_RIGHT));
    ESP_LOGI("APP_MAIN", "Encoders inicializados.");
    
    // Inicialização dos Módulos de Controle
    pid_ctrl_block_handle_t pid_left_handle = init_pid(PID_LEFT);
    pid_ctrl_block_handle_t pid_right_handle = init_pid(PID_RIGHT);
    ESP_LOGI("APP_MAIN", "Controladores PID inicializados.");
    
    //Inicializa Comunicação serial
    init_com();

    // Preparação dos Parâmetros para a Task de Controle
    task_params.pid_l_handle = pid_left_handle;
    task_params.pid_r_handle = pid_right_handle;

    // Criação das Tasks do FreeRTOS
    xTaskCreate(
        pid_task,               // A função "ponte" para a lógica do PID
        "PID Control Task",     // Nome da task para depuração
        4096,                   // Tamanho da pilha (stack size) em words
        &task_params,           // Ponteiro para os nossos parâmetros
        10,                     // Prioridade (mais alta que a de sequenciamento)
        NULL                    // Handle da task (não precisamos dele aqui)
    );
    // Criação das Tasks do FreeRTOS
    xTaskCreate(
        com_task,               // A função "ponte" para a lógica do PID
        "Comunication task",     // Nome da task para depuração
        4096,                   // Tamanho da pilha (stack size) em words
        NULL,           // Ponteiro para os nossos parâmetros
        10,                     // Prioridade (mais alta que a de sequenciamento)
        NULL                    // Handle da task (não precisamos dele aqui)
    );

    ESP_LOGI("APP_MAIN", "Sistema inicializado com sucesso. Tasks de controle e sequenciamento em execução.");
}