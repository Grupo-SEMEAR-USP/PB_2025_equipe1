#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Includes dos seus módulos de hardware e controle
#include "h_bridge.h"
#include "encoder.h"
#include "pid.h"

#define SPEED_FORWARD 5.0f  // Velocidade para andar para frente (ex: 5 rad/s)
#define SPEED_TURN    3.0f  // Velocidade para girar (ex: 3 rad/s)

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


void motor_sequence_task(void *pvParameters) {
    
    printf("Iniciando a tarefa de sequência de motores...\n");

    while (1) {
        TARGET_VALUE_L = SPEED_FORWARD;
        TARGET_VALUE_R = SPEED_FORWARD;
        
        
        // // --- 1. ANDAR PARA FRENTE ---
        // printf("MOVIMENTO: Para frente\n");
        // TARGET_VALUE_L = SPEED_FORWARD;  // Define a velocidade alvo para o motor esquerdo
        // TARGET_VALUE_R = SPEED_FORWARD;  // Define a velocidade alvo para o motor direito
        // BREAK_FLAG = false;              // Libera os freios
        // vTaskDelay(pdMS_TO_TICKS(2000)); // Mantém o movimento por 2000 ms (2 segundos)

        // // --- 2. PARAR ---
        // printf("MOVIMENTO: Parar\n");
        // TARGET_VALUE_L = 0.0f;
        // TARGET_VALUE_R = 0.0f;
        // BREAK_FLAG = true;               // Ativa o freio para uma parada mais firme
        // vTaskDelay(pdMS_TO_TICKS(1000)); // Fica parado por 1000 ms (1 segundo)

        // // --- 3. GIRAR PARA A DIREITA (no próprio eixo) ---
        // printf("MOVIMENTO: Girar para a direita\n");
        // TARGET_VALUE_L = SPEED_TURN;     // Roda esquerda para frente
        // TARGET_VALUE_R = -SPEED_TURN;    // Roda direita para trás
        // BREAK_FLAG = false;
        // vTaskDelay(pdMS_TO_TICKS(1500)); // Mantém o giro por 1.5 segundos

        // // --- 4. PARAR NOVAMENTE ---
        // printf("MOVIMENTO: Parar e reiniciar ciclo\n");
        // TARGET_VALUE_L = 0.0f;
        // TARGET_VALUE_R = 0.0f;
        // BREAK_FLAG = true;
        // vTaskDelay(pdMS_TO_TICKS(2000)); // Pausa antes de repetir a sequência
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

    xTaskCreate(
        motor_sequence_task,    // A função que gera os comandos de velocidade
        "Motor Sequence Task",
        4096,
        NULL,                   // Não precisa de parâmetros
        5,                      // Prioridade (mais baixa)
        NULL
    );

    ESP_LOGI("APP_MAIN", "Sistema inicializado com sucesso. Tasks de controle e sequenciamento em execução.");
}