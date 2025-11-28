#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"      
#include "esp_err.h"

#include "def.h"           
#include "tasks.h"

// #include "h_bridge.h"
// #include "encoder.h"

static const char *TAG = "APP_MAIN";

void app_main(void) {
    ESP_LOGI(TAG, "Iniciando tarefas...");
    esp_err_t ret = init_tasks();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na inicialização das Tasks.");
        return;
    }

    // init_h_bridge(MOTOR_RIGHT);
    // init_h_bridge(MOTOR_LEFT);
    // pcnt_unit_handle_t encoder_unit_r = init_encoder(ENC_RIGHT);
    // pcnt_unit_handle_t encoder_unit_l = init_encoder(ENC_LEFT);
    while(1) {
        // update_motor(MOTOR_LEFT, -LEDC_MAX_DUTY);
        // update_motor(MOTOR_RIGHT, -LEDC_MAX_DUTY);
        // G_ENC_L = get_encoder_position(encoder_unit_l);
        // G_ENC_R = get_encoder_position(encoder_unit_r);
        // ESP_LOGI(TAG, "Encoder L: %d | Encoder R: %d", G_ENC_L, G_ENC_R);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}