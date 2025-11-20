#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"      
#include "esp_err.h"

#include "def.h"           
#include "tasks.h"

static const char *TAG = "APP_MAIN";

void app_main(void) {
    ESP_LOGI(TAG, "Iniciando tarefas...");
    esp_err_t ret = init_tasks();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na inicialização das Tasks.");
        return;
    }

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}