#include "encoder.h"

// Variável global para o handle das unidades PCNT
static pcnt_unit_handle_t right_encoder = NULL;
static pcnt_unit_handle_t left_encoder = NULL;

#define ENC_SIDE(SIDE) ((SIDE) == ENC_LEFT ? left_encoder : right_encoder)

esp_err_t init_encoder(encoder_side_t side){
    pcnt_unit_handle_t side_handler = NULL;
    // Define o planejamento do gerente da contagem do PCNT
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &side_handler)); // Atribui planejamento ao handle

    // Filtro de ruído
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000, // Filtra qualquer pulso menor que 1000 nanosegundos (1us)
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(side_handler, &filter_config));

    /// Definindo ambos os canais para implementar uma decodificação de quadratura x4
    // Configuração do canal A
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_A(side), // Pino de borda
        .level_gpio_num = ENCODER_B(side), // Pino de nível (para direção)
    };
    pcnt_channel_handle_t encoder_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(side_handler, &chan_a_config, &encoder_chan_a)); // Cria o Canal A

    // Configuração do canal B
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_B(side), // Pino de borda
        .level_gpio_num = ENCODER_A(side), // Pino de nível (para direção)
    };
    pcnt_channel_handle_t encoder_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(side_handler, &chan_b_config, &encoder_chan_b)); // Cria o Canal B

    // Define a lógica de contagem
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(encoder_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(encoder_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(encoder_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(encoder_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Inicia o hardware do PCNT
    ESP_ERROR_CHECK(pcnt_unit_enable(side_handler));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(side_handler));
    ESP_ERROR_CHECK(pcnt_unit_start(side_handler));

    // Atribui o handle à variável global correta
    if (side == ENC_LEFT) {
        left_encoder = side_handler;
    } else {
        right_encoder = side_handler;
    }

    return ESP_OK;
}

// Função para obter a contagem atual
int get_encoder_count(encoder_side_t side){
    int count = 0; // Definindo variável temporária para armazenar a contagem
    ESP_ERROR_CHECK(pcnt_unit_get_count(ENC_SIDE(side), &count));
    reset_encoder_count(side);
    return count;
}

// Função para zerar o contador
void reset_encoder_count(encoder_side_t side){
    pcnt_unit_clear_count(ENC_SIDE(side));
}