#include "pid.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Definições essenciais */
float TARGET_VALUE_L = 0;
float TARGET_VALUE_R = 0;

int ENCODER_READ_L = 0;
int ENCODER_READ_R = 0;

float RADS_L = 0;
float RADS_R = 0;

float LEFT_PWM_VALUE = 0;
float RIGHT_PWM_VALUE = 0;

bool BREAK_FLAG = false;

pid_ctrl_block_handle_t init_pid(pid_side_t side) {

  pid_ctrl_config_t config_pid; 
  pid_ctrl_block_handle_t pid_block; 

  pid_ctrl_parameter_t values_pid = {
    .kd = PID_SIDE_KD(side),
    .kp = PID_SIDE_KP(side),
    .ki = PID_SIDE_KI(side),
    .min_integral = Min_integral,
    .max_integral = Max_integral,
    .min_output = Min_Output,
    .max_output = Max_Output,
    .cal_type = PID_CAL_TYPE_INCREMENTAL,
  };

  config_pid.init_param = values_pid;

  ESP_ERROR_CHECK(pid_new_control_block(&config_pid, &pid_block));
  ESP_ERROR_CHECK(pid_update_parameters(pid_block, &values_pid));

  return pid_block;
}

void PWM_limit(float *PWM) {
  *PWM = (*PWM > Max_Output) ? Max_Output : *PWM;
  *PWM = (*PWM < Min_Output) ? Min_Output : *PWM;
}

esp_err_t pid_calculate(pid_ctrl_block_handle_t pid_block_L, pid_ctrl_block_handle_t pid_block_R) {
  float controll_pid_LEFT = 0;
  float controll_pid_RIGHT = 0;

   while(1) {

    ENCODER_READ_L = get_encoder_count(ENC_LEFT);
    ENCODER_READ_R = get_encoder_count(ENC_RIGHT);

    RADS_L = ENCODER_READ_L * PID_TICKS_TO_RADS(PID_LEFT);
    RADS_R = ENCODER_READ_R * PID_TICKS_TO_RADS(PID_RIGHT);

    if(TARGET_VALUE_L == 0 && TARGET_VALUE_R == 0 && BREAK_FLAG) {
      LEFT_PWM_VALUE = 0;
      RIGHT_PWM_VALUE = 0;
    }

    else{
      float error_motor_LEFT = (TARGET_VALUE_L - RADS_L);
      float error_motor_RIGHT = (TARGET_VALUE_R - RADS_R);

      pid_compute(pid_block_L, error_motor_LEFT, &controll_pid_LEFT);
      pid_compute(pid_block_R, error_motor_RIGHT, &controll_pid_RIGHT);

      LEFT_PWM_VALUE += controll_pid_LEFT;
      RIGHT_PWM_VALUE += controll_pid_RIGHT;

      PWM_limit(&LEFT_PWM_VALUE);
      PWM_limit(&RIGHT_PWM_VALUE);
    }

      // ESP_LOGI("PID_DEBUG_L", "Target:%.2f | Atual:%.2f | Erro:%.2f | PWM:%.0f",
      //    TARGET_VALUE_L, RADS_L, (TARGET_VALUE_L - RADS_L), LEFT_PWM_VALUE);
      // ESP_LOGI("PID_DEBUG_R", "Target:%.2f | Atual:%.2f | Erro:%.2f | PWM:%.0f",
      //    TARGET_VALUE_R, RADS_R, (TARGET_VALUE_R - RADS_R), RIGHT_PWM_VALUE);
    

      update_motor(MOTOR_LEFT, LEFT_PWM_VALUE);
      update_motor(MOTOR_RIGHT, RIGHT_PWM_VALUE);

      controll_pid_LEFT = 0;
      controll_pid_RIGHT = 0;

      vTaskDelay(pdMS_TO_TICKS(PERIOD));
    
   }

  return ESP_OK;
}