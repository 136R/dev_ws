#include "app/encoder.h"
#include "app/robot_config.h"
#include "tim.h"

//extern TIM_HandleTypeDef htim4;   /* LEFT  encoder */
//extern TIM_HandleTypeDef htim1;   /* RIGHT encoder */
//
//static TIM_HandleTypeDef *const enc_htim[] = {&htim4, &htim1};

extern TIM_HandleTypeDef htim1;   /* LEFT encoder */
extern TIM_HandleTypeDef htim4;   /* RIGHT encoder */

static TIM_HandleTypeDef *const enc_htim[] = {&htim1, &htim4};
static const uint8_t enc_invert[] = {ENC_LEFT_DIR_INVERT, ENC_RIGHT_DIR_INVERT};

static uint16_t last_cnt[2];
static int64_t  total_cnt[2];

void encoder_init(void)
{
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

    last_cnt[MOTOR_LEFT]  = __HAL_TIM_GET_COUNTER(&htim1);
    last_cnt[MOTOR_RIGHT] = __HAL_TIM_GET_COUNTER(&htim4);

    total_cnt[MOTOR_LEFT]  = 0;
    total_cnt[MOTOR_RIGHT] = 0;
}

int32_t encoder_get_delta(uint8_t id)
{
    if (id > MOTOR_RIGHT) return 0;

    uint16_t now = __HAL_TIM_GET_COUNTER(enc_htim[id]);
    /* int16_t cast handles 16-bit wrap-around automatically */
    int32_t delta = (int16_t)(now - last_cnt[id]);
    last_cnt[id] = now;

    if (enc_invert[id]) delta = -delta;

    total_cnt[id] += delta;
    return delta;
}

int64_t encoder_get_total(uint8_t id)
{
    if (id > MOTOR_RIGHT) return 0;
    return total_cnt[id];
}

void encoder_reset_all(void)
{
    last_cnt[MOTOR_LEFT]  = __HAL_TIM_GET_COUNTER(&htim1);
    last_cnt[MOTOR_RIGHT] = __HAL_TIM_GET_COUNTER(&htim4);
    total_cnt[MOTOR_LEFT]  = 0;
    total_cnt[MOTOR_RIGHT] = 0;
}
