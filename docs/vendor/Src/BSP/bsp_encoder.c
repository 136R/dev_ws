#include "BSP/bsp_encoder.h"
#include "APP/config.h"

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

#define ENCODER1_TIMER	TIM4
#define ENCODER2_TIMER	TIM1
#define ENCODER3_TIMER	TIM3
#define ENCODER4_TIMER	TIM8


#define ENCODER1_Handle	htim4
#define ENCODER2_Handle	htim1
#define ENCODER3_Handle	htim3
#define ENCODER4_Handle	htim8

#define ENCODER_TIM_PERIOD	50000

volatile int32_t uiEncoder1OverflowCount = 0;
volatile int32_t uiEncoder2OverflowCount = 0;
volatile int32_t uiEncoder3OverflowCount = 0;
volatile int32_t uiEncoder4OverflowCount = 0;

extern void vServoCallback(TIM_HandleTypeDef *htim);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	vServoCallback(htim);
	if(htim->Instance == ENCODER1_TIMER){
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			uiEncoder1OverflowCount--; 
		else
			uiEncoder1OverflowCount++;
	}
	else if(htim->Instance == ENCODER2_TIMER){
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			uiEncoder2OverflowCount--; 
		else
			uiEncoder2OverflowCount++;
	}
	else if(htim->Instance == ENCODER3_TIMER){
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			uiEncoder3OverflowCount--; 
		else
			uiEncoder3OverflowCount++;
	}
	else if(htim->Instance == ENCODER4_TIMER){
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			uiEncoder4OverflowCount--; 
		else
			uiEncoder4OverflowCount++;
	}
	else{
		__nop();
	}	
}

int32_t iGetEncoderValue(encoder_t xEncoder){
	int32_t iEncoderValue = 0;
	switch(xEncoder){
		case ENCODER1:
			iEncoderValue = -(ENCODER_TIM_PERIOD*uiEncoder1OverflowCount + __HAL_TIM_GET_COUNTER(&ENCODER1_Handle));
		break;
		case ENCODER2:
			iEncoderValue = -(ENCODER_TIM_PERIOD*uiEncoder2OverflowCount + __HAL_TIM_GET_COUNTER(&ENCODER2_Handle));
			break;
		case ENCODER3:
			iEncoderValue = (ENCODER_TIM_PERIOD*uiEncoder3OverflowCount + __HAL_TIM_GET_COUNTER(&ENCODER3_Handle));
			break;
		case ENCODER4:
			iEncoderValue = -(ENCODER_TIM_PERIOD*uiEncoder4OverflowCount + __HAL_TIM_GET_COUNTER(&ENCODER4_Handle));
			break;		
		default:
			break;
	}
	return iEncoderValue;
}

void vEncoderBspInit(void)
{
	__HAL_TIM_CLEAR_IT(&ENCODER1_Handle, TIM_IT_UPDATE); 
	__HAL_TIM_URS_ENABLE(&ENCODER1_Handle);               
	__HAL_TIM_ENABLE_IT(&ENCODER1_Handle,TIM_IT_UPDATE);

	__HAL_TIM_CLEAR_IT(&ENCODER2_Handle, TIM_IT_UPDATE); 
	__HAL_TIM_URS_ENABLE(&ENCODER2_Handle);               
	__HAL_TIM_ENABLE_IT(&ENCODER2_Handle,TIM_IT_UPDATE);

	__HAL_TIM_CLEAR_IT(&ENCODER3_Handle, TIM_IT_UPDATE); 
	__HAL_TIM_URS_ENABLE(&ENCODER3_Handle);               
	__HAL_TIM_ENABLE_IT(&ENCODER3_Handle,TIM_IT_UPDATE);

	__HAL_TIM_CLEAR_IT(&ENCODER4_Handle, TIM_IT_UPDATE); 
	__HAL_TIM_URS_ENABLE(&ENCODER4_Handle);               
	__HAL_TIM_ENABLE_IT(&ENCODER4_Handle,TIM_IT_UPDATE);
	
	HAL_TIM_Encoder_Start(&ENCODER1_Handle,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&ENCODER2_Handle,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&ENCODER3_Handle,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&ENCODER4_Handle,TIM_CHANNEL_ALL);
}
