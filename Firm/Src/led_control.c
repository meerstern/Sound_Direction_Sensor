/*
 *  			led_control.c
 *
 *  Copyright (c) 2022
 *  K.Watanabe,Crescent
 *  Released under the MIT license
 *  http://opensource.org/licenses/mit-license.php
 *
 */

/* Include system header files -----------------------------------------------*/
#include "main.h"

/* Include user header files -------------------------------------------------*/
#include "led_control.h"

/* Imported variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

uint16_t	ledValue[8]={300};
/* Private function macro ----------------------------------------------------*/
/* Private enum tag ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void initLed()
{
    ledValue[0] = LEDMAX;
    ledValue[1] = LEDMAX;
    ledValue[2] = LEDMAX;
    ledValue[3] = LEDMAX;
    ledValue[4] = LEDMAX;
    ledValue[5] = LEDMAX;
    ledValue[6] = LEDMAX;
	ledValue[7] = LEDMAX;
    setLedValue();

    HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);//MIC1
    HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_2);//MIC1_2
    HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_3);//MIC2
    HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_4);//MIC2_3
    HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);//MIC3
    HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_2);//MIC3_4
    HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_3);//MIC4
    HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_4);//MIC4_1

    HAL_Delay(100);

    ledValue[0] = LEDMIN;
    ledValue[1] = LEDMIN;
    ledValue[2] = LEDMIN;
    ledValue[3] = LEDMIN;
    ledValue[4] = LEDMIN;
    ledValue[5] = LEDMIN;
    ledValue[6] = LEDMIN;
	ledValue[7] = LEDMIN;
    setLedValue();
}


void setLedValue()
{
    //Check Overflow
	if(ledValue[0]>LEDMIN)
    	ledValue[0]=LEDMIN;
    if(ledValue[1]>LEDMIN)
    	ledValue[1]=LEDMIN;
    if(ledValue[2]>LEDMIN)
    	ledValue[2]=LEDMIN;
    if(ledValue[3]>LEDMIN)
    	ledValue[3]=LEDMIN;
    if(ledValue[4]>LEDMIN)
    	ledValue[4]=LEDMIN;
    if(ledValue[5]>LEDMIN)
    	ledValue[5]=LEDMIN;
    if(ledValue[6]>LEDMIN)
    	ledValue[6]=LEDMIN;
    if(ledValue[7]>LEDMIN)
    	ledValue[7]=LEDMIN;

    //Set Pwm Value
	__HAL_TIM_SET_COMPARE( &htim3, TIM_CHANNEL_1, ledValue[0]);//MIC1
    __HAL_TIM_SET_COMPARE( &htim3, TIM_CHANNEL_2, ledValue[1]);//MIC1_2
    __HAL_TIM_SET_COMPARE( &htim3, TIM_CHANNEL_3, ledValue[2]);//MIC2
    __HAL_TIM_SET_COMPARE( &htim3, TIM_CHANNEL_4, ledValue[3]);//MIC2_3
    __HAL_TIM_SET_COMPARE( &htim4, TIM_CHANNEL_1, ledValue[4]);//MIC3
    __HAL_TIM_SET_COMPARE( &htim4, TIM_CHANNEL_2, ledValue[5]);//MIC3_4
    __HAL_TIM_SET_COMPARE( &htim4, TIM_CHANNEL_3, ledValue[6]);//MIC4
    __HAL_TIM_SET_COMPARE( &htim4, TIM_CHANNEL_4, ledValue[7]);//MIC4_1

}


void updateLedValue(float deg)
{
	if( 0 < deg && deg < 22.5 )
	{
	    ledValue[0] = LEDMAX;
	    ledValue[1]+= LEDGRAD;
	    ledValue[2]+= LEDGRAD;
	    ledValue[3]+= LEDGRAD;
	    ledValue[4]+= LEDGRAD;
	    ledValue[5]+= LEDGRAD;
	    ledValue[6]+= LEDGRAD;
		ledValue[7]+= LEDGRAD;
	}
	else if( 22.5 < deg && deg < 45+22.5 )
	{
	    ledValue[0]+= LEDGRAD;
	    ledValue[1] = LEDMAX;
	    ledValue[2]+= LEDGRAD;
	    ledValue[3]+= LEDGRAD;
	    ledValue[4]+= LEDGRAD;
	    ledValue[5]+= LEDGRAD;
	    ledValue[6]+= LEDGRAD;
		ledValue[7]+= LEDGRAD;
	}
	else if( 45+22.5 < deg && deg < 90+22.5 )
	{

	    ledValue[0]+= LEDGRAD;
	    ledValue[1]+= LEDGRAD;
	    ledValue[2] = LEDMAX;
	    ledValue[3]+= LEDGRAD;
	    ledValue[4]+= LEDGRAD;
	    ledValue[5]+= LEDGRAD;
	    ledValue[6]+= LEDGRAD;
		ledValue[7]+= LEDGRAD;

	}
	else if( 90+22.5 < deg && deg < 135+22.5 )
	{
	    ledValue[0]+= LEDGRAD;
	    ledValue[1]+= LEDGRAD;
	    ledValue[2]+= LEDGRAD;
	    ledValue[3] = LEDMAX;
	    ledValue[4]+= LEDGRAD;
	    ledValue[5]+= LEDGRAD;
	    ledValue[6]+= LEDGRAD;
		ledValue[7]+= LEDGRAD;

	}
	else if( 135+22.5 < deg && deg < 180+22.5 )
	{

	    ledValue[0]+= LEDGRAD;
	    ledValue[1]+= LEDGRAD;
	    ledValue[2]+= LEDGRAD;
	    ledValue[3]+= LEDGRAD;
	    ledValue[4] = LEDMAX;
	    ledValue[5]+= LEDGRAD;
	    ledValue[6]+= LEDGRAD;
		ledValue[7]+= LEDGRAD;

	}
	else if( 180+22.5 < deg && deg < 225+22.5 )
	{

	    ledValue[0]+= LEDGRAD;
	    ledValue[1]+= LEDGRAD;
	    ledValue[2]+= LEDGRAD;
	    ledValue[3]+= LEDGRAD;
	    ledValue[4]+= LEDGRAD;
	    ledValue[5] = LEDMAX;
	    ledValue[6]+= LEDGRAD;
		ledValue[7]+= LEDGRAD;

	}
	else if( 225+22.5 < deg && deg < 270+22.5 )
	{

	    ledValue[0]+= LEDGRAD;
	    ledValue[1]+= LEDGRAD;
	    ledValue[2]+= LEDGRAD;
	    ledValue[3]+= LEDGRAD;
	    ledValue[4]+= LEDGRAD;
	    ledValue[5]+= LEDGRAD;
	    ledValue[6] = LEDMAX;
		ledValue[7]+= LEDGRAD;

	}
	else if( 270+22.5 < deg && deg < 315+22.5 )
	{
	    ledValue[0]+= LEDGRAD;
	    ledValue[1]+= LEDGRAD;
	    ledValue[2]+= LEDGRAD;
	    ledValue[3]+= LEDGRAD;
	    ledValue[4]+= LEDGRAD;
	    ledValue[5]+= LEDGRAD;
	    ledValue[6]+= LEDGRAD;
		ledValue[7] = LEDMAX;
	}
	else if( deg > 315+22.5 )
	{
	    ledValue[0] = LEDMAX;
	    ledValue[1]+= LEDGRAD;
	    ledValue[2]+= LEDGRAD;
	    ledValue[3]+= LEDGRAD;
	    ledValue[4]+= LEDGRAD;
	    ledValue[5]+= LEDGRAD;
	    ledValue[6]+= LEDGRAD;
		ledValue[7]+= LEDGRAD;

	}
	else
	{
	    ledValue[0]+= LEDGRAD;
	    ledValue[1]+= LEDGRAD;
	    ledValue[2]+= LEDGRAD;
	    ledValue[3]+= LEDGRAD;
	    ledValue[4]+= LEDGRAD;
	    ledValue[5]+= LEDGRAD;
	    ledValue[6]+= LEDGRAD;
		ledValue[7]+= LEDGRAD;
	}

    setLedValue();
}

/* Private functions ---------------------------------------------------------*/

/***************************************************************END OF FILE****/
