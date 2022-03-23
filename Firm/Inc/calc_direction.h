/*
 *  			calc_direction.c.h
 *
 *  Copyright (c) 2022
 *  K.Watanabe,Crescent
 *  Released under the MIT license
 *  http://opensource.org/licenses/mit-license.php
 *
 *  Refer to the algorithm:
 *  https://github.com/wooters/miniDSP
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CALC_DIRECTION_H
#define __CALC_DIRECTION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include system header files -----------------------------------------------*/
/* Include user header files -------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define SOUND_ACTIVITY	30

//#define FACTORY_CHECK_MODE

/* Exported types ------------------------------------------------------------*/
enum{
	ENUM_REG_DEG_H=0,
	ENUM_REG_DEG_L,
	ENUM_REG_DEG_ORG_H,
	ENUM_REG_DEG_ORG_L,
	ENUM_REG_ACT_H,
	ENUM_REG_ACT_L,
	ENUM_REG_REFRESH,
	ENUM_REG_ID			//ID for device
};
/* Exported enum tag ---------------------------------------------------------*/
/* Exported struct/union tag -------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
void prepareWindow(float samplingRate);
float calcAngle();
void getAdcValue();
void setActivityValue(float fvalue);

#ifdef __cplusplus
}
#endif

#endif /* __CALC_DIRECTION_H */
/***************************************************************END OF FILE****/
