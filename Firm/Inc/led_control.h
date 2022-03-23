/*
 *  			led_control.h
 *
 *  Copyright (c) 2022
 *  K.Watanabe,Crescent
 *  Released under the MIT license
 *  http://opensource.org/licenses/mit-license.php
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_CONTROL_H
#define __LED_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include system header files -----------------------------------------------*/
/* Include user header files -------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
#define LEDMAX 	300
#define LEDMIN	1024
#define LEDGRAD	120

/* Exported types ------------------------------------------------------------*/
/* Exported enum tag ---------------------------------------------------------*/
/* Exported struct/union tag -------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern uint16_t	ledValue[8];

/* Exported function prototypes ----------------------------------------------*/
void initLed();
void setLedValue();
void updateLedValue(float deg);


#ifdef __cplusplus
}
#endif

#endif /* __LED_CONTROL_H */
/***************************************************************END OF FILE****/
