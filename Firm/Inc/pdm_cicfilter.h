/*
 *  			pdm_cicfilter.h
 *
 *  Copyright (c) 2022
 *  K.Watanabe,Crescent
 *  Released under the MIT license
 *  http://opensource.org/licenses/mit-license.php
 *
 *  Refer to the algorithm:
 *  https://github.com/y2kblog/NUCLEO-L476RG_DFSDM_PDM-Mic
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PDM_CICFILTER_H
#define __PDM_CICFILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include system header files -----------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* Include user header files -------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* Private macro -------------------------------------------------------------*/
// CIC filter setting
#define DECIMATION_M        20
#define PDM_SAMPLE_SIZE     256//512

/* Exported types ------------------------------------------------------------*/
/* Exported enum tag ---------------------------------------------------------*/
/* Exported struct/union tag -------------------------------------------------*/
struct CicFilter_t
{
    uint8_t order;
    uint32_t decimation;
    int32_t *out_i;
    int32_t *out_c;
    int32_t *z1_c;
};

/* Exported variables --------------------------------------------------------*/
extern volatile bool needs_CopyPDMbits;
extern volatile bool isCalled_PDM_DMA_Callback;

extern struct CicFilter_t PDM1_st;
extern struct CicFilter_t PDM2_st;
extern struct CicFilter_t PDM3_st;
extern struct CicFilter_t PDM4_st;

extern uint8_t PDM_Buff1[PDM_SAMPLE_SIZE * DECIMATION_M / 8 * 2];
extern uint8_t PDM_Buff2[PDM_SAMPLE_SIZE * DECIMATION_M / 8 * 2];
extern uint8_t PDM_Buff3[PDM_SAMPLE_SIZE * DECIMATION_M / 8 * 2];
extern uint8_t PDM_Buff4[PDM_SAMPLE_SIZE * DECIMATION_M / 8 * 2];

extern uint8_t PDM_RawBits1[PDM_SAMPLE_SIZE * DECIMATION_M / 8];
extern uint8_t PDM_RawBits2[PDM_SAMPLE_SIZE * DECIMATION_M / 8];
extern uint8_t PDM_RawBits3[PDM_SAMPLE_SIZE * DECIMATION_M / 8];
extern uint8_t PDM_RawBits4[PDM_SAMPLE_SIZE * DECIMATION_M / 8];

extern int32_t PDM_Filtered1_int32[PDM_SAMPLE_SIZE];
extern int32_t PDM_Filtered2_int32[PDM_SAMPLE_SIZE];
extern int32_t PDM_Filtered3_int32[PDM_SAMPLE_SIZE];
extern int32_t PDM_Filtered4_int32[PDM_SAMPLE_SIZE];


/* Exported function prototypes ----------------------------------------------*/
void intiCicFilterAll();
void initializeCicFilterStruct(uint8_t, uint32_t, struct CicFilter_t*);
void resetCicFilterStruct(struct CicFilter_t*);
void startPDM(void);
void enableDMA(void);
void disableDMA(void);
void stopDMA(void);
void executeCicFilterAll();
void executeCicFilter(uint8_t*, uint32_t, int32_t*, struct CicFilter_t*);
void finalizeCicFilterStruct(struct CicFilter_t*);
void clearBufferValue();

/***** Interrupt function prototypes *****/
void PDM_DMA_Callback(bool);

#ifdef __cplusplus
}
#endif

#endif /* __PDM_SOFTCICFILTER_H */
/***************************************************************END OF FILE****/
