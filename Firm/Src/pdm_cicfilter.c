/*
 *  			pdm_cicfilter.c
 *
 *  Copyright (c) 2022
 *  K.Watanabe,Crescent
 *  Released under the MIT license
 *  http://opensource.org/licenses/mit-license.php
 *
 *  Refer to the algorithm:
 *  https://github.com/y2kblog/NUCLEO-L476RG_DFSDM_PDM-Mic
 */

/* Include system header files -----------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

/* Include user header files -------------------------------------------------*/
#include "pdm_cicfilter.h"

/* Imported variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;

/* Private function macro ----------------------------------------------------*/
/* Private enum tag ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

volatile bool needs_CopyPDMbits;
volatile bool isCalled_PDM_DMA_Callback;

struct CicFilter_t PDM1_st;
struct CicFilter_t PDM2_st;
struct CicFilter_t PDM3_st;
struct CicFilter_t PDM4_st;

uint8_t PDM_Buff1[PDM_SAMPLE_SIZE * DECIMATION_M / 8 * 2];     /* <Sample Size> * <Decimation of CIC filter> / <bits per 1Byte> * <for Double buffer> */
uint8_t PDM_Buff2[PDM_SAMPLE_SIZE * DECIMATION_M / 8 * 2];     /* <Sample Size> * <Decimation of CIC filter> / <bits per 1Byte> * <for Double buffer> */
uint8_t PDM_Buff3[PDM_SAMPLE_SIZE * DECIMATION_M / 8 * 2];     /* <Sample Size> * <Decimation of CIC filter> / <bits per 1Byte> * <for Double buffer> */
uint8_t PDM_Buff4[PDM_SAMPLE_SIZE * DECIMATION_M / 8 * 2];     /* <Sample Size> * <Decimation of CIC filter> / <bits per 1Byte> * <for Double buffer> */

uint8_t PDM_RawBits1[PDM_SAMPLE_SIZE * DECIMATION_M / 8];
uint8_t PDM_RawBits2[PDM_SAMPLE_SIZE * DECIMATION_M / 8];
uint8_t PDM_RawBits3[PDM_SAMPLE_SIZE * DECIMATION_M / 8];
uint8_t PDM_RawBits4[PDM_SAMPLE_SIZE * DECIMATION_M / 8];

int32_t PDM_Filtered1_int32[PDM_SAMPLE_SIZE];
int32_t PDM_Filtered2_int32[PDM_SAMPLE_SIZE];
int32_t PDM_Filtered3_int32[PDM_SAMPLE_SIZE];
int32_t PDM_Filtered4_int32[PDM_SAMPLE_SIZE];

/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void intiCicFilterAll()
{
    needs_CopyPDMbits = false;
    initializeCicFilterStruct(/* Order = */3, DECIMATION_M, &PDM1_st);
    initializeCicFilterStruct(/* Order = */3, DECIMATION_M, &PDM2_st);
    initializeCicFilterStruct(/* Order = */3, DECIMATION_M, &PDM3_st);
    initializeCicFilterStruct(/* Order = */3, DECIMATION_M, &PDM4_st);
}


void initializeCicFilterStruct(uint8_t CicFilterOrder, uint32_t CicFilterDecimation, struct CicFilter_t* st)
{
    if (CicFilterOrder == 0)
    {
        printf("Order of CIC filter must be over 0\r\n");
        return;
    }
    if(CicFilterDecimation == 0)
    {
        printf("Decimation of CIC filter must be over 0\r\n");
        return;
    }
    st->order = CicFilterOrder;
    st->decimation = CicFilterDecimation;
    st->out_i = (int32_t*) malloc(sizeof(int32_t) * st->order);
    st->out_c = (int32_t*) malloc(sizeof(int32_t) * st->order);
    st->z1_c  = (int32_t*) malloc(sizeof(int32_t) * st->order);
    
    if ((st->out_i == NULL) || (st->out_c == NULL) || (st->z1_c == NULL))
    {
        printf("CicFilterStruct malloc error\r\n");
        return;
    }
    resetCicFilterStruct(st);
}

void resetCicFilterStruct(struct CicFilter_t* st)
{
    if(st == NULL)
        return;
    
    for(uint8_t i = 0; i < st->order; i++)
    {
        *(st->out_i + i) = 0;
        *(st->out_c + i) = 0;
        *(st->z1_c  + i) = 0;
    }
}

void startPDM(void)
{
    HAL_StatusTypeDef Status;

    Status = HAL_SPI_Receive_DMA(&hspi4, (uint8_t *) PDM_Buff4, sizeof(PDM_Buff4) / sizeof(PDM_Buff4[0]));
    if (Status != HAL_OK)
        printf("HAL_SPI4_Receive_DMA Error = %d\r\n", Status);

    Status = HAL_SPI_Receive_DMA(&hspi3, (uint8_t *) PDM_Buff3, sizeof(PDM_Buff3) / sizeof(PDM_Buff3[0]));
    if (Status != HAL_OK)
        printf("HAL_SPI3_Receive_DMA Error = %d\r\n", Status);

    Status = HAL_SPI_Receive_DMA(&hspi2, (uint8_t *) PDM_Buff2, sizeof(PDM_Buff2) / sizeof(PDM_Buff2[0]));
    if (Status != HAL_OK)
        printf("HAL_SPI2_Receive_DMA Error = %d\r\n", Status);

    Status = HAL_SPI_Receive_DMA(&hspi1, (uint8_t *) PDM_Buff1, sizeof(PDM_Buff1) / sizeof(PDM_Buff1[0]));
    if (Status != HAL_OK)
        printf("HAL_SPI1_Receive_DMA Error = %d\r\n", Status);

}



void enableDMA(void)
{
	HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);//SPI4
	HAL_NVIC_EnableIRQ(SPI4_IRQn);

	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);//SPI3
	HAL_NVIC_EnableIRQ(SPI3_IRQn);

	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);//SPI2
	HAL_NVIC_EnableIRQ(SPI2_IRQn);

	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);//SPI1
	HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

void disableDMA(void)
{
	HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);//SPI1
	HAL_NVIC_DisableIRQ(SPI1_IRQn);

	HAL_NVIC_DisableIRQ(DMA1_Stream3_IRQn);//SPI2
	HAL_NVIC_DisableIRQ(SPI2_IRQn);

	HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);//SPI3
	HAL_NVIC_DisableIRQ(SPI3_IRQn);

	HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn);//SPI4
	HAL_NVIC_DisableIRQ(SPI4_IRQn);
}

void executeCicFilterAll()
{
	// Software CIC Filter
	executeCicFilter((uint8_t*) PDM_RawBits1, PDM_SAMPLE_SIZE * DECIMATION_M,
		 (int32_t*) PDM_Filtered1_int32, &PDM1_st);

	executeCicFilter((uint8_t*) PDM_RawBits2, PDM_SAMPLE_SIZE * DECIMATION_M,
		 (int32_t*) PDM_Filtered2_int32, &PDM2_st);

	executeCicFilter((uint8_t*) PDM_RawBits3, PDM_SAMPLE_SIZE * DECIMATION_M,
		 (int32_t*) PDM_Filtered3_int32, &PDM3_st);

	executeCicFilter((uint8_t*) PDM_RawBits4, PDM_SAMPLE_SIZE * DECIMATION_M,
		 (int32_t*) PDM_Filtered4_int32, &PDM4_st);

}

void executeCicFilter(uint8_t* pInBit, uint32_t pInBit_Num, int32_t* pOut_int32, struct CicFilter_t* st)
{

    int32_t in_bit;
    uint8_t in_Byte;
    uint32_t Bit_i, Decimation_count;
    uint8_t i;

    for(Bit_i = 0, Decimation_count = st->decimation - 1; Bit_i < pInBit_Num; ++Bit_i, --Decimation_count)
    {
        in_Byte = *(pInBit + (Bit_i >> 3) );
        if( (in_Byte >> (Bit_i & 0x07)) & 0x01 )    // First bit is [b0]
            in_bit = 1;
        else
            in_bit = -1;

        *(st->out_i) += in_bit;

        for (i = 1; i < st->order; ++i)
        {
            *(st->out_i + i) += *(st->out_i + i - 1);
        }

        // Decimation
        if(Decimation_count == 0)
        {
            Decimation_count = st->decimation;

            // Comb filter
            *(st->out_c) = *(st->out_i + st->order - 1) - *(st->z1_c);
            *(st->z1_c)  = *(st->out_i + st->order - 1);

            for (i = 1; i < st->order; ++i)
            {
                *(st->out_c + i) = *(st->out_c + i - 1) - *(st->z1_c + i);
                *(st->z1_c  + i) = *(st->out_c + i - 1);
            }

            *(pOut_int32 + Bit_i / st->decimation) = *(st->out_c + st->order - 1);
        }
    }
}

void clearBufferValue()
{
	memset(PDM_Buff1,0,sizeof(PDM_Buff1));
	memset(PDM_Buff2,0,sizeof(PDM_Buff2));
	memset(PDM_Buff3,0,sizeof(PDM_Buff3));
	memset(PDM_Buff4,0,sizeof(PDM_Buff4));
}

void finalizeCicFilterStruct(struct CicFilter_t* st)
{
    if (st == NULL)
        return;
    
    st->order = 0;
    st->decimation = 0;
    free(st->out_i);
    free(st->out_c);
    free(st->z1_c);
}


/* ---------------------------------------------------------------- */
/* -- Interrupt callback functions                               -- */
/* ---------------------------------------------------------------- */
inline void PDM_DMA_Callback(bool isCpltCallback)
{
    if(needs_CopyPDMbits)
    {
        // Copy data to PDM_RawBits
        uint32_t CopyBytes = sizeof(PDM_RawBits1) / sizeof(PDM_RawBits1[0]);
        
        if(isCpltCallback)
        {
            // Full
            for (uint32_t i = 0; i < CopyBytes; i++)
            {
                PDM_RawBits1[i] = PDM_Buff1[i + CopyBytes];
            	PDM_RawBits2[i] = PDM_Buff2[i + CopyBytes];
                PDM_RawBits3[i] = PDM_Buff3[i + CopyBytes];
            	PDM_RawBits4[i] = PDM_Buff4[i + CopyBytes];
            }
        }
        else
        {
            // Half
            for (uint32_t i = 0; i < CopyBytes; i++)
            {
                PDM_RawBits1[i] = PDM_Buff1[i];
                PDM_RawBits2[i] = PDM_Buff2[i];
                PDM_RawBits3[i] = PDM_Buff3[i];
                PDM_RawBits4[i] = PDM_Buff4[i];
            }
        }
        isCalled_PDM_DMA_Callback = true;
        needs_CopyPDMbits = false;
    }
}

/* Private functions ---------------------------------------------------------*/

/***************************************************************END OF FILE****/
