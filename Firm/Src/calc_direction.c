/*
 *  			calc_direction.c
 *
 *  Copyright (c) 2022
 *  K.Watanabe,Crescent
 *  Released under the MIT license
 *  http://opensource.org/licenses/mit-license.php
 *
 *  Refer to the algorithm:
 *  https://github.com/wooters/miniDSP
 */

/* Include system header files -----------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <float.h>

/* Include user header files -------------------------------------------------*/
#include "main.h"
#include "arm_const_structs.h"
#include "pdm_cicfilter.h"
#include "calc_direction.h"

/* Imported variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern uint8_t I2CRegData[8];
/* Private function macro ----------------------------------------------------*/
// Select Algorithm of Sound Source Direction
#define ALGORITHM	USE_GCC_PHAT

//Supported Algorithms
#define USE_GCC_PHAT	0
#define USE_SIMP		1

#define MAXOFFSET		5	//Determined from Mic Distance and Target Frequency, Sampling Frequency

// FFT Settings
#define FFTFLAG        	0
#define IFFTFLAG        1
#define DOBITREVERSE    1
#define fftSize         PDM_SAMPLE_SIZE
#define FFT_HANDLE 		arm_cfft_sR_f32_len256//arm_cfft_sR_f32_len512

/* Private enum tag ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float FFT_frq[PDM_SAMPLE_SIZE/2] = {0.0f};
float FFT_window[PDM_SAMPLE_SIZE] = {0.0f};
float p_inputA[PDM_SAMPLE_SIZE*2];
float p_inputB[PDM_SAMPLE_SIZE*2];
float p_output[PDM_SAMPLE_SIZE*2];
float xcorr[PDM_SAMPLE_SIZE+1];
float FFT_inp1[PDM_SAMPLE_SIZE] = {0.0f};
float FFT_inp2[PDM_SAMPLE_SIZE] = {0.0f};
float FFT_inp3[PDM_SAMPLE_SIZE] = {0.0f};
float FFT_inp4[PDM_SAMPLE_SIZE] = {0.0f};
float actBuffer[256]={0.0f};
uint16_t actBufferCount;
float ActivitySetValue=0.0f;


/* Private function prototypes -----------------------------------------------*/
static void findMaxIndex(float *data, uint16_t size, float *maxVal, uint16_t *maxIndex);
static void shiftArray(float *input, float *output, uint16_t size);
static int calcDelay(float *inputA, float *inputB, uint16_t length, uint16_t mode, uint16_t offset);
static void calcQuadDelay(float *inputA, float *inputB, float *inputC, float *inputD, int length, int mode, int offset, int *delay);
static float calcAverage(float *p_input_data, size_t input_data_num);
static float calcStandardDeviation(float *p_input_data, size_t input_data_num);
//static float calcAbs(float val);
static float calcActivity();
static void updateReg(float deg, float degOrg, float actVal);
/* Exported functions --------------------------------------------------------*/
float calcAngle()
{
	float CalcDeg = -1.0f;
	float CalcDegOrg;
	float FFT_OutArg1;
	float FFT_OutArg2;
	float FFT_OutArg3;
	float FFT_OutArg4;

	float SumArgX,SumArgY;
	float ArgVec[4][2];
	int intDelay[5];

	static float ActivitySetValueOld=0.0f;

	// Data copy
	for (uint32_t i = 0; i < PDM_SAMPLE_SIZE; i++)
	{
		FFT_inp1[i] = PDM_Filtered1_int32[i];
		FFT_inp2[i] = PDM_Filtered2_int32[i];
		FFT_inp3[i] = PDM_Filtered3_int32[i];
		FFT_inp4[i] = PDM_Filtered4_int32[i];
	}


	// Windowing
	arm_mult_f32(FFT_inp1, FFT_window, FFT_inp1, PDM_SAMPLE_SIZE);
	arm_mult_f32(FFT_inp2, FFT_window, FFT_inp2, PDM_SAMPLE_SIZE);
	arm_mult_f32(FFT_inp3, FFT_window, FFT_inp3, PDM_SAMPLE_SIZE);
	arm_mult_f32(FFT_inp4, FFT_window, FFT_inp4, PDM_SAMPLE_SIZE);



	// Calc Standard Deviation for Sound Activity
	float activityValue = calcActivity();


	// Calc Delay
	int maxOffset = MAXOFFSET;
	calcQuadDelay(FFT_inp1, FFT_inp2, FFT_inp3, FFT_inp4, PDM_SAMPLE_SIZE, ALGORITHM, maxOffset, intDelay);

	// Align the center
	float avedeg = ( 0.0f + intDelay[0] + intDelay[1] + intDelay[2] )/4;
	FFT_OutArg1 = 0.0f -avedeg;
	FFT_OutArg2 = intDelay[0]-avedeg;
	FFT_OutArg3 = intDelay[1]-avedeg;
	FFT_OutArg4 = intDelay[2]-avedeg;

	/* * * * * * * *
	 *   MIC Pos   *
	 *  M1:(-1,0)  *
	 *  M2:(0,-1)  *
	 *  M3:(1,0)   *
	 *  M4:(0,1)   *
	 * * * * * * * */

	// Calc Sound Vector from delay
	ArgVec[0][0] = -1.0f*FFT_OutArg1;
	ArgVec[0][1] = 0.0f*FFT_OutArg1;

	ArgVec[1][0] = 0.0f*FFT_OutArg2;
	ArgVec[1][1] = -1.0f*FFT_OutArg2;

	ArgVec[2][0] = 1.0f*FFT_OutArg3;
	ArgVec[2][1] = 0.0f*FFT_OutArg3;

	ArgVec[3][0] = 0.0f*FFT_OutArg4;
	ArgVec[3][1] = 1.0f*FFT_OutArg4;

	SumArgX=ArgVec[0][0]+ArgVec[1][0]+ArgVec[2][0]+ArgVec[3][0];
	SumArgY=ArgVec[0][1]+ArgVec[1][1]+ArgVec[2][1]+ArgVec[3][1];

	if( SumArgX == 0.0f )
		SumArgX = 0.00000001f;

	// Calc Degree of Sound Source from Sound Vector
	float r = sqrtf(SumArgY*SumArgY+SumArgX*SumArgX);
	if(SumArgY<0)	CalcDeg =-acosf(SumArgX/r)*180.0f/PI;
	else 			CalcDeg =+acosf(SumArgX/r)*180.0f/PI;

	if( CalcDeg < 0.0f)
		CalcDeg = CalcDeg + 360.0f;

	// Keep original angle value
	CalcDegOrg = CalcDeg;

	// Check ActivitySetValue
	if( ActivitySetValue != ActivitySetValueOld )
	{
		printf("Activity Threshold: %1.1f \r\n", ActivitySetValue);
		ActivitySetValueOld=ActivitySetValue;
	}

	// Eliminate noise in case of small sound
	if( activityValue < ActivitySetValue )
		CalcDeg = -1.0f;


	updateReg(CalcDeg, CalcDegOrg, activityValue);

	printf("Deg:%1.1f,\t DegOrg:%1.1f,\t Act:%1.2f,\t",CalcDeg, CalcDegOrg, activityValue);

	printf("\r\n");

	return CalcDeg;
}


void prepareWindow(float samplingRate)
{
    // Prepare Hanning window
    const float tmp = 2.0f * M_PI / (float) PDM_SAMPLE_SIZE;
    for (uint32_t i = 0; i < PDM_SAMPLE_SIZE; i++)
        *(FFT_window + i) = 0.5f - 0.5f * arm_cos_f32((float) i * tmp);
    // Prepare Frq Array
    for (uint32_t i = 0; i < PDM_SAMPLE_SIZE / 2; i++)
        *(FFT_frq + i) = (float) i * (float) samplingRate / (float) PDM_SAMPLE_SIZE;

}

void getAdcValue()
{
	uint16_t ADC_Value1=0;
	uint16_t ADC_Value2=0;
	uint16_t i=0;

	HAL_ADC_Stop(&hadc1);

	ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = ADC_CHANNEL_0;//0or4
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

	for(i=0;i<10;i++)
	{
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)  							printf("HAL_ADC_ConfigChannel error\r\n");
		if (HAL_ADC_Start(&hadc1) != HAL_OK)                    							printf("HAL_ADC_Start error\r\n");
		if (HAL_ADC_PollForConversion(&hadc1, 5000) != HAL_OK)  							printf("HAL_ADC_PollForConversion error\r\n");
		if ( (HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_EOC_REG) != HAL_ADC_STATE_EOC_REG)	printf("HAL_ADC_GetState error\r\n");
		ADC_Value1 +=  4096- (HAL_ADC_GetValue(&hadc1) & 0x0000FFFF);
	}
	ADC_Value1 = ADC_Value1/10;


	for(i=0;i<10;i++)
	{
		sConfig.Channel = ADC_CHANNEL_4;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)  							printf("HAL_ADC_ConfigChannel error\r\n");
		if (HAL_ADC_Start(&hadc1) != HAL_OK)                    							printf("HAL_ADC_Start error\r\n");
		if (HAL_ADC_PollForConversion(&hadc1, 5000) != HAL_OK)  							printf("HAL_ADC_PollForConversion error\r\n");
		if ( (HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_EOC_REG) != HAL_ADC_STATE_EOC_REG)	printf("HAL_ADC_GetState error\r\n");
		ADC_Value2 += 4096- (HAL_ADC_GetValue(&hadc1) & 0x0000FFFF);
	}
	ADC_Value2 = ADC_Value2/10;


	ActivitySetValue = 0.3f*log10f(ADC_Value1+0.1f) + 0.6*log10f(ADC_Value2+0.1f);
	if(ActivitySetValue<0)ActivitySetValue=0.0f;
	//printf("Activity Threshold: %1.1f \r\n", ActivitySetValue);

}

void setActivityValue(float fvalue)
{
	ActivitySetValue = fvalue;
	//printf("Activity Threshold: %1.1f \r\n", ActivitySetValue);
}



/* Private functions ---------------------------------------------------------*/

static int calcDelay(float *inputA, float *inputB, uint16_t length, uint16_t mode, uint16_t offset)
{

	float tmp_a;
	float tmp_b;
	float tmp_c;
	int16_t delay=0.0;
	float maxval;
	uint16_t max_i;

	uint16_t center_i = ceil(length/2.0);
	uint16_t start_i = center_i-offset;
	uint16_t len = 2*offset+1;

	for (size_t i=0;i<length;i++)
	{
		//real part
		p_inputA[2*i]=inputA[i];
		p_inputB[2*i]=inputB[i];
		//imagenary part
		p_inputA[2*i+1]=0.0;
		p_inputB[2*i+1]=0.0;
	}


	// FFT
	arm_cfft_f32(&FFT_HANDLE, p_inputA, FFTFLAG, DOBITREVERSE);
	arm_cfft_f32(&FFT_HANDLE, p_inputB, FFTFLAG, DOBITREVERSE);


	for (size_t i=length;i<2*length;i++)//Delete
	{
		p_inputA[i]=0.0;
		p_inputB[i]=0.0;
	}


	for (size_t i=0;i<length;i++)
	{
	  if(20<i && i<40)//Limit Frq Range around 1kHz
	  {
		  p_inputA[i]=1.0*p_inputA[i];
		  p_inputB[i]=1.0*p_inputB[i];
	  }
	  else
	  {
		  p_inputA[i]=0.1*p_inputA[i];
		  p_inputB[i]=0.1*p_inputB[i];
	  }

	}


	for (size_t i = 0; i < length; i++)
	{

		switch (mode)
		{
			case 0://PHAT
				tmp_a = p_inputB[2*i]*p_inputA[2*i]+p_inputB[2*i+1]*p_inputA[2*i+1];
				tmp_b = -p_inputB[2*i]*p_inputA[2*i+1]+p_inputA[2*i]*p_inputB[2*i+1];
				tmp_c = sqrt(tmp_a*tmp_a+tmp_b*tmp_b);
				if(tmp_c==0.0f)tmp_c=FLT_MIN;
				p_output[2*i]=tmp_a/tmp_c;
				p_output[2*i+1]=tmp_b/tmp_c;
				break;
			case 1://SIMP
				tmp_a = p_inputB[2*i]*p_inputA[2*i]+p_inputB[2*i+1]*p_inputA[2*i+1];
				tmp_b =-p_inputB[2*i]*p_inputA[2*i+1]+p_inputA[2*i]*p_inputB[2*i+1];
				p_output[2*i]=tmp_a/((float) length);
				p_output[2*i+1]=tmp_b/((float) length);
				break;
			default:
				break;
		}
	}

	// IFFT
	arm_cfft_f32(&FFT_HANDLE, p_output, IFFTFLAG, DOBITREVERSE);

	for (size_t j = 0; j < length; j++)
	{
		xcorr[j] = p_output[2*j];
	}

	shiftArray(xcorr, p_output, length);
	findMaxIndex(p_output+start_i,len,&maxval,&max_i);
	delay = max_i - offset;
	return delay;
}


static void shiftArray(float *input, float *output, uint16_t size)
{
	  uint16_t half = size/2.0;
	  memcpy(output,&input[half],sizeof(float)*(size-half));
	  memcpy(&output[size-half],input,sizeof(float)*half);
}

static void findMaxIndex(float *data, uint16_t size, float *maxVal, uint16_t *maxIndex)
{
	float val=data[0];
	uint16_t index=0;

	for(size_t i = 0; i < size; i++)
	{
		if(data[i]>val)
		{
			val=data[i];
			index=i;
		}
	}

	*maxVal=val;
	*maxIndex=index;
}

static void calcQuadDelay(float *inputA, float *inputB, float *inputC, float *inputD, int length, int mode, int offset, int *delay)
{
	delay[0] = calcDelay(inputA, inputB, length, mode, offset);//A,C
	delay[1] = calcDelay(inputA, inputC, length, mode, offset);//B,D
	delay[2] = calcDelay(inputA, inputD, length, mode, offset);//A,D
}

//static float calcAbs(float val)
//{
//	if(val<0)
//		return -val;
//	else
//		return val;
//}

static float calcAverage(float *p_input_data, size_t input_data_num)
{
    // Average
    float average = 0.0f;
    for(size_t i = 0; i < input_data_num; i++)
        average += *(p_input_data + i);
    average /= (float)input_data_num;
    return average;
}

static float calcStandardDeviation(float *p_input_data, size_t input_data_num)
{
    float average = calcAverage(p_input_data, input_data_num);
    float std = 0.0f;
    for(size_t i = 0; i < input_data_num; i++)
        std += (*(p_input_data + i) - average) * (*(p_input_data + i) - average);
    std /= (float)input_data_num;
    std = sqrtf(std);
    return std;
}


static float calcActivity()
{
	float stdValue[4];

	stdValue[0]=calcStandardDeviation((float*)FFT_inp1,PDM_SAMPLE_SIZE);
	stdValue[1]=calcStandardDeviation((float*)FFT_inp2,PDM_SAMPLE_SIZE);
	stdValue[2]=calcStandardDeviation((float*)FFT_inp3,PDM_SAMPLE_SIZE);
	stdValue[3]=calcStandardDeviation((float*)FFT_inp4,PDM_SAMPLE_SIZE);

#ifdef FACTORY_CHECK_MODE
	printf("[ %1.1f,\t %1.1f,\t %1.1f,\t %1.1f ]\t",
			stdValue[0],stdValue[1],stdValue[2],stdValue[3]);
#endif

	float activityValue = log10f((stdValue[0]+stdValue[1]+stdValue[2]+stdValue[3])/4.0f);
	return activityValue;
}

static void updateReg(float deg, float degOrg, float actVal)
{

	int16_t intDeg = (int16_t)(deg*10.0);
	int16_t intDegOrg = (int16_t)(degOrg*10.0);
	int16_t intAct = (int16_t)(actVal*10.0);

	I2CRegData[ENUM_REG_DEG_H] = (uint8_t)((intDeg&0xFF00)>>8);
	I2CRegData[ENUM_REG_DEG_L] = (uint8_t)((intDeg&0x00FF)>>0);
	I2CRegData[ENUM_REG_DEG_ORG_H] = (uint8_t)((intDegOrg&0xFF00)>>8);
	I2CRegData[ENUM_REG_DEG_ORG_L] = (uint8_t)((intDegOrg&0x00FF)>>0);

	I2CRegData[ENUM_REG_ACT_H] = (uint8_t)((intAct&0xFF00)>>8);
	I2CRegData[ENUM_REG_ACT_L] = (uint8_t)((intAct&0x00FF)>>0);
	I2CRegData[ENUM_REG_REFRESH] = 1;
}
/***************************************************************END OF FILE****/
