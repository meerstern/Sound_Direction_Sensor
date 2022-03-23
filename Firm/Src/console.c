/*
 *  			consle.c
 *
 *  Copyright (c) 2022
 *  K.Watanabe,Crescent
 *  Released under the MIT license
 *  http://opensource.org/licenses/mit-license.php
 *
 */

/* Include system header files -----------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

/* Include user header files -------------------------------------------------*/
#include "main.h"
#include "console.h"
#include "core_cm4.h"
#include "pdm_cicfilter.h"

/* Imported variables --------------------------------------------------------*/
extern float ActivitySetValue;
/* Private function macro ----------------------------------------------------*/
#define STR_GET_ACT_VAL           "act?"
#define STR_SET_ACT_VAL           "act="
#define STR_SET_RESET	          "rst!"

/* Private enum tag ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t consoleBuffer[100];
uint16_t consoleBuffSize=0;
uint8_t consoleString[100];
/* Private function prototypes -----------------------------------------------*/
static void executeConsoleCommand();

/* Exported functions --------------------------------------------------------*/
void getConsoleString(uint8_t *data)
{
	if( consoleBuffSize < sizeof(consoleBuffer) )
	{
		consoleBuffer[consoleBuffSize] = *data;
		consoleBuffSize++;
	}

	if( *data == '\n' || *data == '\r' )
	{
		printf(">>%s",consoleBuffer);
		strcpy((char*)consoleString, (char*)consoleBuffer);
		executeConsoleCommand();
		memset(consoleBuffer,'\0',sizeof(consoleBuffer));
		consoleBuffSize=0;
	}
}


/* Private functions ---------------------------------------------------------*/
static void executeConsoleCommand()
{
	char *cmd = (char*)consoleString;

	// Get Activity Value
	if( strncmp( cmd, STR_GET_ACT_VAL, 4) == 0 )
	{
		printf("\r\nActivity Threshold:%1.1f\r\n", ActivitySetValue);
	}
	// Set Activity Value
	else if( strncmp( cmd, STR_SET_ACT_VAL, 4) == 0 )
	{
		float val = 0;
		char *p = strpbrk( cmd, "1234567890." );
		if ( p != NULL ) val = atof( p );
		ActivitySetValue = val;
		printf("\r\n Set Activity Threshold:%1.1f\r\n", val);
	}
	// Reset Command
	else if( strncmp( cmd, STR_SET_RESET, 4) == 0 )
	{
		printf("\r\n Soft RESET!\r\n");
		disableDMA();
		NVIC_SystemReset();
	}

	memset(consoleString,'\0',sizeof(consoleString));
}


/***************************************************************END OF FILE****/
