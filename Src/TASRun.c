#include <string.h>
#include "n64.h"
#include "snes.h"
#include "TASRun.h"
#include "stm32f4xx_hal.h"
#include "main.h"

#define MAX_NUM_RUNS 2

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

TASRun tasruns[MAX_NUM_RUNS];

uint16_t TASRunGetSize(uint8_t runNum)
{
	return tasruns[runNum].size;
}

uint8_t TASRunIsInitialized(uint8_t runNum)
{
	return tasruns[runNum].initialized;
}

void TASRunSetInitialized(uint8_t runNum, uint8_t init)
{
	tasruns[runNum].initialized = init;
}

RunData (*GetNextFrame(int runNum))[MAX_CONTROLLERS][MAX_DATA_LANES]
{
	if(tasruns[runNum].size == 0) // in case of buffer underflow
	{
		return NULL; // buffer underflow
	}

	RunData (*retval)[MAX_CONTROLLERS][MAX_DATA_LANES] = tasruns[runNum].current;

	// advance frame
	if(tasruns[runNum].current != tasruns[runNum].end)
	{
		(tasruns[runNum].current)++;
	}
	else
	{
		tasruns[runNum].current = tasruns[runNum].runData;
	}

	tasruns[runNum].size--;

	return retval;
}

uint8_t AddTransition(int numRun, TransitionType type, uint32_t frameNumber)
{
	int x = 0;
	while(x < MAX_TRANSITIONS)
	{
		if(tasruns[numRun].transitions_dpcm[x].frameno == 0) // first blank transition slot found
		{
			tasruns[numRun].transitions_dpcm[x].frameno = frameNumber;
			tasruns[numRun].transitions_dpcm[x].type = type;
			return 1;
		}

		x++;
	}

	return 0; // failure: no room to add transition
}

uint32_t TASRunGetFrameCount(int numRun)
{
	return tasruns[numRun].frameCount;
}

uint8_t TASRunIncrementFrameCount(int numRun)
{
	tasruns[numRun].frameCount++;

	int x = 0;
	while(x < MAX_TRANSITIONS)
	{
		if(tasruns[numRun].transitions_dpcm[x].frameno == 0) // out of transitions to search for
		{
			break;
		}

		if(tasruns[numRun].transitions_dpcm[x].frameno == tasruns[numRun].frameCount)
		{
			switch(tasruns[numRun].transitions_dpcm[x].type)
			{
				case TRANSITION_ACE:
					tasruns[numRun].dpcmFix = 0;
					return 1;
				break;
				case TRANSITION_NORMAL:
					tasruns[numRun].dpcmFix = 1;
					return 1;
				break;
				case TRANSITION_RESET_SOFT:
					return 2;
				break;
				case TRANSITION_RESET_HARD:
					return 3;
				break;
			}
		}

		x++;
	}

	return 0;
}

void TASRunSetOverread(int numRun, uint8_t overread)
{
	tasruns[numRun].overread = overread;
}

uint8_t TASRunGetOverread(int numRun)
{
	return tasruns[numRun].overread;
}

void TASRunSetDPCMFix(int numRun, uint8_t dpcm)
{
	tasruns[numRun].dpcmFix = dpcm;
}

uint8_t TASRunGetDPCMFix(int numRun)
{
	return tasruns[numRun].dpcmFix;
}

void TASRunSetClockFix(int numRun, uint8_t cf)
{
	if(cf > 1)
	{
		tasruns[numRun].clockFix = cf;
		htim6.Init.Period = htim7.Init.Period = cf-1;
	}
	else
	{
		tasruns[numRun].clockFix = 0;
	}
}

uint8_t TASRunGetClockFix(int numRun)
{
	return (tasruns[numRun].clockFix != 0) ? 1 : 0;
}

void ResetTASRuns()
{
	memset(tasruns,0,sizeof(tasruns));
	for(int x = 0;x < MAX_NUM_RUNS;x++)
	{
		tasruns[x].buf = tasruns[x].runData;
		tasruns[x].current = tasruns[x].runData;
		tasruns[x].end = &(tasruns[x].runData[MAX_SIZE-1]);
	}
}

void TASRunSetNumControllers(int numRun, uint8_t numControllers)
{
	tasruns[numRun].numControllers = numControllers;
}

uint8_t TASRunGetNumControllers(int numRun)
{
	return tasruns[numRun].numControllers;
}

void TASRunSetNumDataLanes(int numRun, uint8_t numDataLanes)
{
	tasruns[numRun].numDataLanes = numDataLanes;
}

uint8_t TASRunGetNumDataLanes(int numRun)
{
	return tasruns[numRun].numDataLanes;
}

Console TASRunGetConsole(int numRun)
{
	return tasruns[numRun].console;
}

void TASRunSetConsole(int numRun, Console console)
{
	tasruns[numRun].console = console;
}





uint32_t GetSizeOfControllerDataForConsole(int console_type)
{
	switch (console_type) {
	case CONSOLE_N64:
		return sizeof(N64ControllerData);
		break;
	case CONSOLE_SNES:
		return sizeof(SNESControllerData);
		break;
	case CONSOLE_NES:
		return sizeof(NESControllerData);
		break;
	case CONSOLE_GC:
		return sizeof(GCControllerData) ;
		break;
	}
	return 0; // should never reach this
}

uint32_t GetSizeOfInputForRun(int run_index)
{
	return tasruns[run_index].numControllers * tasruns[run_index].numDataLanes * GetSizeOfControllerDataForConsole(tasruns[run_index].console);
}

int ExtractDataAndAddFrame(int run_index, uint8_t *buffer, uint32_t n)
{
	size_t bytesPerInput = GetSizeOfControllerDataForConsole(tasruns[run_index].console);
	uint8_t numControllers = tasruns[run_index].numControllers;
	uint8_t numDataLanes = tasruns[run_index].numDataLanes;

	RunData frame[MAX_CONTROLLERS][MAX_DATA_LANES];

	if(tasruns[run_index].size == MAX_SIZE)
	{
		return 0;
	}

	memset(frame, 0, sizeof(frame)); // prepare the data container

	uint8_t *buffer_position = buffer;
	for(int x = 0;x < numControllers;x++)
	{
		for(int y = 0;y < numDataLanes;y++)
		{
			memcpy(&frame[x][y], buffer_position, bytesPerInput); // copy only what is necessary
			buffer_position += bytesPerInput; // advance the index only what is necessary
		}
	}

	memcpy((RunData*)tasruns[run_index].buf,frame,sizeof(frame));

	// NOTE: These two pointer modifications must occur in an atomic fashion
	//       A poorly-timed interrupt could cause bad things.
	__disable_irq();
	// loop around if necessary
	if(tasruns[run_index].buf != tasruns[run_index].end)
	{
		(tasruns[run_index].buf)++;
	}
	else // buf is at end, so wrap around to beginning
	{
		tasruns[run_index].buf = tasruns[run_index].runData;
	}

	tasruns[run_index].size++;
	__enable_irq();

	return 1;
}

void SetN64InputMode()
{
	// port C4 to input mode
	GPIOC->MODER &= ~(1 << 9);
	GPIOC->MODER &= ~(1 << 8);
}

void SetN64OutputMode()
{
	// port C4 to output mode
	GPIOC->MODER &= ~(1 << 9);
	GPIOC->MODER |= (1 << 8);
}


void SetN64Mode()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = P1_DATA_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void SetSNESMode()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = P1_DATA_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
