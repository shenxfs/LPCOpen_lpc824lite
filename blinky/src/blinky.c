/*
===============================================================================
 Name        : blinky.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

//#include <cr_section_macros.h>
#define TICKRATE_HZ (10)	/* 10 ticks per second */

// TODO: insert other include files here
void SysTick_Handler(void)
{
	Board_LED_Toggle(0);
	Board_LED_Toggle(1);
	Board_LED_Toggle(2);
	Board_LED_Toggle(3);
	Board_LED_Toggle(4);
	Board_LED_Toggle(5);
	Board_LED_Toggle(6);
	Board_LED_Toggle(7);
}

// TODO: insert other definitions and declarations here

int main(void) {

#if defined (__USE_LPCOPEN)
    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
#if !defined(NO_BOARD_LIB)
    // Set up and initialize all required blocks and
    // functions related to the board hardware
    Board_Init();
    // Set the LED to the state of "Off"
    Board_LED_Set(0, false);
#endif
#endif

    // TODO: insert code here
	DEBUGSTR("Blinky sequencer demo\r\n");
	DEBUGOUT("System Clock: %uMHz\r\n", SystemCoreClock / 1000000);
	DEBUGOUT("Device ID: 0x%x\r\n", Chip_SYSCTL_GetDeviceID());
	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);

    // Force the counter to be placed into memory
//    volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1) {
    	__WFI();
    }
    return 0 ;
}
