/******************************************Code documentation**************************************
    FreeRTOS V8.0.1 - Copyright (C) 2014 Real Time Engineers Ltd.
     This file is part of the FreeRTOS distribution.
    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
 ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.
    1 tab == 4 spaces!
    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.
    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.
    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
 ********************************************************************************************************/

/* FreeRTOS.org includes. */
/* Kernel includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

//
// Include FreeRTOS timer functions
//
#include "timers.h"

/* Demo includes. */
#include "basic_io.h"

/* Library includes. */
#include "LPC17xx.h"

/* The interrupt number to use for the software interrupt generation.  This
could be any unused number.  In this case the first chip level (non system)
interrupt is used, which happens to be TIMER0 on the LPC1768. */
#define mainSW_INTERRUPT_ID        ( 1 )

/* Macro to force an interrupt. */
#define mainTRIGGER_INTERRUPT()    NVIC_SetPendingIRQ( mainSW_INTERRUPT_ID )

/* Macro to clear the same interrupt. */
#define mainCLEAR_INTERRUPT()    NVIC_ClearPendingIRQ( mainSW_INTERRUPT_ID )

/* The priority of the software interrupt.  The interrupt service routine uses
an (interrupt safe) FreeRTOS API function, so the priority of the interrupt must
be equal to or lower than the priority set by
configMAX_SYSCALL_INTERRUPT_PRIORITY - remembering that on the Cortex-M3 high
numeric values represent low priority values, which can be confusing as it is
counter intuitive. */
#define mainSOFTWARE_INTERRUPT_PRIORITY         ( 5 )


/* The tasks to be created. */
static void vFreqHandlerTask( void *pvParameters );
static void vPeriodicTask( void *pvParameters );
static void vPrintFreqCurrentTask( void *pvParameters );
static void vPrintFreqChangeTask( void *pvParameters );
static void vChangeLEDTask( void *pvParameters );

/* Enable the software interrupt and set its priority. */
static void prvSetupSoftwareInterrupt();

/* The service routine for the interrupt.  This is the interrupt that the
task will be synchronized with. */
void vSoftwareInterruptHandler( void );

/* Timer callback function for real time clock */
void vRTCTimerCallback( TimerHandle_t xTimer );

/* Timer callback function for switch debouncing */
void vDebounceTimerCallback( TimerHandle_t xTimer );


/*------------------------------------------------------------------------------------------------------------------------*/

/* Declare a variable of type xSemaphoreHandle.  This is used to reference the
semaphore that is used to synchronize a task with an interrupt. */
xSemaphoreHandle xBinarySemaphore;
xSemaphoreHandle xFreqCalcBinarySemaphore;
xSemaphoreHandle xPrintBinarySemaphore;
xSemaphoreHandle xPrintFreqChangeBinarySemaphore;

/* Declare mutex's. Variables are type xSemaphoreHandle */
xSemaphoreHandle xADCValueMutex;
xSemaphoreHandle xFrequencyMutex;
xSemaphoreHandle xRTCMutex;

/*-------------------------------------------------------------------------------------------------------------------------*/

volatile int last;     // value of last reading, for peak detection
volatile int count;     // how many readings were taken
volatile int up;     // direction - up = 1 down = 0
volatile int ticks;     // number of peaks observed in 100 cycles
volatile int g_Freq;     // the last frequency measured

// Number of samples that have a stable value for debouncing the switch.
uint16_t g_ui16StableCount = 0;
TimerHandle_t g_xDebounceTimer;

// Store RTC value in seconds
long lSeconds = 0;
// using last vs current comparison; current is adc_val
int adc_val; // current reading


/*Main function which include button interrupt and other function calls */
int main( void ) {

	// this is Don's code, to set up the ADC
	LPC_SC->PCONP |= 1 << 12; // set PCADC bit
	LPC_PINCON->PINSEL1 = (LPC_PINCON->PINSEL1 & ~(0X3 << 14)) | (0X1 << 14); // set 15:14 to 01
	LPC_PINCON->PINMODE1 = (LPC_PINCON->PINMODE1 & ~(0X3 << 14)) | (0X1 << 15); // set 15:14 to 10
	LPC_ADC->ADCR = 1 | (2 << 8) | (1 << 14) | (1 << 21);

	// set up button interrupt
	LPC_PINCON->PINSEL0  &= ~(0x03 << 18);     // set 19:18 00
	LPC_PINCON->PINMODE0 &= ~(0x03 << 18);     // set 19:18 00
	LPC_GPIOINT->IO0IntEnR |= (1 << 9);      // enable rising edge interrupt on pin 0.9
	NVIC_SetPriority(EINT3_IRQn, 0x08);
	NVIC_EnableIRQ(EINT3_IRQn);

	// Set GPIO0 pin 22 direction to output
	// This pin is connected to the development board LED2
	LPC_GPIO0->FIODIR |= (1 << 22);

	//
	// Create a timer that will interrupt every 1000 ms for the real time clock
	//
	TimerHandle_t xRTCTimer;
	long x = 1;
	xRTCTimer = xTimerCreate("Print Timer", (1000 / portTICK_PERIOD_MS), pdTRUE, (void *) x,	vRTCTimerCallback);

	//
	// Create a 1 ms timer that will interrupt for switch debouncing
	//
	long z = 3;
	g_xDebounceTimer = xTimerCreate("Debounce Timer", (1 / portTICK_PERIOD_MS), pdFALSE, (void *) z, vDebounceTimerCallback);



	/* Before a semaphore is used it must be explicitly created.  In this example
    a binary semaphore is created. */
	vSemaphoreCreateBinary( xBinarySemaphore );
	vSemaphoreCreateBinary( xFreqCalcBinarySemaphore );
	vSemaphoreCreateBinary( xPrintBinarySemaphore );
	vSemaphoreCreateBinary( xPrintFreqChangeBinarySemaphore );

	/* Create mutexes */
	xADCValueMutex = xSemaphoreCreateMutex();
	xFrequencyMutex = xSemaphoreCreateMutex();
	xRTCMutex = xSemaphoreCreateMutex();

	/* Check the semaphores and mutexes were created successfully. */
	if( xBinarySemaphore != NULL || xFreqCalcBinarySemaphore != NULL || xPrintBinarySemaphore ||
			xPrintFreqChangeBinarySemaphore || xADCValueMutex != NULL || xFrequencyMutex != NULL ||
			xRTCMutex != NULL )
	{
		/* Enable the software interrupt and set its priority. */
		prvSetupSoftwareInterrupt();

		/* Create the 'handler' task.  This is the task that will be synchronized
        with the interrupt.  The handler task is created with a high priority to
        ensure it runs immediately after the interrupt exits.  In this case a
        priority of 3 is chosen. */
		xTaskCreate( vFreqHandlerTask, "Handler", 240, NULL, 3, NULL );

		/* Create the task that will periodically generate a software interrupt.
        This is created with a priority below the handler task to ensure it will
        get preempted each time the handler task exits the Blocked state. */
		xTaskCreate( vPeriodicTask, "Periodic", 240, NULL, 4, NULL );

		/* Create the task that will periodically generate a software interrupt.
        This is created with a priority below the handler task to ensure it will
        get preempted each time the handler task exits the Blocked state. */
		xTaskCreate( vPrintFreqCurrentTask, "Print Current Frequency", 240, NULL, 1, NULL );

		/* Create the task that will periodically generate a software interrupt.
        This is created with a priority below the handler task to ensure it will
        get preempted each time the handler task exits the Blocked state. */
		xTaskCreate( vPrintFreqChangeTask, "Print Frequency Change", 240, NULL, 1, NULL );

		/* Create the task that will flicker the LED on and off.
		 */
		xTaskCreate( vChangeLEDTask,    "Change LED State", 240, NULL, 4, NULL );

		/*
		 * Start the RTC timer
		 */
		xTimerStart(xRTCTimer, portMAX_DELAY);

		/* Start the scheduler so the created tasks start executing. */
		vTaskStartScheduler();
	}

	/* If all is well we will never reach here as the scheduler will now be
    running the tasks.  If we do reach here then it is likely that there was
    insufficient heap memory available for a resource to be created. */
	for( ;; );
	return 0;
}
/*------------------------------------------------------------------------------------------------------------------------*/


static void vChangeLEDTask( void *pvParameters )
{
	int freq = 1;

	for( ; ;) {
		// Toggle GPIO0 pin 22 status
		if (LPC_GPIO0->FIOPIN & (1 << 22))
		{
			LPC_GPIO0->FIOCLR = (1 << 22);
		}
		else
		{
			LPC_GPIO0->FIOSET = (1 << 22);
		}
		xSemaphoreTake( xFrequencyMutex, portMAX_DELAY );

		freq = g_Freq; // send this up so button can get it

		// Give back mutex for g_Freq global variable
		xSemaphoreGive( xFrequencyMutex );

		if (freq > 200) {
			freq = 200;
		}
		vTaskDelay(((((200 - freq) * 480) / 150) + 20) / portTICK_PERIOD_MS);
	}


}
/*------------------------------------------------------------------------------------------------------------------------*/

static void vFreqHandlerTask( void *pvParameters )
{
	int current_base_freq = 0;
	int filtered_adc = 2048; // filtered value of ADC reading
	//int tock;    // will compare this with the volatile tick to see how much time passed // NOT USED???

	/* As per most tasks, this task is implemented within an infinite loop.
    Take the semaphore once to start with so the semaphore is empty before the
    infinite loop is entered.  The semaphore was created before the scheduler
    was started so before this task ran for the first time.*/
	xSemaphoreTake( xFreqCalcBinarySemaphore, 0 );

	for( ;; )
	{
		/* Use the semaphore to wait for the event.  The task blocks
        indefinitely meaning this function call will only return once the
        semaphore has been successfully obtained - so there is no need to check
        the returned value. */
		xSemaphoreTake( xFreqCalcBinarySemaphore, portMAX_DELAY );

		// Take mutex for adc_val global variable
		xSemaphoreTake( xADCValueMutex, portMAX_DELAY );

		// Filter using exponential moving average
		filtered_adc = (adc_val >> 2) + ( filtered_adc - (filtered_adc >> 2) );

		// Give back mutex for adc_val global variable
		xSemaphoreGive( xADCValueMutex );

		// The algorithm used to detect peaks of the sine wave
		if(filtered_adc > last + 75) { // make sure we have a noticeable difference, not just noise
			up = 1; // which direction is the wave going

		} else { // if we aren't going up, we are going down
			if((up == 1) && (filtered_adc < last - 75)) { // if we *were* going up, and now are going down, that is a peak
				up = 0; // note we are going down now, no more peaks until we go up again
				if(count >= 1000) { // if we counted for four seconds
					count = 0; // reset count
					//printf("%d %s\n", ticks/4, "Hz"); // output to console, disabled after button setup

					// Take mutex for g_Freq global variable
					xSemaphoreTake( xFrequencyMutex, portMAX_DELAY );

					g_Freq = ticks; // send this up so button can get it
					//printf("%s %d %s\n", "The frequency is ", freq, "Hz");

					// Give back mutex for g_Freq global variable
					xSemaphoreGive( xFrequencyMutex );

					if (ticks > current_base_freq + 10 || ticks < current_base_freq - 10 )
					{
						// Update the base frequency
						current_base_freq = ticks;
						// Allow the print task to print
						xSemaphoreGive( xPrintFreqChangeBinarySemaphore );
					}



					ticks = 0; // reset tick counter
				}
				ticks++; // increment ticks
			}
		}
		count++; // increment count every cycle, zero every peak
		last = filtered_adc; // set last value, to compare to next round
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/

static void vPeriodicTask( void *pvParameters )
{
	/* As per most tasks, this task is implemented within an infinite loop. */
	for( ;; )
	{		/* This task is just used to 'simulate' an interrupt.  This is done by
        periodically generating a software interrupt. */
		vTaskDelay( 1 / portTICK_RATE_MS ); // 1ms delay, helps clean up the noise

		/* Generate the interrupt, printing a message both before hand and
        afterwards so the sequence of execution is evident from the output. */
		//vPrintString( "Periodic task - About to generate an interrupt.\n" );
		mainTRIGGER_INTERRUPT();

	}
}
/*-------------------------------------------------------------------------------------------------------------------------*/


static void vPrintFreqChangeTask( void *pvParameters )
{
	/* Take the semaphore once to start with so the semaphore is empty before the
	infinite loop is entered.  The semaphore was created before the scheduler
	was started so before this task ran for the first time. */
	xSemaphoreTake( xPrintFreqChangeBinarySemaphore, 0 );

	for ( ;; )
	{
		// Wait until there is new frequency data to display.
		xSemaphoreTake( xPrintFreqChangeBinarySemaphore, portMAX_DELAY );

		// Take mutex for g_Freq global variable
		xSemaphoreTake( xFrequencyMutex, portMAX_DELAY );
		//taskENTER_CRITICAL();
		//xTimerChangePeriod(g_xLEDTimer, 500 / portTICK_PERIOD_MS, portMAX_DELAY);
		//taskEXIT_CRITICAL();

		printf("%s %d %s\n", "Frequency change: ", g_Freq, "Hz");

		// Give back mutex for g_Freq global variable
		xSemaphoreGive( xFrequencyMutex );
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/

static void vPrintFreqCurrentTask( void *pvParameters )
{
	/* Take the semaphore once to start with so the semaphore is empty before the
	infinite loop is entered.  The semaphore was created before the scheduler
	was started so before this task ran for the first time. */
	xSemaphoreTake( xPrintBinarySemaphore, 0 );

	for ( ;; )
	{
		// Wait until there is new frequency data to display.
		xSemaphoreTake( xPrintBinarySemaphore, portMAX_DELAY );

		// Take mutex for g_Freq global variable
		xSemaphoreTake( xFrequencyMutex, portMAX_DELAY );

		printf("%s %d %s\n", "Current frequency: ", g_Freq, "Hz");

		// Give back mutex for g_Freq global variable
		xSemaphoreGive( xFrequencyMutex );
	}
}

/*-------------------------------------------------------------------------------------------------------------------------*/

static void prvSetupSoftwareInterrupt()
{
	/* The interrupt service routine uses an (interrupt safe) FreeRTOS API
    function so the interrupt priority must be at or below the priority defined
    by configSYSCALL_INTERRUPT_PRIORITY. */
	NVIC_SetPriority( mainSW_INTERRUPT_ID, mainSOFTWARE_INTERRUPT_PRIORITY );

	/* Enable the interrupt. */
	NVIC_EnableIRQ( mainSW_INTERRUPT_ID );
}
/*-------------------------------------------------------------------------------------------------------------------------*/

// Interrupt handler for our button interrupt
void EINT3_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	// tell print task to print frequency
	xSemaphoreGiveFromISR( xPrintBinarySemaphore, &xHigherPriorityTaskWoken );

	// This eats processor cycles for ~8ms (with i=1,000,000)
	// which would interfere with reading the ADC every 1 ms.
	// i > 1000000 works
	// i <= 500000 doesn't work reliably
	//int i;
	//for(i = 0; i < 1000000; i++){}

	// Disable the button interrupt at the NVIC
	// The interrupt will not be accepted until the interrupt
	// is re-enabled
	NVIC_DisableIRQ(EINT3_IRQn);

	// Start the debounce timer
	xTimerStartFromISR(g_xDebounceTimer, NULL);

	/* Giving the semaphore may have unblocked a task - if it did and the
    unblocked task has a priority equal to or above the currently executing
    task then xHigherPriorityTaskWoken will have been set to pdTRUE and
    portEND_SWITCHING_ISR() will force a context switch to the newly unblocked
    higher priority task.
    NOTE: The syntax for forcing a context switch within an ISR varies between
    FreeRTOS ports.  The portEND_SWITCHING_ISR() macro is provided as part of
    the Cortex-M3 port layer for this purpose.  taskYIELD() must never be called
    from an ISR! */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

void vSoftwareInterruptHandler( void )
{
	portBASE_TYPE xHigherPriorityTaskWokenA = pdFALSE;
	portBASE_TYPE xHigherPriorityTaskWokenB = pdFALSE;
	portBASE_TYPE xHigherPriorityTaskWokenC = pdFALSE;

	//int count = 0; // made this a volatile so it actually counts

	LPC_ADC->ADCR |= 1 << 24; // start conversion
	while((LPC_ADC->ADDR0 & (1 << 31)) == 0); // wait for conversion to finish

	// Take mutex for adc_val global variable
	xSemaphoreTakeFromISR( xADCValueMutex, &xHigherPriorityTaskWokenA );

	adc_val = (LPC_ADC->ADDR0 >> 4) & 0xfff; // read value

	// Give back mutex for adc_val global variable
	xSemaphoreGiveFromISR( xADCValueMutex,  &xHigherPriorityTaskWokenA );


	/* 'Give' the semaphore to unblock the frequency calculation task. */
	xSemaphoreGiveFromISR( xFreqCalcBinarySemaphore, &xHigherPriorityTaskWokenB );

	/* Clear the software interrupt bit using the interrupt controllers
    Clear Pending register. */
	mainCLEAR_INTERRUPT();

	/* Giving the semaphore may have unblocked a task - if it did and the
    unblocked task has a priority equal to or above the currently executing
    task then xHigherPriorityTaskWoken will have been set to pdTRUE and
    portEND_SWITCHING_ISR() will force a context switch to the newly unblocked
    higher priority task.
	 */

	/* A higher priority task was woken if either semaphore/mutex returned pdTRUE */
	xHigherPriorityTaskWokenC = (xHigherPriorityTaskWokenA == pdTRUE ||
			xHigherPriorityTaskWokenA == pdTRUE ? pdTRUE: pdFALSE);

	/* NOTE: The syntax for forcing a context switch within an ISR varies between
    FreeRTOS ports.  The portEND_SWITCHING_ISR() macro is provided as part of
    the Cortex-M3 port layer for this purpose.  taskYIELD() must never be called
    from an ISR! */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWokenC );
}

/*------------------------------------------------------------------------------------------------------------------------*/

/* Timer callback function for switch debouncing */
void vDebounceTimerCallback( TimerHandle_t xTimer )
{
	// If pin is low (button has been released), increment the count of "stable" time,
	// otherwise reset the count to 0 (button being held, or switch bouncing is occuring
	g_ui16StableCount = ( (LPC_GPIO0->FIOPIN & (1 << 9)) == 0 ? g_ui16StableCount + 1 : 0);

	if (g_ui16StableCount <= 10)    // If pin isn't stable yet,
	{
		// restart the debouncing timer.
		xTimerStart(g_xDebounceTimer, portMAX_DELAY);
	}
	else	// Once we've captured a number of consecutive stable values
	{
		// Clear the GPIO interrupt flag
		// Note: this must be done before clearing NVIC
		// pending bit, or the pending bit will be immediately
		// set again.
		LPC_GPIOINT->IO0IntClr |= (1 << 9);

		// Clear the NVIC pending interrupt bit
		NVIC->ICPR[0] |= ( 1 << 21 );

		// Re-enable the interrupt at the NVIC
		NVIC_EnableIRQ(EINT3_IRQn);

		// Re-enable GPIO 0 interrupt 9
		LPC_GPIOINT->IO0IntEnR |= (1 << 9);

		// Reset count for next time we need debouncing
		g_ui16StableCount = 0;
	}
}

/*------------------------------------------------------------------------------------------------------------------------*/

//
// Function for RTC Timer Callback
//
void vRTCTimerCallback( TimerHandle_t xTimer )
{
	// Increment the count of seconds
	lSeconds++;

	// Print seconds, formatted as HH:MM:SS
	printf( "Elapsed time: %02d:%02d:%02d\n", lSeconds / 3600, lSeconds / 60, lSeconds % 60 );
}

/*------------------------------------------------------------------------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* This function will only be called if an API call to create a task, queue
    or semaphore fails because there is too little heap RAM remaining. */
	for( ;; );
}
/*-------------------------------------------------------------------------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will only be called if a task overflows its stack.  Note
    that stack overflow checking does slow down the context switch
    implementation. */
	for( ;; );
}
/*------------------------------------------------------------------------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* This example does not use the idle hook to perform any processing. */
}
/*------------------------------------------------------------------------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This example does not use the tick hook to perform any processing. */
}
