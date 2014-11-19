/*
 * Original FreeRTOS "Example12" project modified by:
 *
 * My Hong, Drew May, Joshua Udd
 * TCSS 422
 * Autumn 2014
 * University of Washington Tacoma
 *
 * Original example was modified to include sampling from the ADC
 * and calculating the frequency of a sin wave which was then printed
 * when a button on P0.9 was pressed.
 *
 * During presentation of project to the professor, the button functionality
 * did not work correctly.
 */

/*
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
*/

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/* Demo includes. */
#include "basic_io.h"

/* Library includes. */
#include "LPC17xx.h"

/* The interrupt number to use for the software interrupt generation.  This
could be any unused number.  In this case the first chip level (non system)
interrupt is used, which happens to be the watchdog on the LPC1768. */
#define mainSW_INTERRUPT_ID		( 0 )

/* Macro to force an interrupt. */
#define mainTRIGGER_INTERRUPT()	NVIC_SetPendingIRQ( mainSW_INTERRUPT_ID )

/* Macro to clear the same interrupt. */
#define mainCLEAR_INTERRUPT()	NVIC_ClearPendingIRQ( mainSW_INTERRUPT_ID )

/* The priority of the software interrupt.  The interrupt service routine uses
an (interrupt safe) FreeRTOS API function, so the priority of the interrupt must
be equal to or lower than the priority set by
configMAX_SYSCALL_INTERRUPT_PRIORITY - remembering that on the Cortex-M3 high
numeric values represent low priority values, which can be confusing as it is
counter intuitive. */
#define mainSOFTWARE_INTERRUPT_PRIORITY 		( 5 )
#define CROSSOVER_VALUE                         2000  //Defines the "zero" line of the sine wave
#define SAMPLE_RATE_MS_DIVIDE                     2   //4 = Sample every 0.25 milliseconds. (Milliseconds / SAMPLE_RATE_MS_DIVIDE)
#define DELAY_ERROR                               12   //A error factor to account for additional instruction time

/* The tasks to be created. */
static void vHandlerTask( void *pvParameters );
static void vPeriodicTask( void *pvParameters );
static void vCalculateTask( void *pvParameters );
static void vPrintTask( void *pvParameters );

/* Enable the software interrupt and set its priority. */
static void prvSetupSoftwareInterrupt();

/* The service routine for the interrupt.  This is the interrupt that the
task will be synchronized with. */
void vSoftwareInterruptHandler( void );

/*-----------------------------------------------------------*/

/* Declare a variable of type xSemaphoreHandle.  This is used to reference the
semaphore that is used to synchronize a task with an interrupt. */
xSemaphoreHandle xBinarySemaphore;

/*
 * Binary semaphore that signals the print frequency task.
 */
xSemaphoreHandle xBinaryButtonSemaphore;

/*
 * This is the current runtime in milliseconds. This is used to
 * determine the period.
 */
int real_time_clock_ms;

/*
 * The queue that holds the ADC values.
 */
xQueueHandle xQueue;

/*
 * The average frequency of the Sine wave.
 */
double avg_frequency;
/*-----------------------------------------------------------*/

int main( void )
{

	//Set ADC Registers
    LPC_SC->PCONP |= 1 << 12; // set PCADC bit
    LPC_PINCON->PINSEL1 = (LPC_PINCON->PINSEL1 & ~(0X3 << 14)) | (0X1 << 14); // set 15:14 to 01
    LPC_PINCON->PINMODE1 = (LPC_PINCON->PINMODE1 & ~(0X3 << 14)) | (0X1 << 15); // set 15:14 to 10
    LPC_ADC->ADCR = 1 | (2 << 8) | (1 << 14) | (1 << 21);
    //Set button registers
    LPC_PINCON->PINSEL0 |= (0 << 19); // set 19:18 to 00. Activates GPIO on H[5]
    LPC_PINCON->PINMODE0 |= (0 << 19); // set 19:18 to 00. Activates Pull up resistor (Active Low Button)
    LPC_GPIOINT->IO0IntEnF |= (1 << 9);   //Enable falling edge interrupt on Pin 0.9

    NVIC_SetPriority(EINT3_IRQn,0x08);    //Set priority of interrupt
    NVIC_EnableIRQ(EINT3_IRQn);           //Enable the interrupt service routine

    avg_frequency = 0; //Reset average

    xQueue = xQueueCreate(1, sizeof(int)); //Queue of size 1 to hold the adc_val

    /* Before a semaphore is used it must be explicitly created.  In this example
    a binary semaphore is created. */
    vSemaphoreCreateBinary( xBinarySemaphore );
    vSemaphoreCreateBinary( xBinaryButtonSemaphore);

    /* Check the semaphores and queue were created successfully. */
    if( xBinarySemaphore != NULL && xBinaryButtonSemaphore != NULL && xQueue != NULL)
    {
    	/* Enable the software interrupt and set its priority. */
    	prvSetupSoftwareInterrupt();

        /* Create the 'handler' task.  This is the task that will be synchronized
         * with the interrupt.  The handler task is created with a high priority to
         * ensure it runs immediately after the interrupt exits.
         * This task reads from the ADC and keeps track of the real time clock
         */
        xTaskCreate( vHandlerTask, "Handler", 240, NULL, 4, NULL );

        /* Create the task that will periodically generate a software interrupt.
         * This is created with a priority below the handler task to ensure it will
         * get preempted each time the handler task exits the Blocked state.
         * This task creates a 1ms delay
         */
        xTaskCreate( vPeriodicTask, "Periodic", 240, NULL, 1, NULL );

        /* Create the calculating task that will run immediately after the handler
         * task. This task takes the data read from the ADC and computes the frequency
         * of a sin wave if able.
         */
        xTaskCreate( vCalculateTask, "Calculate", 240, NULL, 2, NULL);

        /* Create the printing task that will print out the average frequency
         * after a button is pressed.
         */
        xTaskCreate( vPrintTask, "Print Frequency", 240, NULL, 3, NULL);

        /* Start the scheduler so the created tasks start executing. */

        __enable_irq(); //Enable interrupts

        vTaskStartScheduler();
    }

    /* If all is well we will never reach here as the scheduler will now be
    running the tasks.  If we do reach here then it is likely that there was
    insufficient heap memory available for a resource to be created. */
    for( ;; );
    return 0;
}
/*-----------------------------------------------------------*/

void EINT3_IRQHandler(void) { //Interrupt handler for Button
	//We only have one interrupt so no need to check what called the interrupt
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	//Load another binary semaphore
	xSemaphoreGiveFromISR( xBinaryButtonSemaphore, &xHigherPriorityTaskWoken);
	LPC_GPIOINT->IO0IntClr |= (1 << 9); // clear the status of Pin 0.9
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/

static void vCalculateTask( void *pvParameters ) {
	//Calculates the frequency
	int prev_adc_value;
	int queue_value;
	int current_time;
	double avg_period = 0;
	int period_counts = 0;
	int frequency_counts = 0; //Number of frequencies calculated
	portBASE_TYPE xQueueStatus; //Status of xQueueReceive
	for ( ;; ) {
		prev_adc_value = queue_value; //Place the old data

		//Grab data from the queue used with the Handler task
		xQueueStatus = xQueueReceive( xQueue, &queue_value, portMAX_DELAY); //Grab adc_value from queue and wait forever if queue is empty
		current_time = real_time_clock_ms;
		if (xQueueStatus == pdPASS) {
			if (prev_adc_value < CROSSOVER_VALUE && queue_value >= CROSSOVER_VALUE + 50 && current_time != 0) { //We have crossed the sin threshold (ignore 0 periods
				avg_period = ((avg_period * period_counts) + ((double) (current_time - DELAY_ERROR) / (double) SAMPLE_RATE_MS_DIVIDE)) / (period_counts + 1); //These periods are good, lets do frequency
				avg_frequency = 1 / (avg_period * 0.001);
				printf("Frequency = %f\n", avg_frequency); //Print statement for debugging purposes
				period_counts++;
				real_time_clock_ms = 0; //Reset the clock
			}
		}
	}
}
/*-----------------------------------------------------------*/

static void vHandlerTask( void *pvParameters )
{

	int adc_val = 0; //ADC Voltage
    /* As per most tasks, this task is implemented within an infinite loop.

    Take the semaphore once to start with so the semaphore is empty before the
    infinite loop is entered.  The semaphore was created before the scheduler
    was started so before this task ran for the first time.*/
    xSemaphoreTake( xBinarySemaphore, 0 );

    for( ;; )
    {

        /* Use the semaphore to wait for the event.  The task blocks
        indefinitely meaning this function call will only return once the
        semaphore has been successfully obtained - so there is no need to check
        the returned value. */
        xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );

        //Semaphore has been taken successfully, read from ADC
      	real_time_clock_ms++;
    	//Read from the ADC
        LPC_ADC->ADCR |= 1 << 24; // start conversion
        while((LPC_ADC->ADDR0 & (1 << 31)) == 0); // wait for conversion to finish
        adc_val = (LPC_ADC->ADDR0 >> 4) & 0xfff; // read value
        //printf("ADC Val = %d\n", adc_val);
        //Put the adc_val into a queue for the computing task to read
        xQueueSend(xQueue, &adc_val, portMAX_DELAY); //Send adc_val to queue and wait indefinitely if queue is full
    }
}
/*-----------------------------------------------------------*/


static void vPrintTask( void *pvParameters ) {
	xSemaphoreTake( xBinaryButtonSemaphore, 0 );

	for ( ;; ) {
		xSemaphoreTake( xBinaryButtonSemaphore, portMAX_DELAY );
		printf("Frequency = %f\n", avg_frequency); //Print the average frequency
	}
}
/*-----------------------------------------------------------*/

static void vPeriodicTask( void *pvParameters )
{

    /* As per most tasks, this task is implemented within an infinite loop. */
    for( ;; )
    {
        /* This task is just used to 'simulate' an interrupt.  This is done by
        periodically generating a software interrupt. */
    	vTaskDelay(portTICK_RATE_MS / SAMPLE_RATE_MS_DIVIDE);

        mainTRIGGER_INTERRUPT();
    }
}
/*-----------------------------------------------------------*/

static void prvSetupSoftwareInterrupt() //DONE
{
	/* The interrupt service routine uses an (interrupt safe) FreeRTOS API
	function so the interrupt priority must be at or below the priority defined
	by configSYSCALL_INTERRUPT_PRIORITY. */
	NVIC_SetPriority( mainSW_INTERRUPT_ID, mainSOFTWARE_INTERRUPT_PRIORITY );

	/* Enable the interrupt. */
	NVIC_EnableIRQ( mainSW_INTERRUPT_ID );
}
/*-----------------------------------------------------------*/

void vSoftwareInterruptHandler( void ) //DONE
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* 'Give' the semaphore to unblock the task. */
    xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken );

    /* Clear the software interrupt bit using the interrupt controllers
    Clear Pending register. */
    mainCLEAR_INTERRUPT();

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
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* This function will only be called if an API call to create a task, queue
	or semaphore fails because there is too little heap RAM remaining. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will only be called if a task overflows its stack.  Note
	that stack overflow checking does slow down the context switch
	implementation. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* This example does not use the idle hook to perform any processing. */
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This example does not use the tick hook to perform any processing. */
}







