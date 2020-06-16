/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "Task.h"
#include "Queue.h"
#include "semphr.h"
#include "timers.h"

#include "cpu_utils.h"

#define LED_R									(PH0)
#define LED_Y									(PH1)
#define LED_G									(PH2)

#define DEBUG_CREATE_TASK
#define DEBUG_RISING_FLAG

#define DELAY_MS_10								(10)
#define DELAY_MS_100							(DELAY_MS_10*10)


#define printf(...)	\
{	\
	vTaskSuspendAll();	\
	printf(__VA_ARGS__);	\
	fflush(stdout); \
	xTaskResumeAll();	\
}	\


#define mainNORMAL_TASK_PRIORITY           	( tskIDLE_PRIORITY + 0UL )
#define mainABOVENORMAL_TASK_PRIORITY     	( tskIDLE_PRIORITY + 1UL )
#define mainHIGH_TASK_PRIORITY             		( tskIDLE_PRIORITY + 2UL )

TaskHandle_t										xTaskCreateSub;

typedef enum{

	flag_RisingFlag = 0 ,
	flag_bbb ,	
	flag_ccc ,
	
	flag_ddd ,
	
	flag_DEFAULT	
}Flag_Index;

uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))


void vTask_CreateSubTask(void * pvParameters)
{
	(void) pvParameters;
	for( ;; )
	{
		#if defined (DEBUG_CREATE_TASK)
		printf(">>>>TaskSub Delete\r\n");
		#endif /*DEBUG_CREATE_TASK*/
		
		set_flag(flag_RisingFlag,DISABLE);

		/* Delete the Init Thread */ 
		vTaskDelete(xTaskCreateSub);
	}	
}

void vTask_CreateTask( void * pvParameters )
{
	(void) pvParameters;	
	for( ;; )
	{		
		if (is_flag_set(flag_RisingFlag))
		{
			// Perform action here.			
			xTaskCreate(vTask_CreateSubTask, "vTaskSub", configMINIMAL_STACK_SIZE/2, NULL, mainABOVENORMAL_TASK_PRIORITY, &xTaskCreateSub ); 
			
			#if defined (DEBUG_CREATE_TASK)
			printf(">>>>TaskSub Create\r\n");		
			#endif /*DEBUG_CREATE_TASK*/
		}
	}  
}

void vTask_RisingFlag( void * pvParameters )
{
	uint32_t millisec = DELAY_MS_100*5;
	static uint16_t cnt = 0;
	
	portTickType xLastWakeTime;
//	Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	(void) pvParameters;
	for( ;; )
	{
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, ( millisec *configTICK_RATE_HZ / ( ( TickType_t ) 1000 ) ));

		// Perform action here.	
		set_flag(flag_RisingFlag,ENABLE);

		#if defined (DEBUG_RISING_FLAG)
		printf("RisingFlag(%4dms) :%4d (heap : %3d bytes , CPU : %3d )\r\n" ,millisec, ++cnt,xPortGetFreeHeapSize(),osGetCPUUsage());
		#else
		( void ) cnt;		//Remove compiler warning about cnt being set but never used.
		#endif	/*DEBUG_RISING_FLAG*/		
	}  
}

void vTask_100ms( void *pvParameters )
{	
	uint32_t millisec = DELAY_MS_100;

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	(void) pvParameters;
	for( ;; )
	{
        vTaskDelayUntil( &xLastWakeTime, ( millisec *configTICK_RATE_HZ / ( ( TickType_t ) 1000 ) ));
		
		LED_G ^= 1;
	}  
}

void vTask_50ms( void *pvParameters )
{	
	uint32_t millisec = DELAY_MS_10*5;

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	(void) pvParameters;
	for( ;; )
	{
        vTaskDelayUntil( &xLastWakeTime, ( millisec *configTICK_RATE_HZ / ( ( TickType_t ) 1000 ) ));
		
		LED_Y ^= 1;
	}  
}

void vTask_10ms( void *pvParameters )
{	
	uint32_t millisec = DELAY_MS_10;

	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	(void) pvParameters;
	for( ;; )
	{
        vTaskDelayUntil( &xLastWakeTime, ( millisec *configTICK_RATE_HZ / ( ( TickType_t ) 1000 ) ));
		
		LED_R ^= 1;
	}  
}


__weak void vApplicationTickHook( void )
{

}

__weak void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

__weak void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}



void TMR1_IRQHandler(void)
{
	static uint16_t CNT = 0;	
//	static uint32_t log = 0;	
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
	
		if (CNT++ > 1000)
		{		
			CNT = 0;
//			printf("%s : %2d\r\n" , __FUNCTION__ , log++);
		}
    }
}

void TIMER1_HW_Init(void)
{
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void TIMER0_HW_Init(void)
{
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
}

void TIMER0_Polling(uint32_t u32Usec)
{
	TIMER_Delay(TIMER0, u32Usec);
}

void LED_Init(void)
{
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

	TIMER0_HW_Init();
	TIMER1_HW_Init();
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

 
	
    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{	
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());

	LED_Init();
	TIMER1_Init();

    xTaskCreate( vTask_10ms, "10ms"				,configMINIMAL_STACK_SIZE 	, NULL	,mainNORMAL_TASK_PRIORITY		, NULL);
    xTaskCreate( vTask_50ms, "50ms"				,configMINIMAL_STACK_SIZE 	, NULL	,mainNORMAL_TASK_PRIORITY		, NULL);
    xTaskCreate( vTask_100ms, "100ms"			,configMINIMAL_STACK_SIZE 	, NULL	,mainNORMAL_TASK_PRIORITY		, NULL);

	//function flag enable and create sub task
	xTaskCreate(vTask_RisingFlag	,"RisingFlag"		,configMINIMAL_STACK_SIZE	,NULL	,mainNORMAL_TASK_PRIORITY		,NULL);   	
	xTaskCreate(vTask_CreateTask	,"CreateTask"		,configMINIMAL_STACK_SIZE	,NULL	,mainNORMAL_TASK_PRIORITY		,NULL); 
	
    vTaskStartScheduler();

    for( ;; );


}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
