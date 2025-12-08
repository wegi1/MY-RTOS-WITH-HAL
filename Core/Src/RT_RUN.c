
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
//=====================================================================================================================
#include "leds.h"
#include "uart.h"
#include "MY_RTOS.h"
#include "stm32f4xx_hal.h"
//=====================================================================================================================
extern void iddle_task(void);
extern uint32_t my_utoa(uint8_t * buf, uint32_t data);
extern uint32_t my_itoa(uint8_t * buf, uint32_t data);
extern void my_htoa32(uint8_t * buf, uint32_t data);
extern void my_htoa64(uint8_t * buf, uint64_t data);
extern void set_LF(uint32_t * buff_ad);
extern uint8_t idx[];
//---
uint8_t idx[20];

//---
//=====================================================================================================================
//---
const  char clr_scr[] = {"\033[2J\033[3J\033[92m\033[0;0f"};
const  char run_INFO[] = {"MY RTOS coded by wegi\r\nRAW VERSION running \r\r\n\nSTARTED !!!\r\r\n\n"};
//---
//=====================================================================================================================



//=====================================================================================================================
void RT_RUN(void)
{
	HAL_NVIC_SetPriority(EXTI0_IRQn, 14, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

//---
	OS_start();   // start RTOS and immediatelly run the iddle task
//---

	while(1){;}
}
//=====================================================================================================================
//=====================================================================================================================
//=====================================================================================================================
void task1_handler(void)
{
	uint8_t idx2[20];
	uint8_t idx3[20];
	bool result ;
	static uint32_t cntr = 0;
	uint32_t * pdata = (uint32_t*) 0x40004408;

	while(1){
		led_toggle(LED_GREEN);
		if((cntr %5) == 0) {
			result = false;
			while( result != true ) {
				result = put_Queue1_task((uint32_t) &clr_scr[0]);
			}
			result = false;
			while( result != true ) {
				result = put_Queue1_task((uint32_t) &run_INFO[0]);
			}
		}
		result = false;
		while( result != true ) {
			result = put_Queue1_task((uint32_t)"\r\nPrint from TASK1\r\n");
		}

		my_utoa(&idx[0], cntr);


		cntr ++;
		result = false;
		while( result != true ) {
			result = put_Queue1_task((uint32_t) &idx[0]);
		}

		result = false;
		while( result != true ) {
			result = put_Queue1_task((uint32_t)" Seconds from start...\r\n");
		}


		my_htoa32(&idx3[0], (uint32_t) pdata);
		my_htoa32(&idx2[0], (uint32_t) pdata[0]);
		set_LF((uint32_t*) &idx2[10]);

		result = false;
		while( result != true ) {
			result = put_Queue1_task((uint32_t)"Register at: ");
		}
		result = false;
		while( result != true ) {
			result = put_Queue1_task((uint32_t) &idx3[0]);
		}

		result = false;
		while( result != true ) {
			result = put_Queue1_task((uint32_t)" contains value ");
		}

		result = false;
		while( result != true ) {
			result = put_Queue1_task((uint32_t) &idx2[0]);

		}
		OS_delay(1000);
	}
}
//=====================================================================================================================
void task2_handler(void)
{
	bool result ;
	while(1){
		led_toggle(LED_ORANGE);
		result = false;
		while( result != true ) {
			result = put_Queue1_task((uint32_t)"Print from TASK2\r\n");
		}
		OS_delay(500);
	}
}
//=====================================================================================================================
void task3_handler(void)
{
	bool result ;
	while(1){
		led_toggle(LED_RED);
		result = false;
		while( result != true ) {
			result = put_Queue1_task((uint32_t)"Print from TASK3\r\n");
		}
		OS_delay(250);
	}
}
//=====================================================================================================================
void task4_handler(void)
{
	bool result ;
	while(1){
		led_toggle(LED_BLUE);
		result = false;
		while( result != true ) {
			result = put_Queue1_task((uint32_t)"Print from TASK4\r\n");
		}
		OS_delay(125);
	}
}
//=====================================================================================================================
void task5_handler(void)
{

	while(1){

		OS_delay(handle_Queue1());
	}
}
//=====================================================================================================================
void task6_handler(void)
{
	bool result ;
	while(1){
		result = false;
		while( result != true ) {
			result = put_Queue1_task((uint32_t)"\r\nHAL TASK PRINT\r\n\r\n");
		}
		HAL_Delay(3333);
	}
}
//=====================================================================================================================
//=====================================================================================================================
void task7_handler(void)
{
	bool result ;
	while(1){
		result = false;
		while( result != true ) {
			result = put_Queue1_task((uint32_t)"\r\n ... BUTTON PRESSED ... !!!\r\n\r\n");
		}
		OS_delay(20); // debounce delay

		while(1) {
			if((GPIOA->IDR & 1 ) == 0) { break;}
			OS_delay(10); // check every 10ms of button releasse
		}
		result = false;
		while( result != true ) {
			result = put_Queue1_task((uint32_t)"\r\n ....... BUTTON RELEASSED .......\r\n\r\n");
		}
		OS_delay(20);  // debounce delay
		EXTI->PR  = 1; // clear pending bit
		EXTI->IMR = 1; // enable IRQ turn arround

		OS_delay(0xFFFFFFFF); // job is done - disable task
	}
}
//=====================================================================================================================

void HardFault_Handler(void)
{
	for(;;);
}
//---
void MemManage_Handler(void)
{
	for(;;);
}
//---
void BusFault_Handler(void)
{
	for(;;);
}
//=====================================================================================================================
void EXTI0_IRQHandler(void) {
	EXTI->PR  = 1; // clear pending bit
	EXTI->IMR = 0; // disable IRQ by mask

	Unblock_Task(7); // enable Handling button debonce, releasse and back to enable Interrupts
}
//=====================================================================================================================
//=====================================================================================================================
//=====================================================================================================================
//=====================================================================================================================


