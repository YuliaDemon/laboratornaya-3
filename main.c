
#include "main.h"
#include <stdio.h>
#include <stdint.h>

#define GPIO_PIN_0                 ((uint16_t)0x0001)  
#define GPIO_PIN_1                 ((uint16_t)0x0002)  
#define GPIO_PIN_2                 ((uint16_t)0x0004)  
#define GPIO_PIN_3                 ((uint16_t)0x0008)  
#define GPIO_PIN_4                 ((uint16_t)0x0010)  
#define GPIO_PIN_5                 ((uint16_t)0x0020)  
#define GPIO_PIN_6                 ((uint16_t)0x0040) 
#define GPIO_PIN_7                 ((uint16_t)0x0080) 
#define GPIO_PIN_8                 ((uint16_t)0x0100)  
#define GPIO_PIN_9                 ((uint16_t)0x0200)  
#define GPIO_PIN_10                ((uint16_t)0x0400) 
#define GPIO_PIN_11                ((uint16_t)0x0800)  
#define GPIO_PIN_12                ((uint16_t)0x1000)  
#define GPIO_PIN_13                ((uint16_t)0x2000)  
#define GPIO_PIN_14                ((uint16_t)0x4000)  
#define GPIO_PIN_15                ((uint16_t)0x8000)  
#define GPIO_PIN_All               ((uint16_t)0xFFFF)

#define PIN_CODE_0                 (GPIOA->IDR & GPIO_PIN_0)  
#define PIN_CODE_1                 ((GPIOA->IDR & GPIO_PIN_4)>>3)  
#define PIN_CODE_2                 ((GPIOA->IDR & GPIO_PIN_6)>>4)  
#define PIN_CODE_3                 ((GPIOA->IDR & GPIO_PIN_7)>>4)  
#define PIN_CODE_4                 ((GPIOA->IDR & GPIO_PIN_8)>>4)  
#define PIN_CODE_5                 ((GPIOA->IDR & GPIO_PIN_9)>>4) 
#define PIN_CODE_6                 ((GPIOA->IDR & GPIO_PIN_10)>>4) 
#define PIN_CODE_7                 ((GPIOA->IDR & GPIO_PIN_12)>>5)

uint8_t  value = 0;                                  
int32_t code=0;                                        //Переменная, принимающая значения с тумблеера
uint32_t button = 0;//Количество нажатий кнопки
uint8_t time = 255;
// функцция вывода значения на светодиоды 
void vivod_value (uint8_t time_out);
//  функция записи значения в массив
void load_value (void);  
//функция задержки 
void delay (uint32_t ticks);
void enter_button(void);
//uint32_t pin_state = 0;
	
int main(void)
{

	RCC->APB2ENR = RCC->APB2ENR | RCC_APB2ENR_IOPCEN_Msk; // Включение тактирования для порта B
	   /* Установка регистра CRL битов MODE(0-7)[1:1] на тип выхода скоростью 50 MHz */
    	GPIOC->CRL = (GPIOC->CRL) | ( GPIO_CRL_MODE0_0 | GPIO_CRL_MODE0_1);
	 	GPIOC->CRL = (GPIOC->CRL) | ( GPIO_CRL_MODE1_0 | GPIO_CRL_MODE1_1);
 		GPIOC->CRL = (GPIOC->CRL) | ( GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1);
		GPIOC->CRL = (GPIOC->CRL) | ( GPIO_CRL_MODE3_0 | GPIO_CRL_MODE3_1);
		GPIOC->CRL = (GPIOC->CRL) | ( GPIO_CRL_MODE4_0 | GPIO_CRL_MODE4_1);
		GPIOC->CRL = (GPIOC->CRL) | ( GPIO_CRL_MODE5_0 | GPIO_CRL_MODE5_1);
		GPIOC->CRL = (GPIOC->CRL) | ( GPIO_CRL_MODE6_0 | GPIO_CRL_MODE6_1);
		GPIOC->CRL = (GPIOC->CRL) | ( GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1);
     

	  /* Установка регистра CRL битов CNF5[0:0] на тип выхода push-pull */
        GPIOC->CRL = (GPIOC->CRL) & ( ~( GPIO_CRL_CNF0_0 | GPIO_CRL_CNF0_1));
		GPIOC->CRL = (GPIOC->CRL) & ( ~( GPIO_CRL_CNF1_0 | GPIO_CRL_CNF1_1));
		GPIOC->CRL = (GPIOC->CRL) & ( ~( GPIO_CRL_CNF2_0 | GPIO_CRL_CNF2_1));
		GPIOC->CRL = (GPIOC->CRL) & ( ~( GPIO_CRL_CNF3_0 | GPIO_CRL_CNF3_1));
		GPIOC->CRL = (GPIOC->CRL) & ( ~( GPIO_CRL_CNF4_0 | GPIO_CRL_CNF4_1));
		GPIOC->CRL = (GPIOC->CRL) & ( ~( GPIO_CRL_CNF5_0 | GPIO_CRL_CNF5_1));
		GPIOC->CRL = (GPIOC->CRL) & ( ~( GPIO_CRL_CNF6_0 | GPIO_CRL_CNF6_1));
		GPIOC->CRL = (GPIOC->CRL) & ( ~( GPIO_CRL_CNF7_0 | GPIO_CRL_CNF7_1));
     
RCC->APB2ENR = RCC->APB2ENR | RCC_APB2ENR_IOPAEN_Msk;  // Включение тактирования для порта A
    /* Установка регистра CRL битов CNF [1:0] на тип входа  pull-up */
	 GPIOA->CRL &= (~GPIO_CRL_CNF0_0);
  GPIOA->CRL |= GPIO_CRL_CNF0_1;
	GPIOA->CRL &= (~GPIO_CRL_CNF4_0);
  GPIOA->CRL |= GPIO_CRL_CNF4_1;
	GPIOA->CRL &= (~GPIO_CRL_CNF6_0);
  GPIOA->CRL |= GPIO_CRL_CNF6_1;
	GPIOA->CRL &= (~GPIO_CRL_CNF7_0);
  GPIOA->CRL |= GPIO_CRL_CNF7_1;
	GPIOA->CRH &= (~GPIO_CRH_CNF8_0);
  GPIOA->CRH |= GPIO_CRH_CNF8_1;
	GPIOA->CRH &= (~GPIO_CRH_CNF9_0);
  GPIOA->CRH |= GPIO_CRH_CNF9_1;
	GPIOA->CRH &= (~GPIO_CRH_CNF10_0);
  GPIOA->CRH |= GPIO_CRH_CNF10_1;
	GPIOA->CRH &= (~GPIO_CRH_CNF12_0);
  GPIOA->CRH |= GPIO_CRH_CNF12_1;
	
    enter_button();
			
	while (1)
	{
                 if (button == 0 )
                 {
                 vivod_value (time);
                 }
		        if (button == 1 )
						{
                        vivod_value (time);
						load_value();
						}
					 if (button ==2)
						{
                     // GPIOC->BSRR = value;
				   	//load_value();
		       // vivod_value (time);
						}
						if (button == 3)
						{
						GPIOC->BRR = 0xFF;
						value = 0;
						}
						if (button == 4)
						{
						button = 0;
						//on_off = 0;
						time = code;
						}
					}						
	}

void vivod_value (uint8_t time_out)
	 {	 
         GPIOC->BSRR = value;
		// GPIOC->BRR = value;
         delay (time_out);
         GPIOC->BRR = value;
		 value= value +1;
         
		 if (value == 255)
		 {
		 value = 0;
		 }
		//delay (time_out);
	}
void delay (uint32_t ticks)
{
	for (uint32_t i = 0; i < (1000*(1+ticks)); i++)
	{
	}
}
void enter_button(void)
{
	/****************************************************************/
	/*Включение прерываний от пина 13*/
	/****************************************************************/
	//Включение тактирования на блок альтернативных функций
RCC->APB2ENR = RCC->APB2ENR | RCC_APB2ENR_AFIOEN;
//Разрешить прерывание 13 пина порта С
AFIO->EXTICR[3] = 0x0020;
//Разрешить прерывание 13 линии
EXTI-> IMR|=(EXTI_IMR_MR13);
EXTI-> EMR|=(EXTI_EMR_MR13);
//Прерывание 13 линии по спадающему фронту фронту
EXTI->RTSR|=(EXTI_RTSR_TR13);
/* Разрешение прерываний */
NVIC->ISER[1] = (uint32_t)(1UL << (((uint32_t)EXTI15_10_IRQn) & 0x1FUL));
}
/*Обработчик прерывания*/
void EXTI15_10_IRQHandler (void)
{

EXTI->PR |= GPIO_PIN_13;
	
button=button+1;
}
void load_value (void)
	{
		// Считываем состояние портов
    code = 0;		
	    code |= PIN_CODE_0;  
		code |= PIN_CODE_1;
		code |= PIN_CODE_2;
		code |= PIN_CODE_3;
		code |= PIN_CODE_4;
		code |= PIN_CODE_5;
		code |= PIN_CODE_6;
		code |= PIN_CODE_7;
	}