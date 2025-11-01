

#include "main.h"


//MAIN FUNCTION
int main(void){

	GPIOA_Config();


	while(1){

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		delay(500000);
	}
}


//GPIOA Configuration
void GPIOA_Config(void){


	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIOA_Init={};

	GPIOA_Init.Pin = GPIO_PIN_5;

	GPIOA_Init.Mode= GPIO_MODE_OUTPUT_PP;

	HAL_GPIO_Init(GPIOA, &GPIOA_Init);

}


void delay(uint32_t delayVal){

	uint32_t i;
	for(i=0; i < delayVal; i++){

	}
}


