#include "main.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
void SystemClock_Config(void);
static void MX_GPIO_Init(void);


void debugMessage()
{
	char message[] = "Debug!\r\n";
	//char message[25];
	//sprintf(message, "P: %i L: %i %i\r\n", a,b,val);
	for(int i =0; i < strlen(message); i++)
	{
		while(!(USART1->SR & (1<<7)));
		USART1->DR = message[i];
		//while(!(USART1->SR & (1<<6)));
	}
}
void pinsDebug(int H, int P, int V)
{
	char message[100];
	sprintf(message, "P: %i H: %i V: %i\r\n", P,H,V);
	for(int i =0; i < strlen(message); i++)
	{
		while(!(USART1->SR & (1<<7)));
		USART1->DR = message[i];
		//while(!(USART1->SR & (1<<6)));
	}
}

void sendPix(char val)
{
	//char message[] = "Debug!\r\n";
	char message = val;
		while(!(USART1->SR & (1<<7)));
		USART1->DR = message;
		//while(!(USART1->SR & (1<<6)));
}

#define height  240
#define width  320
volatile int bytes = 0;;
volatile int busyFlag = 0;
bool decyzja = 0;
volatile int PBCount = 0;
volatile int VSYNCount = 0;
volatile int HREFcount = 0;
volatile int PCLKcount = 0;
bool HREF_reset = 0;
volatile char PixHREF[308];
volatile int pixNum = 0;

void EXTI15_10_IRQHandler(void)
{
	VSYNCount++;
	if(VSYNCount == 1)
	{
		EXTI->FTSR &= ~(1<<12);
		EXTI->RTSR |= (1<<12);
		EXTI->IMR |= (1<<8);

		EXTI->PR |= (1<<12);
	}
	else if(VSYNCount == 2)
	{
		//EXTI->IMR &= ~(1<<0);
		EXTI->IMR &= ~(1<<8);
		EXTI->IMR &= ~(1<<12);
		busyFlag = 0;
		pinsDebug(HREFcount,PCLKcount,VSYNCount);

		/*for(int i =0; i < 240; i++)
		{
			pinsDebug(HREFcount,PixHREF[i],VSYNCount);
		}*/
	}
	EXTI->PR |= (1<<12);
}
void EXTI9_5_IRQHandler(void)
{
	HREFcount++;
	PCLKcount = 0;
	while(GPIOA->IDR & (1<<8))
	{
		while(!(GPIOB->IDR & (1<<0)));
		PixHREF[PCLKcount++] = (GPIOA->IDR & (0xFF));
		//PCLKcount++;
		while(GPIOB->IDR & (1<<0)); // WAIT FOR PLCK FE
	}
	EXTI->PR |= (1<<8);
	for(int i=0; i< PCLKcount; i++)
	{
		while(!(USART1->SR & (1<<7)));
		USART1->DR = PixHREF[i];
	}

}
/*
void EXTI0_IRQHandler(void)
{
	PCLKcount++;
	EXTI->PR |= (1<<0);
}*/
void buttonInit()
{
	RCC->APB2ENR |= (1<<3);
	GPIOB->CRH &= ~(0xF<<24);
	GPIOB->CRH |= (0x8<<24);
	GPIOB->ODR |= (1<<14);

}


void USART1init()
{
	RCC->APB2ENR |= (1<<2)|(1<<14);
	USART1->CR1 |=(1<<13)|(1<<3);
	USART1->BRR = 72000000/921600;
	//USART1->BRR = 72000000/115200;
	GPIOA->CRH &= ~(0xF<<4);
	GPIOA->CRH |= (0xB<<4); // PA9 TX
	//TEST
	char message[] = "USART1 ON!\r\n";
	for(int i =0; i < strlen(message); i++)
	{
		while(!(USART1->SR & (1<<7)));
		USART1->DR = message[i];
		//while(!(USART1->SR & (1<<6)));
	}
}


void I2CSend(uint8_t* message)
{


	I2C2->CR1 |= (1<<8); //START
	uint8_t correct =0;
	while(!(I2C2->SR1 & (1<<0)));

	I2C2->DR = (message[0]<<1);
	HAL_Delay(100);
	if(I2C2->SR1 & (1 << 1)) correct ++;



    // Dummy read
    volatile uint32_t temp = I2C2->SR1;
    temp = I2C2->SR2;
    (void)temp;


	I2C2->DR = message[1];
	while(!(I2C2->SR1 & (1<<7)));
	HAL_Delay(10);
	if (!(I2C2->SR1 & (1<<10)))  correct ++;



	I2C2->DR = message[2];
	while(!(I2C2->SR1 & (1<<7)));
	HAL_Delay(10);
	if (!(I2C2->SR1 & (1<<10)))  correct ++;


	 I2C2->CR1 |= (1<<9);
	// char wiadomosc[50];


	    
	    char wiadomosc[50];
	    sprintf(wiadomosc, "Reg %02X ustawiono na %02X\r\n", message[1], message[2]);


		for(int i =0; i < strlen(wiadomosc); i++)
		{
			while(!(USART1->SR & (1<<7)));
			USART1->DR = wiadomosc[i];
		}
}

void XCLKinit()
{
	RCC->APB1ENR |= (1<<1);
	RCC->APB2ENR |= (1<<3);
	GPIOB->CRL &= ~(0x7<<4);
	GPIOB->CRL |= (0xB<<4);
	//GPIOB->ODR |= (1<<1);;
	TIM3->CR1 |= (1<<0);
	TIM3->CCMR2 &= ~(0xF<<12);
	TIM3->CCMR2 |= (6<<12);
	//TIM3->PSC = 71;
	TIM3->ARR = 2;
	TIM3->CCR4  = 1;
	TIM3->CCER |= (1<<12);


	char message[] = "XCLK ON!\r\n";
	for(int i =0; i < strlen(message); i++)
	{
		while(!(USART1->SR & (1<<7)));
		USART1->DR = message[i];
		//while(!(USART1->SR & (1<<6)));
	}
}

void RestInit()
{
	//PCLK PB0
	RCC->APB2ENR |= (3<<2);
	//PCLK PB0
	GPIOB->CRL &= ~(0xF<<0);
	GPIOB->CRL |= (8<<0);

	//VSYNC PB12
	GPIOB->CRH &= ~(0xF<<16);
	GPIOB->CRH |= (0x4<<16);

	//PA0-PA7 [D0-D7]
	GPIOA->CRL &= ~(0xFFFFFFFF<<0);
	GPIOA->CRL |= (0x44444444<<0);

	//HSYNC PA8
	GPIOA->CRH &= ~(0xF<<0);
	GPIOA->CRH |= (0x8<<0);
}

void OV7670init()
{
	//I2C PB10 PB11
	RCC->APB2ENR |= (1<<3);
	RCC->APB1ENR |= (1<<22);
	I2C2->CR2 = 36;
	I2C2->CCR = 30;
	I2C2->CCR |= (1<<15);
	I2C2->TRISE = 12;
	I2C2->CR1 |= (1<<0);
	GPIOB->CRH &= ~(0xFF<<8);
	GPIOB->CRH |= (0xFF<<8);
	XCLKinit();
	RestInit();
	//XCLK PB1
	/*
	RCC->APB1ENR |= (1<<1);
	RCC->APB2ENR |= (1<<3);
	GPIOB->CRL &= ~(0x7<<4);
	GPIOB->CRL |= (0xB<<4);
	TIM3->CR1 |= (1<<0);
	TIM3->CCMR2 &= ~(0xF<<12);
	TIM3->CCMR2 |= (6<<12);
	TIM3->ARR = 2;
	TIM3->CCR4  = 1;
	TIM3->CCER |= (1<<12);
	*/



	char message[] = "OV7670 ON!\r\n";
	for(int i =0; i < strlen(message); i++)
	{
		while(!(USART1->SR & (1<<7)));
		USART1->DR = message[i];
		//while(!(USART1->SR & (1<<6)));
	}


}

void readOV7670(uint8_t *request)
{
	//request[0] adres
	//request[1] rejestr
	uint8_t received = 0;
	I2C2->CR1 |= (1<<8);
	while(!(I2C2->SR1 & (1<<0)));
	I2C2->DR = (request[0]<<1); //CONNECTING TO DEVICE
	while(!(I2C2->SR1 & (1 << 1)));

	(void) I2C2->SR1;
	(void) I2C2->SR2;
	I2C2->DR = request[1];
	while(!(I2C2->SR1&(1<<7))); //CONNECTING TO REGISTER
	I2C2->CR1 |= (1<<9);
	I2C2->CR1 |= (1<<8);
	while(!(I2C2->SR1 & (1<<0))); //REPEATED START
	I2C2->DR = (request[0]<<1)|1; //CONNECTING TO DEVICE
	while(!(I2C2->SR1 & (1 << 1)));

	(void) I2C2->SR1;
	(void) I2C2->SR2;

	I2C2->CR1 &= ~(1<<10);
	while(!(I2C2->SR1 & (1<<6)));
	received  = I2C2->DR;

	I2C2->CR1 |= (1<<9);

    char wiadomosc[40];
    sprintf(wiadomosc, "Z rejestru %02X urzadzenia %02X uzyskano %02X\r\n", request[1], request[0], received);
    for (int i = 0; i < strlen(wiadomosc); i++) {
        while (!(USART1->SR & (1 << 7)));
        USART1->DR = wiadomosc[i];
    }
}

void setOV7670()
{
	uint8_t message[3];
	//message[0] = 0x21;
	// | [0] adres | [1]rejestr | [2] wartosc |
	HAL_Delay(100);
	message[0] = 0x21;
	//message[1] = 0x0D; message[2] = 0x20;
	//I2CSend(message);
	//readOV7670(message);
	//message[1] = 0x42; message[2] = 0x80;
	//I2CSend(message);
	//readOV7670(message);

	message[1] = 0x12; message[2] = 0x80;
	I2CSend(message); //RESET
	readOV7670(message);
	HAL_Delay(100);
	message[1] = 0x72; message[2] = 0x00;
	//I2CSend(message);
	readOV7670(message);

	message[1] = 0x40; message[2] = 0b11110000;
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x15; message[2] = 0x18;
	readOV7670(message);

	message[1] = 0x12; message[2] = 0b00001100;
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x32; message[2] = 0x00;
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x17; message[2] = 0x11;
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x18; message[2] = 0x61;
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x11; message[2] = 0b10111001;
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x3E; message[2] = 0x00;
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x73; message[2] = 0x01;
	//I2CSend(message);
	readOV7670(message);


	message[1] = 0x13; message[2] = 0x00;
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x10; message[2] = 0x00; //MSB EXP
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x45; message[2] = 0x0F; //LSB EXP
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x01; message[2] = 0x1E; //R GAIN
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x02; message[2] = 0x10; //G GAIN
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x03; message[2] = 0x04; //B GAIN
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x24; message[2] = 0x1A; // MAIN GAIN
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x25; message[2] = 0x09; // LOW GAIN
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x26; message[2] = 0x15; //HI GAIN
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x56; message[2] = 0x0A; // KONTRAST
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x7C; message[2] = 0x05; //GAMMA
	I2CSend(message);
	readOV7670(message);

	message[1] = 0x32; readOV7670(message); // HREF
	message[1] = 0x17; readOV7670(message); // HSTART
	message[1] = 0x18; message[2] = 0xB1;readOV7670(message); // HSTOP
	message[1] = 0x03; readOV7670(message); // VREF
	message[1] = 0x19; readOV7670(message); // VSTART
	message[1] = 0x1A; readOV7670(message); // VSTOP
	message[1] = 0x3E; readOV7670(message); //COM14
	message[1] = 0x70; readOV7670(message);
	message[1] = 0x71; readOV7670(message); //COM14

	HAL_Delay(100);
	debugMessage();
}


int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  USART1init();
  OV7670init();
  //XCLKinit();
  setOV7670();
  MX_GPIO_Init();
  RCC->APB2ENR |= (1<<0);
  RCC->APB2ENR |= (1<<3);
  buttonInit();

  bool p_button=1;
  bool button =1 ;

  //PRZERWANIE PB12 VSYNC
  AFIO->EXTICR[3] |= (1<<0);
  EXTI->FTSR |= (1<<12); // FT PB12

  //PRZERWANIE  PA8 HREF
  AFIO->EXTICR[2] &= ~(1<<0);
  EXTI->RTSR |= (1<<8); //RT PA8

  // PRZERWANIE PB0  PCLK
 // AFIO->EXTICR[0] |= (1<<0);
 //EXTI->RTSR |= (1<<0); // wlaczenie przerwan PCLK

  NVIC_EnableIRQ(EXTI15_10_IRQn);//PB12 VSYNC
  NVIC_EnableIRQ(EXTI9_5_IRQn); //PA8 HREF
  //NVIC_EnableIRQ(EXTI0_IRQn); //PCLK

  while(1)
  {
	  button = (GPIOB->IDR & (1<<14));
	  if(button == 1 && p_button == 0 && busyFlag!= 1)
	  {
		  //bytes = 0;
		  HREF_reset = 0;
		  VSYNCount = 0;
		  HREFcount = 0;
		  PCLKcount = 0;
		  busyFlag = 1;
		  EXTI->IMR |= (1<<12); // PRZERWANIE VSYNC ON
	  }
	  p_button = button; //FE przycisk
  }
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
