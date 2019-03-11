
#include "one_wire_driver.h"
#include "dwt_stm32_delay.h"

 OWD_StateTypeDef OWD_Reset(OWD_HandleTypeDef * howd)
{
	HAL_GPIO_WritePin(howd->GPIOX,howd->GPIO_PIN,GPIO_PIN_SET);
  DWT_Delay_us(100);
	
	if(HAL_GPIO_ReadPin(howd->GPIOX,howd->GPIO_PIN)==GPIO_PIN_RESET) return OWD_Error;
	HAL_GPIO_WritePin(howd->GPIOX,howd->GPIO_PIN,GPIO_PIN_RESET);
	if(HAL_GPIO_ReadPin(howd->GPIOX,howd->GPIO_PIN)==GPIO_PIN_SET) return OWD_Error;
	DWT_Delay_us(640);

	HAL_GPIO_WritePin(howd->GPIOX,howd->GPIO_PIN,GPIO_PIN_SET);	
	DWT_Delay_us(5);
	if(HAL_GPIO_ReadPin(howd->GPIOX,howd->GPIO_PIN)==GPIO_PIN_RESET) return OWD_Error;
	DWT_Delay_us(57);
	
	for(unsigned int count=600 ; count;count--)
	{
		if(HAL_GPIO_ReadPin(howd->GPIOX,howd->GPIO_PIN)==GPIO_PIN_SET) DWT_Delay_us(1);
		else
			{
			   DWT_Delay_us(count);
				if(HAL_GPIO_ReadPin(howd->GPIOX,howd->GPIO_PIN)==GPIO_PIN_SET)
				return OWD_OK;
				else
				return OWD_Error;
			}		
	}	
	return OWD_Error;	
}

 void	OWD_Transmit_bit(uint8_t bit,OWD_HandleTypeDef * howd)
{
	
	HAL_GPIO_WritePin(howd->GPIOX,howd->GPIO_PIN,GPIO_PIN_SET);
	DWT_Delay_us(10);
	
	
	HAL_GPIO_WritePin(howd->GPIOX,howd->GPIO_PIN,GPIO_PIN_RESET);
	if(bit)
		{
			DWT_Delay_us(5);	
			HAL_GPIO_WritePin(howd->GPIOX,howd->GPIO_PIN,GPIO_PIN_SET);
			DWT_Delay_us(90);	
		}
	else
		{
			DWT_Delay_us(90);	
			HAL_GPIO_WritePin(howd->GPIOX,howd->GPIO_PIN,GPIO_PIN_SET);
			DWT_Delay_us(5);	
		}	
}


	void OWD_Transmit_byte	(uint8_t byte,OWD_HandleTypeDef * howd)
{
	for(uint8_t count=8;count;count--)
		{
			OWD_Transmit_bit(byte&0x01,howd);
			byte>>=1;
		}	
}
 uint8_t OWD_Receive_bit(OWD_HandleTypeDef * howd)
{
 uint8_t bit=0;
		HAL_GPIO_WritePin(howd->GPIOX,howd->GPIO_PIN,GPIO_PIN_SET);
  	DWT_Delay_us(10);	
	//low
		HAL_GPIO_WritePin(howd->GPIOX,howd->GPIO_PIN,GPIO_PIN_RESET);

	//delay 2us
		DWT_Delay_us(2);	

	//high
	  HAL_GPIO_WritePin(howd->GPIOX,howd->GPIO_PIN,GPIO_PIN_SET);

	//delay 5-8us
		DWT_Delay_us(8);	
	
  //sampling...
	 
	  bit=HAL_GPIO_ReadPin(howd->GPIOX,howd->GPIO_PIN);///USER_ERROR
	  
	//delay time slot	
		DWT_Delay_us(80);
	  
	return bit;	
}

	uint8_t   				OWD_Receive_byte	(OWD_HandleTypeDef * howd)
{
	uint8_t byte=0x00;
	
	for(uint8_t mask=0x01;mask;mask<<=1)
	{
		if(OWD_Receive_bit(howd)) byte|=mask;
	}
	return byte;
}
