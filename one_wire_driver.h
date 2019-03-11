#ifndef  ONE_WIRE_DRIVER_H
#define  ONE_WIRE_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif



#include "stm32f1xx_hal.h"
#include "dwt_stm32_delay.h"
typedef struct  
	{
		uint16_t   GPIO_PIN;/*!< Pin Number      */
		GPIO_TypeDef *  GPIOX;/*!< GPIO Type    */
	}OWD_HandleTypeDef;/*!< One Wire Driver Handle Type Def  */

	typedef uint16_t	OWD_StateTypeDef;	
	#define OWD_OK    (0x0000)
	#define OWD_Error (0x0001)
	
	static	unsigned char OWD_Buffer;
	  
	OWD_StateTypeDef 	OWD_Reset(OWD_HandleTypeDef * howd);
	
	void	 						OWD_Transmit_bit	(uint8_t bit,OWD_HandleTypeDef * howd);
	void						OWD_Transmit_byte	(uint8_t bit,OWD_HandleTypeDef * howd); 	
	uint8_t 					OWD_Receive_bit		(OWD_HandleTypeDef * howd);
	uint8_t   				OWD_Receive_byte	(OWD_HandleTypeDef * howd);
	
#ifdef __cplusplus
}
#endif


	
	#endif
//end if ONE_WIRE_DRIVER_H
