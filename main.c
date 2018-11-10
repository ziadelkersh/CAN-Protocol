#include <stdint.h>

#include "GPIO.h"
#include "GPIO_Cfg.h"
#include "M4MemMap.h"

#include "CAN.h"
#include "CAN_Cfg.h"

int main(void)
{
	uint8_t Data[4]={
			  0x12,
			  0x11,
			  0xAB,
			  0x10
	                };
	uint16_t *DataPtr;
	uint16_t Dataout[4];
	uint16_t *DataPtrOut;
	DataPtrOut=&Dataout[0];
	DataPtr=&Data[0];
	/* initialization */
	GPIO_Init();
	CAN_Init();

	/* infinite loop */
	while(1)
	{
		/* Can transmitter */
		CAN_TxData(0,DataPtr,0);
		/* Can receiver */
		CAN_RxData(0,DataPtrOut,0);
		/* Can Status */
		CAN_GetStatus(0,1);
	}

	};
	
	return 0;
}
/* interrupt service routine */
void ISR(void)
{
	GPIO_Write (2,0xff);
}
}
