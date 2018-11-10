#include "GPIO.h"
#include "GPIO_Cfg.h"
#include "M4MemMap.h"
#include <stdint.h>
typedef volatile uint32_t* const GPIO_RegAddType;
#define PORTS_NUMBER 6u
/*Register memory map*/
#define PORTA_BASE_ADDRESS 0x40004000
#define PORTB_BASE_ADDRESS 0x40005000
#define PORTC_BASE_ADDRESS 0x40006000
#define PORTD_BASE_ADDRESS 0x40007000
#define PORTE_BASE_ADDRESS 0x40024000
#define PORTF_BASE_ADDRESS 0x40025000
static const uint32_t PortsBaseAddressLut[PORTS_NUMBER] =
{       PORTA_BASE_ADDRESS,
	PORTB_BASE_ADDRESS,
	PORTC_BASE_ADDRESS,
	PORTD_BASE_ADDRESS,
	PORTE_BASE_ADDRESS,
	PORTF_BASE_ADDRESS
};
#define GPIO_REG_ADDRESS(CHANNEL_ID,REG_OFFSET)\
(PortsBaseAddressLut[CHANNEL_ID] + REG_OFFSET)

/*Port Control*/
#define GPIODATA_WRITE(DATA,MASK,PORT_ID)  *((GPIO_RegAddType)(GPIO_REG_ADDRESS(PORT_ID,0x000) + (MASK << 2))) = DATA
#define GPIODATA_READ(MASK,PORT_ID)        *((GPIO_RegAddType)(GPIO_REG_ADDRESS(PORT_ID,0x000) + (MASK << 2)))
#define GPIODIR_REG(PORT_ID)               *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x400))

/*Interrupt Control*/
#define GPIOIS_REG(PORT_ID)               *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x404))
#define GPIOIBE_REG(PORT_ID)              *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x408))
#define GPIOIEV_REG(PORT_ID)              *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x40C))
#define GPIOIM_REG(PORT_ID)               *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x410))
#define GPIORIS_REG(PORT_ID)              *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x414))
#define GPIOMIS_REG(PORT_ID)              *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x418))
#define GPIOICR_REG(PORT_ID)              *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x41C))

/*Mode Control*/
#define GPIOAFSEL_REG(PORT_ID)            *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x420))
#define GPIOPCTL_REG(PORT_ID)             *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x52C))
#define GPIOADCCTL_REG(PORT_ID)           *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x530))
#define GPIODMACTL_REG(PORT_ID)           *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x534))

/*Pad control*/
#define GPIODR2R_REG(PORT_ID)           *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x500))
#define GPIODR4R_REG(PORT_ID)           *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x504))
#define GPIODR8R_REG(PORT_ID)           *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x508))
#define GPIOODR_REG(PORT_ID)            *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x50C))
#define GPIOPUR_REG(PORT_ID)            *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x510))
#define GPIOPDR_REG(PORT_ID)            *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x514))
#define GPIOSLR_REG(PORT_ID)            *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x518))
#define GPIODEN_REG(PORT_ID)            *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x51C))
#define GPIOAMSEL_REG(PORT_ID)          *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x528))

/*Commit control*/
#define GPIOLOCK_REG(PORT_ID)          *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x520))
#define GPIOCR_REG(PORT_ID)            *((GPIO_RegAddType)GPIO_REG_ADDRESS(PORT_ID,0x524))

/*Internal symbols*/
#define GPIO_PORT_UNLOCK_VALUE 0x4C4F434B
#define GPIO_INT_SENCE_BIT_NUM 1
#define GPIO_INT_SENCE_MASK (1 << GPIO_INT_EVENT_BIT_NUM)
#define GPIO_INT_EVENT_BIT_NUM 0
#define GPIO_INT_EVENT_MASK (1 << GPIO_INT_EVENT_BIT_NUM)
#define GPIO_INT_BE_BIT_NUM 2
#define GPIO_INT_BR_MASK (1 << GPIO_INT_BE_BIT_NUM)

static uint8_t GPIO_GroupState[GPIO_GROUPS_NUMBER] = {0};

/*A function to initialize all the GPIO Groups in the configurations*/
GPIO_CheckType GPIO_Init(void)
{
	uint8_t LoopIndex;
	uint8_t ErrorFlag = 0;
	GPIO_CheckType RetVar;
	const GPIO_CfgType * CfgPtr;


	for(LoopIndex = 0; (LoopIndex < GPIO_GROUPS_NUMBER) && (ErrorFlag == 0); LoopIndex ++)
	{
		if(GPIO_ConfigParam[LoopIndex].PortId < PORTS_NUMBER)
		{


			/*Enable port clock gate*/
			CfgPtr = & GPIO_ConfigParam[LoopIndex];
			/*enable ADC Clock in ADC Module */
			RCGCADC = 0x03 ;
			RCGCGPIO_REG |= 1 << CfgPtr->PortId;
			/* enable ADC clock in gpio module */
			RCGCGPIO_REG |=1<< CfgPtr->UseACDTrig;
			/*Unlock the group*/
			GPIOLOCK_REG(CfgPtr->PortId) = GPIO_PORT_UNLOCK_VALUE;
			GPIOCR_REG(CfgPtr->PortId)  |= (CfgPtr->PortMask);
			/*Data Control*/
			GPIODIR_REG(CfgPtr->PortId) |= (CfgPtr->PortMask & CfgPtr->PortDirection);
			/*Pad Control*/
			GPIODR2R_REG(CfgPtr->PortId) |= (CfgPtr->PortMask & CfgPtr->Use2mACrt);
			GPIODR4R_REG(CfgPtr->PortId) |= (CfgPtr->PortMask & CfgPtr->Use4mACrt);
			GPIODR8R_REG(CfgPtr->PortId) |= (CfgPtr->PortMask & CfgPtr->Use8mACrt);

			GPIOPDR_REG(CfgPtr->PortId) |= (CfgPtr->PortMask & CfgPtr->UsePullDown);
			GPIOPUR_REG(CfgPtr->PortId) |= (CfgPtr->PortMask & CfgPtr->UsePullUp);
			GPIOODR_REG(CfgPtr->PortId) |= (CfgPtr->PortMask & CfgPtr->UseOpenDrain);

			GPIODEN_REG(CfgPtr->PortId)   |= (CfgPtr->PortMask & CfgPtr->SetPinType);
			GPIOAMSEL_REG(CfgPtr->PortId) |= (CfgPtr->PortMask & ~CfgPtr->SetPinType);

			/*Mode control*/
			GPIOAFSEL_REG(CfgPtr->PortId)  |= (CfgPtr->PortMask & CfgPtr->UseAlterFun);
			GPIOADCCTL_REG(CfgPtr->PortId) |= (CfgPtr->PortMask & CfgPtr->UseACDTrig);
			GPIODMACTL_REG(CfgPtr->PortId) |= (CfgPtr->PortMask & CfgPtr->UseDMATrig);
			GPIO_GroupState[LoopIndex] = 1;
			RetVar = GPIO_OK;
		}
		else
		{
			/*Invalid GroupId*/
			RetVar = GPIO_NOK;
			ErrorFlag = 1;
		}


	}
	return RetVar;
}
/*A function to Digital Write data to a specific group*/
GPIO_CheckType GPIO_Write(uint8_t GroupId,uint8_t GroupData)
{
	const GPIO_CfgType *CfgPtr;
	GPIO_CheckType RetVar = GPIO_NOK;
if(GroupId < GPIO_GROUPS_NUMBER)
{
	if(GPIO_GroupState[GroupId] == 1)
	{
		CfgPtr = &GPIO_ConfigParam[GroupId];
		if((CfgPtr->SetPinType == 0xff) &&
		   (CfgPtr->PortDirection == 0xff) &&
		   (CfgPtr->UseAlterFun == 0x00))
		{
			GPIODATA_WRITE(GroupData,CfgPtr->PortMask,CfgPtr->PortId);
			RetVar = GPIO_OK;
		}
	}
	}
return RetVar;

}
/*A function to Digital read data from a specific group*/
GPIO_CheckType GPIO_Read(uint8_t GroupId,uint8_t* GroupDataPtr)
{
	const GPIO_CfgType *CfgPtr;
	GPIO_CheckType RetVar = GPIO_NOK;
if(GroupId < GPIO_GROUPS_NUMBER)
{
	if(GPIO_GroupState[GroupId] == 1)
	{
		CfgPtr = &GPIO_ConfigParam[GroupId];
		if((CfgPtr->SetPinType == 0xff) &&
		   (CfgPtr->PortDirection == 0x00) &&
		   (CfgPtr->UseAlterFun == 0x00))
		{
			*GroupDataPtr = GPIODATA_READ(CfgPtr->PortMask,CfgPtr->PortId);
			RetVar = GPIO_OK;
		}
	}
	}
return RetVar;
}
/*A function to select which peripheral will be connected to a GPIO pin*/
GPIO_CheckType GPIO_SetAlternFuntion(uint8_t GroupId,uint8_t AlternFuncId)
{

}
/*A function to Select the interrupt event for a specific GPIO Group*/
GPIO_CheckType GPIO_SetInterruptEvent(uint8_t GroupId,GPIO_IntEventType IntEvent,GPIO_IntMaskStatus IntMaskStatus)
{
	
}
/*A function to clear a specific pin interrupt flag*/
GPIO_CheckType GPIO_ClrInterruptFlag(uint8_t GroupId)
{

}
/*A function to Get a specific pin interrupt status*/
GPIO_CheckType GPIO_GetInterruptStatus(uint8_t GroupId,GPIO_IntStatus *IntStatusPtr)
{

}

static uint8_t GetPinNumber(uint8_t PinMask)
{

}
