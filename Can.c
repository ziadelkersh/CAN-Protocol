/*
 * Can.c
 *
 *  Created on: Nov 10, 2018
 *      Author: ziad
 */
#include "CAN.h"
#include "CAN_Cfg.h"
#include "Utilities.h"
#include <stdint.h>

typedef volatile uint32_t * const CAN_RegAddType;

#define CAN_NUMBER 2u
#define CAN0BaseAddress 0x40040000
#define CAN1BaseAddress 0x40041000

static const uint32_t CANBaseAddressLut[CAN_NUMBER] =
{
		CAN0BaseAddress,
		CAN1BaseAddress
};

/* defining Clock gating control register 0 */
#define RCGC0                    *((CAN_RegAddType)(0x400FE100))

/* Defining Control Registers */
#define CANCTL(CAN_ID)                  *((CAN_RegAddType)(CANBaseAddressLut[CAN_ID] + 0x000))
#define CANSTS(CAN_ID)                  *((CAN_RegAddType)(CANBaseAddressLut[CAN_ID] + 0x004))
#define CANERR(CAN_ID)                  *((CAN_RegAddType)(CANBaseAddressLut[CAN_ID] + 0x008))
#define CANBIT(CAN_ID)                  *((CAN_RegAddType)(CANBaseAddressLut[CAN_ID] + 0x00C))
#define CANINT(CAN_ID)                  *((CAN_RegAddType)(CANBaseAddressLut[CAN_ID] + 0x010))
#define CANTST(CAN_ID)                  *((CAN_RegAddType)(CANBaseAddressLut[CAN_ID] + 0x014))
#define CANBRPE(CAN_ID)                 *((CAN_RegAddType)(CANBaseAddressLut[CAN_ID] + 0x018))


/* Registers Initializations */
#define CAN0_EN                         1<<24

#define CANCTL_INIT                     0x01
#define CANCTL_TEST                     0x80
#define CANCTL_CCE                      0x40
#define CANCTL_DAR                      0x20
#define CANCTL_IE                       0x02

#define CANBIT_TSEG2_MASK               12
#define CANBIT_TSEG2_EN                 0x07
#define CANBIT_TSEG1_MASK               8
#define CANBIT_TSEG1_EN                 0x0F
#define CANBIT_SJW_MASK                 6
#define CANBIT_SJW_EN                   0x03
#define CANBIT_BRP_EN                   0x03F

#define CANBRPE_EN                      0x3C0


#define CANIFnCRQ_Busy                  0x8000

#define CANIFnCMSK_WRNRD                0x80
#define CANIFnCMSK_MASK                 0x40
#define CANIFnCMSK_ARB                  0x20
#define CANIFnCMSK_CONTROL              0x10
#define CANIFnCMSK_DATA_EN              0x03

#define CANIFnARB2_IDE_MASK             14
#define CANIFnARB2_DIR                  0x2000
#define CANIFnARB2_MSGVAL               0x8000
#define CANIFnARB2_ID_STANDARD_MASK     2
#define CANIFnARB2_ID_STANDARD_EN       0x7FF
#define CANIFnARB2_ID_EXTENDED_MASK     16
#define CANIFnARB2_ID_EXTENDED_EN       0x1FFF0000
#define CANIFnARB1_ID_EXTENDED_EN       0xFFFF

#define CANIFnMCTL_TXRQST               0x100
#define CANIFnMCTL_UMASK                0x1000
#define CANIFnMCTL_EOB                  0x80


#define CANSTS_BOFF_MASK                7
#define CANSTS_BOFF_EN                  0x80
#define CANSTS_TXOK_MASK                3
#define CANSTS_TXOK_EN                  0x08
#define CANSTS_RXOK_MASK                4
#define CANSTS_RXOK_EN                  0x10

static CAN_Checktype CAN_TxInit (uint8_t Module);
static CAN_Checktype CAN_RxInit (uint8_t Module);
static uint8_T TxStart=0 ;

CAN_Checktype CAN_Init(void)
{
	uint8_t LoopIndex;
	uint8_t ErrorFlag=0;
	CAN_Checktype RetVar;
	for(LoopIndex=0;(LoopIndex<CAN_GROUPS_NUMBER)&&(ErrorFlag==0);LoopIndex++)
	{
		if(CAN_ConfigParam[LoopIndex].CANId<CAN_NUMBER)
		{
			/* Enabling CAN Clock Gating Control */
			/* If CAN Id equals to 0 it initialize CAN0 and if it equals to 1 it initialize CAN1 */
			RCGC0 |=(CAN0_EN +CAN_ConfigParam[LoopIndex].CANId);

			/* setting the INIT bit in CANCTL */
			CANCTL(CAN_ConfigParam[LoopIndex].CANId)|=CANCTL_INIT;

        #if Mode_Type == Test_Mode
			/* Setting Test bit to enable CANTST register */
			CANCTL(CAN_ConfigParam[LoopIndex].CANId)|=CANCTL_TEST;
			/* Selecting which mode is used : Basic,silent or Loopback Mode */
			CANTST(CAN_ConfigParam[LoopIndex].V=CANId)|=Mode_selector;
        #endif

		 if(CAN_ConfigParam[LoopIndex].InterruptEnable==1)
		 {
			 /*Setting IE bit to enable CANINT register */
			 CANCTL (CAN_ConfigParam[loopIndex].CANId)|=CANCTL_IE
			 CANIF1MCTL(CAN_ConfigParam[LoopIndex].CANId)|=0x800;
			 //CANIF1MCTL(CAN_ConfigParam[LoopIndex].CANId)|=0x400;
		 }
	      /* Setting the CCE bit in CANCTL */
		 CANCTL(CAN_ConfigParam[LOOPIndex].CANId)|=CANCTL_CCE;
		  /*Setting the bits in CANBIT*/
		  /*Entering the value of phase 2 in TSEG2 in the register CANBIT */
		 CANBIT(CAN_ConfigParam[LoopIndex].CANId)|=((CAN_ConfigParam[LoopIndex].TSEG2)<<CANBIT_TSEG2_MASK)& CANBIT_TSEG2_EN;
		 /* Entering the value of Phase 1 and Propagation in TSEG1 in the register CANBIT */
		 CANBIT(CAN_ConfigParam[LoopIndex].CANId) |=((CAN_ConfigParam[LoopIndex].Tseg1)<< CANBIT_TSEG1_MASK )& CANBIT_TSEG1_EN;
		 /*Entering the value of sjw in the register CANBIT */
		 CANBIT(CAN_ConfigParam[LoopIndex].CANId) |= ((CAN_ConfigParam[LoopIndex].SJW) << CANBIT_SJW_MASK) & CANBIT_SJW_EN;
		 /*Entering the value of Baud rate pre-scalar in the register CANBIT*/
		 CANBIT(CAN_ConfigParam[LoopIndex].CANId) |= (CAN_ConfigParam[LoopIndex].BRP) & CANBIT_BRP_EN ;
		 /*Entering the value of Baud rate pre-scalar extension in the register CANBRPE*/
		 CANBRPE(CAN_ConfigParam[LoopIndex].CANId) |= (CAN_ConfigParam[LoopIndex].BRP) & CANBRPE_EN;
		 /* Clearing the INIT bit in CANCTL */
		 CANCTL(CAN_ConfigParam[LoopIndex].CANId) &= ~(CANCTL_INIT);

		 CAN_TxInit(MSG_ConfigParam[LoopIndex].ModuleType);
		 CAN_RxInit(MSG_ConfigParam[LoopIndex].ModuleType);

		 RetVar = CAN_OK;
		 }
		else
		{
			RetVar =CAN_NOK;
			ErrorFlag=1;
		}
		turn RetVar ;
	}
}
CAN_Checktype CAN_TxData(uint8_t ChannelID,uint8_t *DataPtr,uint8_t Module)
{
	CAN_CheckType RetVar;
	if (CAN_ConfigParam[ChannelID].CANId< CAN_NUMBER)
	{
		if (MSG_ConfigParam[Module].DLC>=0x07)
		{
			/* Putting the input data into the data registers */
			 CANIF1DA1(CAN_ConfigParam[ChannelID].CANId) |= *DataPtr;
		     CANIF1DA1(CAN_ConfigParam[ChannelID].CANId) |= (*(DataPtr + 1)) << 8;
		     CANIF1DA2(CAN_ConfigParam[ChannelID].CANId) |= *(DataPtr + 2);
		     CANIF1DA2(CAN_ConfigParam[ChannelID].CANId) |= (*(DataPtr + 3)) << 8;
		     CANIF1DB1(CAN_ConfigParam[ChannelID].CANId) |= *(DataPtr + 4);
		     CANIF1DB1(CAN_ConfigParam[ChannelID].CANId) |= (*(DataPtr + 5)) << 8;
		     CANIF1DB2(CAN_ConfigParam[ChannelID].CANId) |= *(DataPtr + 6);
		     CANIF1DB2(CAN_ConfigParam[ChannelID].CANId) |= (*(DataPtr + 7)) << 8;
		}
		else if (MSG_ConfigParam[Module].Dlc>=0x05)
		{
			 /* Putting the input data into the data registers */
			CANIF1DA1(CAN_ConfigParam[ChannelID].CANId) |= *DataPtr;
			CANIF1DA1(CAN_ConfigParam[ChannelID].CANId) |= (*(DataPtr + 1)) << 8;
			CANIF1DA2(CAN_ConfigParam[ChannelID].CANId) |= *(DataPtr + 2);
			CANIF1DA2(CAN_ConfigParam[ChannelID].CANId) |= (*(DataPtr + 3)) << 8;
			CANIF1DB1(CAN_ConfigParam[ChannelID].CANId) |= *(DataPtr + 4);
			CANIF1DB1(CAN_ConfigParam[ChannelID].CANId) |= (*(DataPtr + 5)) << 8;
	    }
		 else if(MSG_ConfigParam[Module].DLC >= 0x03)
		        {
		            /* Putting the input data into the data registers */
		            CANIF1DA1(CAN_ConfigParam[ChannelID].CANId) |= *DataPtr;
		            CANIF1DA1(CAN_ConfigParam[ChannelID].CANId) |= (*(DataPtr + 1)) << 8;
		            CANIF1DA2(CAN_ConfigParam[ChannelID].CANId) |= *(DataPtr + 2);
		            CANIF1DA2(CAN_ConfigParam[ChannelID].CANId) |= (*(DataPtr + 3)) << 8;
		        }
		 else if(MSG_ConfigParam[Module].DLC >= 0x01)
		        {
		            /* Putting the input data into the data registers */
		            CANIF1DA1(CAN_ConfigParam[ChannelID].CANId) |= *DataPtr;
		            CANIF1DA1(CAN_ConfigParam[ChannelID].CANId) |= (*(DataPtr + 1)) << 8;
	        	}
		 else
		         {
		             /* Do nothing */
		         }

		         /* Putting the message priority in MNUM bits in CANIF1CRQ */
		         /* Range of message priority: 0x01-0x20 */
		         CANIF1CRQ(CAN_ConfigParam[ChannelID].CANId) = Sort(MSG_ConfigParam[Module].ID);

		         TxStart = 1;
		         RetVar = CAN_OK;
		     }
		     else
		     {
		         RetVar = CAN_NOK;

		     }
		     return RetVar;
		 }

CAN_CheckType CAN_RxData (uint8_t ChannelID,uint16_t *DataPtr,uint8_t Module)
{
	 CAN_CheckType RetVar;
	    if(CAN_ConfigParam[ChannelID].CANId < CAN_NUMBER)
	    {
	        /* Accessing the CANIF2CMSK register */
	        /* Clearing the WRNRD bit in CANIF2CMSK to transfer the message object data to CANIF2 registers */
	        CANIF2CMSK(CAN_ConfigParam[ChannelID].CANId) &= ~(CANIFnCMSK_WRNRD);
	        /* Setting the MASK bit in CANIF2CMSK to specify whether to transfer the IDMASK, DIR and MXTD */
	        CANIF2CMSK(CAN_ConfigParam[ChannelID].CANId) |= CANIFnCMSK_MASK;
	        /* Setting the ARB bit in CANIF2CMSK to specify whether to transfer the ID, DIR, XTD and MSGVAL */
	        CANIF2CMSK(CAN_ConfigParam[ChannelID].CANId) |= CANIFnCMSK_ARB;
	        /* Setting the CONTROL bit in CANIF2CMSK to specify whether to transfer the control bits */
	        CANIF2CMSK(CAN_ConfigParam[ChannelID].CANId) |= CANIFnCMSK_CONTROL;
	        /* Setting the DATAA and DATAB bits in CANIF2CMSK to specify which bits to transfer */
	        CANIF2CMSK(CAN_ConfigParam[ChannelID].CANId) |= CANIFnCMSK_DATA_EN;
	        /* Putting the message priority in MNUM bits in CANIF1CRQ */
	                /* Range of message priority: 0x01-0x20 */
	                CANIF2CRQ(CAN_ConfigParam[ChannelID].CANId) = Sort(MSG_ConfigParam[Module].ID);

	                /* Fetching the received data into DataPtr */
	                *DataPtr = CANIF2DB2(CAN_ConfigParam[ChannelID].CANId);
	                *(DataPtr + 1) = CANIF2DB1(CAN_ConfigParam[ChannelID].CANId);
	                *(DataPtr + 2) = CANIF2DA2(CAN_ConfigParam[ChannelID].CANId);
	                *(DataPtr + 3) = CANIF2DA1(CAN_ConfigParam[ChannelID].CANId);

	                //CANIF1MCTL(CAN_ConfigParam[ChannelID].CANId) |= 0x800;
	                //CANIF2MCTL(CAN_ConfigParam[ChannelID].CANId) |= 0x400;

	                RetVar = CAN_OK;
	            }
	            else
	            {
	                RetVar = CAN_NOK;
	            }
	            return RetVar;
	        }
Status_CheckType CAN_GetStatus(uint8_t ChannelID,uint8_t MessagePriority)
{
	Status_CheckType RetVar;
	/* The CAN controller is in bus-off state */
	if(((CANSTS(CAN_ConfigParam[ChannelID].CANId)&CANSTS_BOFF_EN)>> CANSTS_BOFF_MASK)==1)
	{
		RetVar=BusOff;
	}
	/* the CAN controller is not in bus-off state */
	else
	{
		/* transmission */
		if (MSG_ConfigParam[ChannelID].RTR==1)
		{
		/* the transmitted message is in progress */
			if (TxStart==0)
			{
				RetVar=InitialState;
			}
			else
			{
				if(((CANSTS(CAN_ConfigParam[ChannelID].CANId)& CANSTS_TXOK_EN)>>CANSTS_TXOK_MASK)==0)
				{
					RetVar =TxInprogress;
				}
				/* The transmitted message is done */
				else
				{
					RetVar=TxDone;
					/* Clearing the TXOK BIT */
					CANSTS(CAN_ConfigParam[ChannelID].CANId) &= ~(CANSTS_TXOK_EN);

				}
			}
		}
		/* Reception */
		else
		{
			/*the received message is in progress */
			if(((CANSTS(CAN_ConfigParam[ChannelID].CANId)&CANSTS_RXOK_EN)>> CANSTS_RXOK_MASK)==0)
			{
				RetVar=RxInprogress;
			}
			/* the received message is done */
			else
			{
				RetVar=RxDone;
				/* Clearing the RXOK bit */
				CANSTS(CAN_ConfigParam[channelID].CANId)&= ~(CANSTS_RXOK_EN);
			}

			}
		}return RetVar ;
		void CAN_StopCurrentObject(uint8_t ChannelID)
		{
			CANCTL(ChannelID)|=CANCTL_DAR;
		}
}

}
CAN_CheckType CAN_TxInit(uint8_t Module)
{
    uint8_t LoopIndex;
    uint8_t ErrorFlag = 0;
    CAN_CheckType RetVar;
    for(LoopIndex = 0; (LoopIndex < CAN_GROUPS_NUMBER) && (ErrorFlag == 0); LoopIndex ++)
    {
        if((CAN_ConfigParam[LoopIndex].CANId < CAN_NUMBER) && ((CANIF1CRQ(CAN_ConfigParam[LoopIndex].CANId) & CANIFnCRQ_Busy) == 0))
        {
            /* Accessing the CANIF1CMSK register */
            /* Setting the WRNRD bit in CANIF1CMSK to specify a write to the register */
            CANIF1CMSK(CAN_ConfigParam[LoopIndex].CANId) |= CANIFnCMSK_WRNRD;
            /* Setting the ARB bit in CANIF1CMSK to specify whether to transfer the ID, DIR, XTD and MSGVAL */
            CANIF1CMSK(CAN_ConfigParam[LoopIndex].CANId) |= CANIFnCMSK_ARB;
            /* Setting the CONTROL bit in CANIF1CMSK to specify whether to transfer the control bits */
            CANIF1CMSK(CAN_ConfigParam[LoopIndex].CANId) |= CANIFnCMSK_CONTROL;
            /* Setting the DATAA and DATAB bits in CANIF1CMSK to specify which bits to transfer */
            CANIF1CMSK(CAN_ConfigParam[LoopIndex].CANId) |= CANIFnCMSK_DATA_EN;

            /* Accessing the CANIF1ARB2 register */
            /* Choosing type of identifier whether is it standard or extended by clearing or setting the XTD bit */
            CANIF1ARB2(CAN_ConfigParam[LoopIndex].CANId) |= MSG_ConfigParam[Module].IDE << CANIFnARB2_IDE_MASK  ;
            /* Setting the DIR bit to indicate transmit */
            CANIF1ARB2(CAN_ConfigParam[LoopIndex].CANId) |= CANIFnARB2_DIR;
            /* Setting the MSGVAL bit to indicate that the message object is valid */
            CANIF1ARB2(CAN_ConfigParam[LoopIndex].CANId) |= CANIFnARB2_MSGVAL;
            if(MSG_ConfigParam[Module].IDE == 0)
            {
                /* Input the MSGID in ID bits to put the identifier */
                CANIF1ARB2(CAN_ConfigParam[LoopIndex].CANId) |= (MSG_ConfigParam[Module].ID & CANIFnARB2_ID_STANDARD_EN) << CANIFnARB2_ID_STANDARD_MASK;
            }
            else
            {
                /* Input the MSGID in ID bits to put the identifier */
                CANIF1ARB1(CAN_ConfigParam[LoopIndex].CANId) |= (MSG_ConfigParam[Module].ID & CANIFnARB1_ID_EXTENDED_EN);
                CANIF1ARB2(CAN_ConfigParam[LoopIndex].CANId) |= (MSG_ConfigParam[Module].ID & CANIFnARB2_ID_EXTENDED_EN) >> CANIFnARB2_ID_EXTENDED_MASK;
            }

            /* Accessing the CANIF1MCTL register */
            /* Setting the DLC bits to determine the length of data */
            CANIF1MCTL(CAN_ConfigParam[LoopIndex].CANId) |= MSG_ConfigParam[Module].DLC;
            /* Setting the TXRQST bit in CANIF1MCTL to make the data available for transmission */
            CANIF1MCTL(CAN_ConfigParam[LoopIndex].CANId) |= CANIFnMCTL_TXRQST;

            /* CAN is successfully initialized and ready */
            RetVar = CAN_OK;
        }
        else if((CAN_ConfigParam[LoopIndex].CANId < CAN_NUMBER) && ((CANIF2CRQ(CAN_ConfigParam[LoopIndex].CANId) & CANIFnCRQ_Busy) == CANIFnCRQ_Busy))
        {
            /* CAN is successfully initialized but it is busy */
            RetVar = CAN_Busy;
        }
        else
        {
            /* CAN is not initialized */
            RetVar = CAN_NOK;
            ErrorFlag = 1;
        }
    }
    return RetVar;
}

CAN_CheckType CAN_RxInit(uint8_t Module)
{
    uint8_t LoopIndex;
    uint8_t ErrorFlag = 0;
    CAN_CheckType RetVar;
    for(LoopIndex = 0; (LoopIndex < CAN_GROUPS_NUMBER) && (ErrorFlag == 0); LoopIndex ++)
    {
        if((CAN_ConfigParam[LoopIndex].CANId < CAN_NUMBER) && ((CANIF2CRQ(CAN_ConfigParam[LoopIndex].CANId) & CANIFnCRQ_Busy) == 0))
        {
            /* Accessing the CANIF2CMSK register */
            /* Setting the WRNRD bit in CANIF2CMSK to specify a write to the RAM */
            CANIF2CMSK(CAN_ConfigParam[LoopIndex].CANId) |= CANIFnCMSK_WRNRD;
            /* Setting the MASK bit in CANIF2CMSK to specify whether to transfer the IDMASK, DIR and MXTD */
            CANIF2CMSK(CAN_ConfigParam[LoopIndex].CANId) |= CANIFnCMSK_MASK;
            /* Setting the ARB bit in CANIF1CMSK to specify whether to transfer the ID, DIR, XTD and MSGVAL */
            CANIF2CMSK(CAN_ConfigParam[LoopIndex].CANId) |= CANIFnCMSK_ARB;
            /* Setting the CONTROL bit in CANIF1CMSK to specify whether to transfer the control bits */
            CANIF2CMSK(CAN_ConfigParam[LoopIndex].CANId) |= CANIFnCMSK_CONTROL;
            /* Setting the DATAA and DATAB bits in CANIF2CMSK to specify which bits to transfer */
            CANIF2CMSK(CAN_ConfigParam[LoopIndex].CANId) |= CANIFnCMSK_DATA_EN;

            /* Accessing the CANIF2ARB2 register */
            /* Choosing identifier type whether standard or extended by clearing or setting XTD bit */
            CANIF2ARB2(CAN_ConfigParam[LoopIndex].CANId) |= MSG_ConfigParam[Module].IDE << CANIFnARB2_IDE_MASK;
            /* Clearing the DIR bit to indicate received */
            CANIF2ARB2(CAN_ConfigParam[LoopIndex].CANId) &= ~(CANIFnARB2_DIR);
            /* Setting the MSGVAL bit to indicate that the message object is valid */
            CANIF2ARB2(CAN_ConfigParam[LoopIndex].CANId) |= CANIFnARB2_MSGVAL;
            if(MSG_ConfigParam[Module].IDE == 0)
            {
                /* Input the MSGID in ID bits to put the identifier */
                CANIF2ARB2(CAN_ConfigParam[LoopIndex].CANId) |= (MSG_ConfigParam[Module].ID & CANIFnARB2_ID_STANDARD_EN ) << CANIFnARB2_ID_STANDARD_MASK;
            }
            else
            {
                /* Input the MSGID in ID bits to put the identifier */
                CANIF2ARB1(CAN_ConfigParam[LoopIndex].CANId) |= (MSG_ConfigParam[Module].ID & CANIFnARB1_ID_EXTENDED_EN);
                CANIF2ARB2(CAN_ConfigParam[LoopIndex].CANId) |= (MSG_ConfigParam[Module].ID & CANIFnARB2_ID_EXTENDED_EN) >> CANIFnARB2_ID_EXTENDED_MASK;
            }

            /* Accessing the CANIF2MCTL register */
            /* Setting the UMASK bit to enable the mask (MSK, MXTD and MDIR) */
            CANIF2MCTL(CAN_ConfigParam[LoopIndex].CANId) |= CANIFnMCTL_UMASK;
            /* Setting the EOB bit for a single message object */
            CANIF2MCTL(CAN_ConfigParam[LoopIndex].CANId) |= CANIFnMCTL_EOB;
            /* Setting the DLC bits to determine the length of data */
            CANIF2MCTL(CAN_ConfigParam[LoopIndex].CANId) |= MSG_ConfigParam[Module].DLC;

            /* CAN is successfully initialized and ready */
            RetVar = CAN_OK;
        }
        else if((CAN_ConfigParam[LoopIndex].CANId < CAN_NUMBER) && ((CANIF2CRQ(CAN_ConfigParam[LoopIndex].CANId) & CANIFnCRQ_Busy) == CANIFnCRQ_Busy))
        {
            /* CAN is successfully initialized but it is busy */
            RetVar = CAN_Busy;
        }
        else
        {
            /* CAN is not initialized */
            RetVar = CAN_NOK;
            ErrorFlag = 1;
        }
    }
    return RetVar;
}
