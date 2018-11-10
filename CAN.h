/*
 * CAN.h
 *
 *  Created on: Nov 11, 2018
 *      Author: ziad
 */

#ifndef CAN_H_
#define CAN_H_

#include <stdint.h>
#include "CAN_Cfg.h"


typedef struct
{
    uint8_t CANId;  /* To choose CAN0 or CAN1 */
    uint8_t TSEG2;  /* Phase 2 */
    uint8_t TSEG1;  /* Propagation + Phase 1 */
    uint8_t SJW;    /* Synchronization Jump Width */
    uint16_t BRP;   /* Baud Rate Pre-scalar */
    uint8_t InterruptEnable;

}CAN_CfgType;

typedef struct
{
    uint32_t ID;
    uint8_t RTR;
    uint8_t IDE;
    uint8_t DLC;
    uint8_t ModuleType;
}MessageParameters;



typedef enum {CAN_OK = 1, CAN_NOK = 0, CAN_Busy = 2} CAN_CheckType;
typedef enum {InitialState = 5 ,BusOff = 0, TxDone = 1, TxInprogress = 2, RxDone = 3, RxInprogress = 4} Status_CheckType;

extern const CAN_CfgType CAN_ConfigParam [CAN_GROUPS_NUMBER];
extern const MessageParameters MSG_ConfigParam [Message_Groups_Number];

CAN_CheckType CAN_Init(void);
CAN_CheckType CAN_TxData(uint8_t ChannelID, uint8_t* DataPtr, uint8_t Module);
CAN_CheckType CAN_RxData(uint8_t ChannelID, uint16_t* DataPtr, uint8_t Module);
Status_CheckType CAN_GetStatus(uint8_t ChannelID, uint8_t MessagePriority);
void CAN_StopCurrentObject(uint8_t ChannelID);

#endif


#endif /* CAN_H_ */
