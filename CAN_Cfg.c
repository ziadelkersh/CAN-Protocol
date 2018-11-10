/*
 * CAN_Cfg.c
 *
 *  Created on: Nov 11, 2018
 *      Author: ziad
 */
#include "CAN.h"
#include "CAN_Cfg.h"

const CAN_CfgType CAN_ConfigParam [CAN_GROUPS_NUMBER] =
{
    {
         0x00,
         0x03,
         0x04,
         0x03,
         0x31,
         0
    }
};

const MessageParameters MSG_ConfigParam [Message_Groups_Number]=
{
    {
        0x12,
        1,
        0,
        0x04,
        0x00
    },
    {
        0x1234,
        1,
        1,
        0x08,
        0x01
    }
};



