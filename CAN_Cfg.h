/*
 * CAN_Cfg.h
 *
 *  Created on: Nov 11, 2018
 *      Author: ziad
 */

#ifndef CAN_CFG_H_
#define CAN_CFG_H_

#define CAN_GROUPS_NUMBER       1u
#define Message_Groups_Number   2u

#define Normal_Mode                 1
#define Test_Mode                   2
#define Mode_Type                   Test_Mode

#define Basic_Mode                  0x04
#define Silent_Mode                 0x08
#define Loopback_Mode               0x10
#define Mode_Selector               Loopback_Mode


#endif /* CAN_CFG_H_ */
