/*
 * Utilities.c
 *
 *  Created on: Nov 12, 2018
 *      Author: ziad
 */
#include <stdint.h>
#include "Utilities.h"
#include "CAN.h"
#include "CAN_Cfg.h"

uint8_t Sort (uint8_t ID)
{
	uint8_t LoopIndex1;
	uint8_t LoopIndex2;
	uint8_t sort[Message_Groups_Number];
	uint8_t temp;
	/* Storing the values of all identifiers in an array */
	for(LoopIndex1=0;LoopIndex1<Message_Groups_Number;LoopIndex1++)
	{
		sort[LoopIndex1]=MSG_ConfigParam[LoopIndex1].ID;
	}
	/* Sorting the identifiers in an ascending order */
	for(LoopIndex1=0;LoopIndex1<Message_Groups_Number;LoopIndex1++)
	{
		for(LoopIndex2=1;LoopIndex2<Message_Groups_Number;LoopIndex2++)
		{
			if(sort[LoopIndex1]>sort[LoopIndex2])
			{
				temp=sort[loopIndex1];
				sort[LoopIndex1]=sort[LoopIndex2];
				sort[LoopIndex2]=temp;
			}
			else
			{
				/* Do nothing */
			}
		}
	}
	/* comparing the input identifier with the list of identifiers and returning the corresponding priority */
	LoopIndex1=0;
	while(sort[LoopIndex1] !=ID)
	{
		LoopIndex1++;
	}
	return (LoopIndex1+1)
}



