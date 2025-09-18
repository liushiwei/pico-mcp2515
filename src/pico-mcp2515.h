#ifndef _PIC_CAN_H_
#define _PIC_CAN_H_

#include <stdio.h>

enum
{
	PKG_STATE_STRART,
    PKG_STATE_ID,
    PKG_STATE_RTR,
    PKG_STATE_IDE,
    PKG_STATE_DATA,
};

typedef struct {
  long id;
  uint8_t rtr;
  uint8_t ide;
  uint8_t dlc;
  uint8_t dataArray[20];
} packet_t;

void printPacket(int channel ,packet_t * packet);
#endif